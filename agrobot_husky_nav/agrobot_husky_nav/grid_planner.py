#!/usr/bin/env python3
"""grid_planner — planejamento A* sobre o OccupancyGrid do SLAM.

Extraído do nó de exploração para ser usado também pela navegação até marcadores. A ideia é
simples: o mapa fino do `slam_toolbox` (5 cm) é grosso demais para um A* rodando a cada 2 s, e
fino demais para o que importa, então ele é **reamostrado pelo pior caso** (se qualquer célula
fina dentro do bloco é obstáculo, o bloco inteiro é obstáculo) para uma grade de planejamento.

Sobre essa grade valem duas zonas ao redor de cada obstáculo:

* **rígida** (`robot_radius`): proibida, é onde o robô bateria;
* **suave** (`+ clearance_margin`): permitida, mas cara, para o caminho preferir o meio do corredor.

O desconhecido é atravessável com penalidade (`unknown_penalty`), senão o robô nunca sairia de
uma sala recém-mapeada. Quem chama decide o destino; aqui só se responde "por onde".
"""
import heapq
import math

import cv2
import numpy as np

VIZINHOS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]


class GridPlanner:
    def __init__(self, robot_radius=0.45, clearance_margin=0.35, plan_resolution=0.10,
                 allow_unknown=True, unknown_penalty=2.5, soft_cost=2.5):
        self.radius = float(robot_radius)
        self.clearance = float(clearance_margin)
        self.plan_res = float(plan_resolution)
        self.allow_unknown = bool(allow_unknown)
        self.unknown_pen = float(unknown_penalty)
        self.soft_cost = float(soft_cost)
        self.explored_m2 = 0.0

    # ------------------------------------------------------------------ grade de custo
    def build_cost(self, m):
        """`m` é um nav_msgs/OccupancyGrid. Devolve (custo, proibido, resolução, origem)."""
        res = m.info.resolution
        grid = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        self.explored_m2 = float((grid >= 0).sum()) * res * res
        step = max(1, int(round(self.plan_res / res)))
        occ = (grid == 100).astype(np.uint8)
        unk = (grid == -1).astype(np.uint8)
        k = np.ones((step, step), np.uint8)
        occ_c = cv2.dilate(occ, k)[::step, ::step]
        unk_c = (cv2.erode(1 - unk, k)[::step, ::step] == 0).astype(np.uint8)
        eff_res = res * step
        r_hard = max(1, int(round(self.radius / eff_res)))
        r_soft = max(r_hard + 1, int(round((self.radius + self.clearance) / eff_res)))
        hard = cv2.dilate(occ_c, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_hard + 1,) * 2))
        soft = cv2.dilate(occ_c, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_soft + 1,) * 2))
        cost = np.ones(occ_c.shape, dtype=np.float32)
        cost[soft > 0] = self.soft_cost
        if self.allow_unknown:
            cost[unk_c > 0] = np.maximum(cost[unk_c > 0], self.unknown_pen)
        blocked = (hard > 0) | ((unk_c > 0) & (not self.allow_unknown))
        return cost, blocked, eff_res, (m.info.origin.position.x, m.info.origin.position.y)

    @staticmethod
    def world_to_cell(x, y, eff_res, origin):
        return (int((y - origin[1]) / eff_res), int((x - origin[0]) / eff_res))

    @staticmethod
    def cell_to_world(iy, ix, eff_res, origin):
        return (origin[0] + (ix + 0.5) * eff_res, origin[1] + (iy + 0.5) * eff_res)

    @staticmethod
    def celula_livre_proxima(blocked, end, raio_max=8):
        """Anel a anel ao redor de `end`, devolve a primeira célula fora da zona rígida."""
        h, w = blocked.shape
        if 0 <= end[0] < h and 0 <= end[1] < w and not blocked[end]:
            return end
        for rr in range(1, raio_max + 1):
            melhor, melhor_d = None, 1e18
            for dy in range(-rr, rr + 1):
                for dx in range(-rr, rr + 1):
                    if max(abs(dy), abs(dx)) != rr:
                        continue
                    ny, nx = end[0] + dy, end[1] + dx
                    if 0 <= ny < h and 0 <= nx < w and not blocked[ny, nx]:
                        d = dy * dy + dx * dx
                        if d < melhor_d:
                            melhor, melhor_d = (ny, nx), d
            if melhor is not None:
                return melhor
        return None

    # ------------------------------------------------------------------ A*
    def plan(self, m, pose, goal, snap_goal=True):
        """Caminho [(x, y)] no frame do mapa, de `pose` a `goal`, ou None se não houver."""
        cost, blocked, eff_res, origin = self.build_cost(m)
        h, w = cost.shape
        start = self.world_to_cell(pose[0], pose[1], eff_res, origin)
        end = self.world_to_cell(goal[0], goal[1], eff_res, origin)
        if not (0 <= start[0] < h and 0 <= start[1] < w and 0 <= end[0] < h and 0 <= end[1] < w):
            return None
        # o robô pode estar encostado, dentro da própria zona inflada: libera a vizinhança da partida
        blocked = blocked.copy()
        y0, x0 = start
        blocked[max(0, y0 - 2):y0 + 3, max(0, x0 - 2):x0 + 3] = False
        if blocked[end]:
            if not snap_goal:
                return None
            end = self.celula_livre_proxima(blocked, end)
            if end is None:
                return None
        gscore = {start: 0.0}
        came = {}
        openh = [(0.0, start)]
        visited = set()
        while openh:
            _, cur = heapq.heappop(openh)
            if cur in visited:
                continue
            visited.add(cur)
            if cur == end:
                break
            cy, cx = cur
            gc = gscore[cur]
            for dy, dx, passo in VIZINHOS:
                ny, nx = cy + dy, cx + dx
                if not (0 <= ny < h and 0 <= nx < w) or blocked[ny, nx]:
                    continue
                ng = gc + passo * float(cost[ny, nx])
                if ng < gscore.get((ny, nx), 1e18):
                    gscore[(ny, nx)] = ng
                    came[(ny, nx)] = cur
                    heapq.heappush(openh, (ng + math.hypot(ny - end[0], nx - end[1]), (ny, nx)))
        if end not in came and end != start:
            return None
        caminho = [end]
        while caminho[-1] != start:
            caminho.append(came[caminho[-1]])
        caminho.reverse()
        return [self.cell_to_world(iy, ix, eff_res, origin) for iy, ix in caminho]

    @staticmethod
    def comprimento(caminho):
        return sum(math.hypot(caminho[i][0] - caminho[i - 1][0], caminho[i][1] - caminho[i - 1][1])
                   for i in range(1, len(caminho)))
