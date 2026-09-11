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

O caminho cru do A* é uma **escada**: numa grade de 10 cm com 8 vizinhos, o rumo só existe em
múltiplos de 45 graus. Seguir isso faz o robô serpentear, então o caminho passa por três etapas
antes de sair daqui: encurtamento por visada livre (junta pontos que se enxergam), arredondamento
de quina (Chaikin) e reamostragem uniforme. O encurtamento nunca aproxima o caminho mais da parede
do que o A* já havia aceitado, senão a suavização comeria a folga de segurança.
"""
import heapq
import math

import cv2
import numpy as np

VIZINHOS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]


class GridPlanner:
    def __init__(self, robot_radius=0.45, clearance_margin=0.35, plan_resolution=0.10,
                 allow_unknown=True, unknown_penalty=2.5, soft_cost=2.5,
                 smooth=True, smooth_iterations=2, sample_step=0.15):
        self.radius = float(robot_radius)
        self.clearance = float(clearance_margin)
        self.plan_res = float(plan_resolution)
        self.allow_unknown = bool(allow_unknown)
        self.unknown_pen = float(unknown_penalty)
        self.soft_cost = float(soft_cost)
        self.smooth = bool(smooth)
        self.smooth_iterations = int(smooth_iterations)
        self.sample_step = float(sample_step)
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

    # ------------------------------------------------------------------ suavização
    @staticmethod
    def celulas_da_reta(a, b):
        """Bresenham: as células que uma reta entre dois centros de célula atravessa."""
        (y0, x0), (y1, x1) = a, b
        dy, dx = abs(y1 - y0), abs(x1 - x0)
        sy, sx = (1 if y1 > y0 else -1), (1 if x1 > x0 else -1)
        err = dx - dy
        saida = []
        while True:
            saida.append((y0, x0))
            if (y0, x0) == (y1, x1):
                return saida
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x0 += sx
            if e2 < dx:
                err += dx; y0 += sy

    def visada_livre(self, blocked, cost, a, b, custo_max):
        """Verdadeiro se dá para ir reto de `a` a `b` sem entrar em célula proibida **e** sem
        chegar mais perto da parede do que o caminho original já chegava (`custo_max`)."""
        for cel in self.celulas_da_reta(a, b):
            if blocked[cel] or float(cost[cel]) > custo_max + 1e-3:
                return False
        return True

    def encurtar(self, celulas, blocked, cost):
        """Junta pontos que se enxergam: a escada de 45 graus vira poucos trechos retos."""
        if len(celulas) < 3:
            return celulas
        saida = [celulas[0]]
        i = 0
        while i < len(celulas) - 1:
            melhor = i + 1
            for j in range(len(celulas) - 1, i, -1):
                custo_max = max(float(cost[c]) for c in celulas[i:j + 1])
                if self.visada_livre(blocked, cost, celulas[i], celulas[j], custo_max):
                    melhor = j
                    break
            saida.append(celulas[melhor])
            i = melhor
        return saida

    @staticmethod
    def _livre_no_mundo(blocked, eff_res, origin, p):
        iy = int((p[1] - origin[1]) / eff_res)
        ix = int((p[0] - origin[0]) / eff_res)
        h, w = blocked.shape
        return 0 <= iy < h and 0 <= ix < w and not blocked[iy, ix]

    def arredondar(self, pts, blocked, eff_res, origin):
        """Chaikin: cada quina vira dois pontos a 1/4 e 3/4 do trecho. Ponto que cairia em célula
        proibida é descartado — melhor um canto vivo do que raspar a parede."""
        for _ in range(self.smooth_iterations):
            if len(pts) < 3:
                break
            novo = [pts[0]]
            for i in range(len(pts) - 1):
                (x0, y0), (x1, y1) = pts[i], pts[i + 1]
                for a in (0.25, 0.75):
                    c = (x0 + (x1 - x0) * a, y0 + (y1 - y0) * a)
                    if self._livre_no_mundo(blocked, eff_res, origin, c):
                        novo.append(c)
                    else:
                        novo.append(pts[i] if a < 0.5 else pts[i + 1])
            novo.append(pts[-1])
            pts = novo
        return pts

    def reamostrar(self, pts):
        """Espaçamento uniforme: a perseguição mede distância ao longo do caminho, e trecho
        desigual faria o ponto perseguido saltar."""
        if len(pts) < 2:
            return pts
        passo = self.sample_step
        saida = [pts[0]]
        sobra = 0.0                       # quanto já se andou desde o último ponto emitido
        for i in range(1, len(pts)):
            (x0, y0), (x1, y1) = pts[i - 1], pts[i]
            d = math.hypot(x1 - x0, y1 - y0)
            if d < 1e-9:
                continue
            andado = 0.0
            while sobra + (d - andado) >= passo:
                andado += passo - sobra
                saida.append((x0 + (x1 - x0) * andado / d, y0 + (y1 - y0) * andado / d))
                sobra = 0.0
            sobra += d - andado
        if math.hypot(saida[-1][0] - pts[-1][0], saida[-1][1] - pts[-1][1]) > 1e-3:
            saida.append(pts[-1])
        return saida

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
        if self.smooth and len(caminho) > 2:
            caminho = self.encurtar(caminho, blocked, cost)
        pts = [self.cell_to_world(iy, ix, eff_res, origin) for iy, ix in caminho]
        if self.smooth and len(pts) > 2:
            pts = self.arredondar(pts, blocked, eff_res, origin)
            pts = self.reamostrar(pts)
        return pts

    @staticmethod
    def comprimento(caminho):
        return sum(math.hypot(caminho[i][0] - caminho[i - 1][0], caminho[i][1] - caminho[i - 1][1])
                   for i in range(1, len(caminho)))
