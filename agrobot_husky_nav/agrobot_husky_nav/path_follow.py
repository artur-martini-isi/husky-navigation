#!/usr/bin/env python3
"""path_follow — perseguição de ponto à frente (pure pursuit) com curvatura e rampa.

Compartilhado por `goto_point` e `explore`. A primeira versão comandava `w = k * erro_de_rumo`, o
que faz o robô **serpentear**: o erro de rumo cai a zero só quando o robô aponta exatamente para o
ponto perseguido, então ele passa do ponto, corrige para o outro lado e repete. Pure pursuit de
verdade resolve isso geometricamente: existe um arco de círculo que sai da posição atual, com o
rumo atual, e chega no ponto perseguido. Sua curvatura é

    κ = 2·sen(α) / L      (α = ângulo até o ponto perseguido, L = distância até ele)

e basta comandar `w = v·κ`. O robô entra na curva em vez de caçá-la.

Três ajustes completam a suavidade:

* **distância adaptativa**: o ponto perseguido se afasta com a velocidade, então parado ele é
  preciso e andando ele é suave (`lookahead_min + lookahead_gain·v`);
* **rampa**: `max_linear_accel` e `max_angular_accel` limitam a variação por ciclo, porque degrau
  de velocidade vira solavanco e escorrega a roda;
* **histerese no giro parado**: entra acima de `turn_in_place_angle`, só sai abaixo de
  `turn_resume_angle`. Sem isso o robô fica alternando entre girar e andar bem na fronteira.
"""
import math


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PurePursuit:
    def __init__(self, max_linear=0.3, min_linear=0.08, max_angular=0.7, k_angular=1.3,
                 lookahead_min=0.5, lookahead_gain=1.2, lookahead_max=1.2,
                 turn_in_place_angle=0.7, turn_resume_angle=0.25,
                 max_linear_accel=0.4, max_angular_accel=1.5,
                 curvature_slowdown=1.2, control_rate=10.0):
        self.max_lin, self.min_lin = float(max_linear), float(min_linear)
        self.max_ang, self.k_ang = float(max_angular), float(k_angular)
        self.ld_min, self.ld_gain = float(lookahead_min), float(lookahead_gain)
        self.ld_max = float(lookahead_max)
        self.turn_ang, self.resume_ang = float(turn_in_place_angle), float(turn_resume_angle)
        self.a_lin, self.a_ang = float(max_linear_accel), float(max_angular_accel)
        self.curv_slow = float(curvature_slowdown)
        self.dt = 1.0 / float(control_rate)
        self.v = self.w = 0.0
        self.girando = False
        self.alvo = None
        self.ld = self.ld_min

    def reset(self):
        self.v = self.w = 0.0
        self.girando = False
        self.alvo = None

    # ------------------------------------------------------------------ ponto perseguido
    def distancia_perseguida(self):
        return max(self.ld_min, min(self.ld_max, self.ld_min + self.ld_gain * self.v))

    def ponto(self, pose, path):
        """Ponto do caminho a `distancia_perseguida()` à frente; devolve também o caminho podado.

        O trecho já percorrido é descartado a partir do ponto mais próximo, senão o robô
        perseguiria eternamente o começo do caminho.
        """
        if not path:
            return None, path
        d = [math.hypot(x - pose[0], y - pose[1]) for x, y in path]
        i0 = min(range(len(d)), key=d.__getitem__)
        path = path[i0:]
        self.ld = self.distancia_perseguida()
        acc = 0.0
        for i in range(1, len(path)):
            acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if acc >= self.ld:
                return path[i], path
        return path[-1], path

    # ------------------------------------------------------------------ comando
    def rampa(self, v_alvo, w_alvo):
        self.v += max(-self.a_lin * self.dt, min(self.a_lin * self.dt, v_alvo - self.v))
        self.w += max(-self.a_ang * self.dt, min(self.a_ang * self.dt, w_alvo - self.w))
        if abs(self.v) < 1e-3:
            self.v = 0.0
        if abs(self.w) < 1e-3:
            self.w = 0.0
        return self.v, self.w

    def parar(self):
        return self.rampa(0.0, 0.0)

    def comando(self, pose, path, dist_goal=None, escala=1.0):
        """(v, w) para seguir `path`, mais o caminho podado. `escala` reduz a velocidade por
        motivo externo (obstáculo à frente), sem mexer na geometria da curva."""
        alvo, path = self.ponto(pose, path)
        self.alvo = alvo
        if alvo is None:
            return self.parar() + (path,)
        dx, dy = alvo[0] - pose[0], alvo[1] - pose[1]
        L = math.hypot(dx, dy)
        alfa = wrap(math.atan2(dy, dx) - pose[2])

        # muito torto para curvar: gira parado, com histerese para não ficar alternando
        if abs(alfa) > (self.resume_ang if self.girando else self.turn_ang):
            self.girando = True
            w = max(-self.max_ang, min(self.max_ang, self.k_ang * alfa))
            v, w = self.rampa(0.0, w)
            return v, w, path
        self.girando = False

        kappa = 0.0 if L < 1e-3 else 2.0 * math.sin(alfa) / L
        v = self.max_lin / (1.0 + self.curv_slow * abs(kappa))   # curva fechada, mais devagar
        v = max(v, self.min_lin)                                  # piso: abaixo disso não anda
        if dist_goal is not None:                                 # desacelera na chegada
            v = min(v, max(self.min_lin, 0.8 * dist_goal))
        v *= max(0.0, min(1.0, escala))                           # freio externo pode zerar
        v = min(v, self.max_lin)
        w = v * kappa
        if abs(w) > self.max_ang:                                 # respeita o arco: baixa v, não w
            v *= self.max_ang / abs(w)
            w = math.copysign(self.max_ang, w)
        v, w = self.rampa(v, w)
        return v, w, path
