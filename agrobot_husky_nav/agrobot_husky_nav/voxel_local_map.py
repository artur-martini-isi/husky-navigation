#!/usr/bin/env python3
"""voxel_local_map — mapa local para navegação indoor a partir do Livox MID360.

O MID360 tem varredura **não repetitiva**: cada quadro cobre pouco, e é a soma de vários
quadros que preenche a cena. Este nó acumula alguns décimos de segundo de nuvem,
**compensando o movimento do robô pela odometria**, voxeliza o resultado e publica duas
saídas: a nuvem voxelizada (para ver) e uma grade de ocupação local (para navegar).

    /livox/lidar ──► transforma p/ base_link ──► acumula (compensado por odometria)
                 ──► voxeliza (quantização + únicos) ──┬─► <ns>/voxel_cloud  (PointCloud2)
                                                       └─► <ns>/local_map    (OccupancyGrid)

A grade é uma janela rolante centrada no robô: célula **ocupada** quando há voxel dentro da
faixa de altura do robô, **livre** quando só há retorno de chão, e **desconhecida** quando não
chegou nada. É o formato que um planejador local consome direto.

Alturas são em `base_link`, cujo z=0 fica na altura do eixo das rodas: o chão está em
z ≈ -0,165 m. Por isso a faixa de obstáculo padrão começa em -0,02 (logo acima do chão).
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def rpy_matrix(roll, pitch, yaw):
    """Mesma convenção do static_transform_publisher: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class VoxelLocalMap(Node):
    def __init__(self):
        super().__init__('voxel_local_map')
        p = self.declare_parameter
        p('input_topic', '/livox/lidar')
        p('odom_topic', 'platform/odom/filtered')
        p('map_frame', 'odom')            # odom (recomendado) ou base_link
        p('base_frame', 'base_link')
        # montagem do MID360 (mesmos valores do nav.launch.py): invertido e girado 90 graus
        p('livox_xyz', [-0.168, 0.0, 0.45])
        p('livox_rpy', [math.pi, 0.0, -math.pi / 2])
        p('voxel_size', 0.10)
        p('accumulate_s', 0.6)
        p('min_range', 0.6)               # abaixo disso é a própria estrutura do robô
        p('max_range', 12.0)
        # Classificação por **plano de chão ajustado** em vez de altura fixa: o conjunto
        # sensor+piso tem ~1 grau de inclinação, o que a 8 m viraria 14 cm de erro.
        p('obstacle_min_height', 0.15)    # altura acima do chão para contar como obstáculo
        p('obstacle_max_height', 1.80)    # acima disso é teto/luminária
        p('ground_max_height', 0.08)      # até aqui é chão -> célula livre
        # Filtro do próprio robô: o MID360 olha para baixo e enxerga o top plate e o arco.
        # A300 tem ~0,99 x 0,67 m; a caixa abaixo cobre o corpo com folga.
        p('self_filter_x', [-0.62, 0.62])
        p('self_filter_y', [-0.46, 0.46])
        p('self_filter_z_max', 0.90)
        p('grid_size_m', 20.0)
        p('grid_resolution', 0.10)
        p('inflation_m', 0.0)             # 0 = sem inflar; use ~raio do robô para planejar
        p('publish_rate', 5.0)
        p('publish_voxel_cloud', True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame = str(g('map_frame'))
        self.base_frame = str(g('base_frame'))
        self.voxel = float(g('voxel_size'))
        self.acc_s = float(g('accumulate_s'))
        self.min_r, self.max_r = float(g('min_range')), float(g('max_range'))
        self.h_obst_min = float(g('obstacle_min_height'))
        self.h_obst_max = float(g('obstacle_max_height'))
        self.h_ground = float(g('ground_max_height'))
        self.fx = [float(v) for v in g('self_filter_x')]
        self.fy = [float(v) for v in g('self_filter_y')]
        self.fz = float(g('self_filter_z_max'))
        self.plane = np.array([0.0, 0.0, -0.165])   # z = a*x + b*y + c, começa no valor nominal
        self.grid_m, self.res = float(g('grid_size_m')), float(g('grid_resolution'))
        self.inflation = float(g('inflation_m'))
        self.publish_cloud = bool(g('publish_voxel_cloud'))
        self.cells = int(round(self.grid_m / self.res))

        xyz = [float(v) for v in g('livox_xyz')]
        rpy = [float(v) for v in g('livox_rpy')]
        self.R = rpy_matrix(*rpy)
        self.t = np.array(xyz)

        self.buf = deque()                # (t, pontos_base_link Nx3, pose (x,y,yaw))
        self.pose = None                  # pose atual em odom
        self.pose_time = 0.0
        self.n_clouds = self.n_maps = 0
        self.last_stats = ''

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(PointCloud2, str(g('input_topic')), self.on_cloud, sensor_qos)
        self.create_subscription(Odometry, str(g('odom_topic')), self.on_odom, sensor_qos)
        self.pub_cloud = self.create_publisher(PointCloud2, 'voxel_cloud', 1)
        self.pub_map = self.create_publisher(OccupancyGrid, 'local_map', 1)
        self.create_timer(1.0 / float(g('publish_rate')), self.publish)
        self.create_timer(10.0, self.log_summary)
        self.get_logger().info(
            'mapa local: voxel %.2f m, janela %.1f s, grade %.0f x %.0f m a %.2f m/celula, quadro "%s"'
            % (self.voxel, self.acc_s, self.grid_m, self.grid_m, self.res, self.map_frame))

    # ------------------------------------------------------------------ entradas
    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_time = time.monotonic()

    def on_cloud(self, msg: PointCloud2):
        pts = self.read_xyz(msg)
        if pts is None or len(pts) == 0:
            return
        # sensor -> base_link
        pts = pts @ self.R.T + self.t
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = (r > self.min_r) & (r < self.max_r)
        # remove o corpo do robô (caixa em base_link), não só um raio
        body = ((pts[:, 0] > self.fx[0]) & (pts[:, 0] < self.fx[1]) &
                (pts[:, 1] > self.fy[0]) & (pts[:, 1] < self.fy[1]) & (pts[:, 2] < self.fz))
        pts = pts[keep & ~body]
        if len(pts) == 0:
            return
        self.n_clouds += 1
        self.buf.append((time.monotonic(), pts.astype(np.float32), self.pose))
        cutoff = time.monotonic() - self.acc_s
        while self.buf and self.buf[0][0] < cutoff:
            self.buf.popleft()

    @staticmethod
    def read_xyz(msg: PointCloud2):
        """Lê x,y,z de um PointCloud2 sem depender do sensor_msgs_py."""
        names = [f.name for f in msg.fields]
        if not all(n in names for n in ('x', 'y', 'z')):
            return None
        offs = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        n = len(raw) // step
        raw = raw[:n * step].reshape(n, step)
        out = np.empty((n, 3), dtype=np.float32)
        for i, axis in enumerate(('x', 'y', 'z')):
            o = offs[axis]
            out[:, i] = raw[:, o:o + 4].copy().view(np.float32).ravel()
        good = np.isfinite(out).all(axis=1)
        return out[good]

    # ------------------------------------------------------------------ saída
    def accumulated_points(self):
        """Junta a janela, levando as nuvens antigas para a pose atual do robô."""
        if not self.buf:
            return None
        cur = self.pose
        chunks = []
        for _, pts, pose in self.buf:
            if cur is not None and pose is not None:
                dx, dy = pose[0] - cur[0], pose[1] - cur[1]
                dyaw = pose[2] - cur[2]
                c, s = math.cos(dyaw), math.sin(dyaw)
                # roda a nuvem antiga para a orientação atual e desloca pela diferença de pose
                x = pts[:, 0] * c - pts[:, 1] * s
                y = pts[:, 0] * s + pts[:, 1] * c
                cy, sy = math.cos(-cur[2]), math.sin(-cur[2])
                x = x + (dx * cy - dy * sy)
                y = y + (dx * sy + dy * cy)
                chunks.append(np.column_stack((x, y, pts[:, 2])).astype(np.float32))
            else:
                chunks.append(pts)
        return np.vstack(chunks)

    def publish(self):
        pts = self.accumulated_points()
        if pts is None or len(pts) == 0:
            return
        # ---------------------------------------------------------------- voxelização
        q = np.floor(pts / self.voxel).astype(np.int32)
        _, idx = np.unique(q, axis=0, return_index=True)
        vox = (q[idx].astype(np.float32) + 0.5) * self.voxel
        stamp = self.get_clock().now().to_msg()

        if self.publish_cloud:
            self.pub_cloud.publish(self.make_cloud(vox, stamp, self.base_frame))

        # ---------------------------------------------------------------- grade de ocupação
        half = self.grid_m / 2.0
        self.fit_ground(vox)
        a, b, c = self.plane
        height = vox[:, 2] - (a * vox[:, 0] + b * vox[:, 1] + c)   # altura acima do chão
        obst = vox[(height >= self.h_obst_min) & (height <= self.h_obst_max)]
        ground = vox[height < self.h_ground]
        grid = np.full((self.cells, self.cells), -1, dtype=np.int8)

        def stamp_cells(sel, value):
            if len(sel) == 0:
                return
            ix = ((sel[:, 0] + half) / self.res).astype(np.int32)
            iy = ((sel[:, 1] + half) / self.res).astype(np.int32)
            ok = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
            grid[iy[ok], ix[ok]] = value

        stamp_cells(ground, 0)
        stamp_cells(obst, 100)

        if self.inflation > 0:
            grid = self.inflate(grid)

        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id=self.base_frame)
        msg.info.resolution = self.res
        msg.info.width = msg.info.height = self.cells
        msg.info.origin.position.x = -half
        msg.info.origin.position.y = -half
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.pub_map.publish(msg)
        self.n_maps += 1
        self.last_stats = ('%d pontos -> %d voxels | ocupadas %d, livres %d | chão: %+.1f graus, %.3f m'
                           % (len(pts), len(vox), int((grid == 100).sum()), int((grid == 0).sum()),
                              math.degrees(math.atan(-self.plane[0])), self.plane[2]))

    def fit_ground(self, vox):
        """Ajusta z = a*x + b*y + c aos pontos baixos e suaviza entre ciclos."""
        a, b, c = self.plane
        h = vox[:, 2] - (a * vox[:, 0] + b * vox[:, 1] + c)
        seed = vox[(h > -0.25) & (h < 0.20)]          # candidatos a chão perto do plano atual
        if len(seed) < 200:
            return
        A = np.column_stack([seed[:, 0], seed[:, 1], np.ones(len(seed))])
        try:
            coef, *_ = np.linalg.lstsq(A, seed[:, 2], rcond=None)
        except np.linalg.LinAlgError:
            return
        if abs(coef[0]) > 0.15 or abs(coef[1]) > 0.15 or not (-0.6 < coef[2] < 0.2):
            return                                    # ajuste implausível: mantém o anterior
        self.plane = 0.7 * self.plane + 0.3 * np.asarray(coef)

    def inflate(self, grid):
        """Dilata as células ocupadas pelo raio de inflação (só numpy, sem scipy)."""
        r = int(round(self.inflation / self.res))
        if r <= 0:
            return grid
        occ = (grid == 100)
        out = occ.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                out |= np.roll(np.roll(occ, dy, axis=0), dx, axis=1)
        grid = grid.copy()
        grid[out] = 100
        return grid

    @staticmethod
    def make_cloud(points, stamp, frame_id):
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=frame_id)
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def log_summary(self):
        odom = 'ok' if self.pose is not None and time.monotonic() - self.pose_time < 2.0 else 'SEM ODOM'
        self.get_logger().info('nuvens=%d mapas=%d | janela=%d quadros | odom=%s | %s'
                               % (self.n_clouds, self.n_maps, len(self.buf), odom, self.last_stats))


def main(args=None):
    rclpy.init(args=args)
    node = VoxelLocalMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
