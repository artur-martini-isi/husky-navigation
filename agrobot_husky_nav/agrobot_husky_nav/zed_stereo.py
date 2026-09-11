#!/usr/bin/env python3
"""zed_stereo — publica o par estéreo da ZED 2i sem o SDK da Stereolabs.

A ZED aparece como uma câmera UVC comum (`/dev/video0`) que entrega **um quadro único
lado a lado**: metade esquerda = câmera esquerda, metade direita = câmera direita. Este nó
abre esse dispositivo com OpenCV, corta o quadro ao meio e publica as duas imagens com
`CameraInfo` preenchido a partir da calibração de fábrica.

Nada de CUDA, nada de SDK, nada de profundidade neural: só as duas imagens e a calibração.
Quem quiser retificação, disparidade ou nuvem de pontos pode ligar o `stereo_image_proc`
por cima, porque o `CameraInfo` já sai com `R`/`P` de retificação.

    /dev/video0 (YUYV, lado a lado)
        └─► zed_stereo ─┬─► <ns>/zed/left/image_raw          (sensor_msgs/Image, bgr8)
                        ├─► <ns>/zed/left/image_raw/compressed (sensor_msgs/CompressedImage, jpeg)
                        ├─► <ns>/zed/left/camera_info         (sensor_msgs/CameraInfo)
                        └─► idem para right/
        serviço <ns>/zed/save  (std_srvs/Trigger) grava o par atual em disco

A calibração de fábrica é baixada por número de série em https://calib.stereolabs.com/?SN=<serial>
(esta câmera: 33943780) e fica em `calibration_file`. Sem ela o nó funciona igual, mas o
`CameraInfo` sai vazio e não dá para retificar.
"""
import configparser
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger

# nome no arquivo .conf -> (largura por olho, altura, fps padrão)
RESOLUTIONS = {
    '2K':  (2208, 1242, 15),
    'FHD': (1920, 1080, 30),
    'HD':  (1280, 720, 30),
    'VGA': (672, 376, 60),
}


class ZedStereo(Node):
    def __init__(self):
        super().__init__('zed_stereo')
        p = self.declare_parameter
        p('device', '/dev/video0')
        p('resolution', 'HD')                 # 2K | FHD | HD | VGA
        p('fps', 0)                           # 0 = usa o padrão da resolução
        p('publish_rate', 0.0)                # 0 = publica todo quadro capturado
        p('calibration_file', '/home/robot/zed_calib.conf')
        p('publish_raw', True)
        p('publish_compressed', True)
        p('jpeg_quality', 80)
        p('frame_id_left', 'zed_left_camera_optical_frame')
        p('frame_id_right', 'zed_right_camera_optical_frame')
        p('save_dir', '/home/robot/zed_captures')
        p('flip_180', False)                  # se a câmera estiver montada de cabeça para baixo
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.res_name = str(g('resolution')).upper()
        if self.res_name not in RESOLUTIONS:
            raise SystemExit('resolução inválida: %s (use %s)' % (self.res_name, list(RESOLUTIONS)))
        self.w, self.h, default_fps = RESOLUTIONS[self.res_name]
        self.fps = int(g('fps')) or default_fps
        self.publish_raw = bool(g('publish_raw'))
        self.publish_compressed = bool(g('publish_compressed'))
        self.jpeg_quality = int(g('jpeg_quality'))
        self.frame_left, self.frame_right = str(g('frame_id_left')), str(g('frame_id_right'))
        self.save_dir = str(g('save_dir'))
        self.flip = bool(g('flip_180'))
        self.bridge = CvBridge()
        self.last_pair = None
        self.frames = self.dropped = 0
        self.t_start = time.monotonic()

        # ---------------------------------------------------------------- câmera
        device = str(g('device'))
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit('não consegui abrir %s (o usuário está no grupo video?)' % device)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w * 2)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # menos latência
        except Exception:  # noqa: BLE001
            pass
        got_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (got_w, got_h) != (self.w * 2, self.h):
            self.get_logger().warn('a câmera entregou %dx%d, esperado %dx%d' % (got_w, got_h, self.w * 2, self.h))
            self.w, self.h = got_w // 2, got_h

        # ---------------------------------------------------------------- calibração
        self.info_left, self.info_right = self.load_calibration(str(g('calibration_file')))

        # ---------------------------------------------------------------- ROS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = {}
        for side in ('left', 'right'):
            if self.publish_raw:
                self.pub[side + '_raw'] = self.create_publisher(Image, 'zed/%s/image_raw' % side, qos)
            if self.publish_compressed:
                self.pub[side + '_jpg'] = self.create_publisher(
                    CompressedImage, 'zed/%s/image_raw/compressed' % side, qos)
            self.pub[side + '_info'] = self.create_publisher(CameraInfo, 'zed/%s/camera_info' % side, qos)
        self.create_service(Trigger, 'zed/save', self.srv_save)

        rate = float(g('publish_rate')) or float(self.fps)
        self.create_timer(1.0 / rate, self.grab)
        self.create_timer(10.0, self.log_summary)
        self.get_logger().info(
            'ZED 2i em %s: %s (%dx%d por olho) a %d fps | raw=%s jpeg=%s | calibração: %s' % (
                device, self.res_name, self.w, self.h, self.fps, self.publish_raw,
                self.publish_compressed, 'sim' if self.info_left.k[0] else 'NÃO'))

    # ------------------------------------------------------------------ calibração
    def load_calibration(self, path):
        """Lê o .conf de fábrica da ZED e devolve CameraInfo já retificado (R e P)."""
        left, right = CameraInfo(), CameraInfo()
        for ci, frame in ((left, self.frame_left), (right, self.frame_right)):
            ci.width, ci.height = self.w, self.h
            ci.header.frame_id = frame
            ci.distortion_model = 'plumb_bob'
        if not os.path.exists(path):
            self.get_logger().warn('sem arquivo de calibração em %s: CameraInfo sai vazio '
                                   '(baixe com: curl -o %s "https://calib.stereolabs.com/?SN=<serial>")'
                                   % (path, path))
            return left, right
        cfg = configparser.ConfigParser()
        cfg.read(path)
        try:
            def intr(section):
                s = cfg[section]
                K = np.array([[float(s['fx']), 0.0, float(s['cx'])],
                              [0.0, float(s['fy']), float(s['cy'])],
                              [0.0, 0.0, 1.0]])
                D = np.array([float(s['k1']), float(s['k2']), float(s['p1']),
                              float(s['p2']), float(s.get('k3', 0.0))])
                return K, D

            K1, D1 = intr('LEFT_CAM_' + self.res_name)
            K2, D2 = intr('RIGHT_CAM_' + self.res_name)
            st = cfg['STEREO']
            baseline = float(st['Baseline']) / 1000.0          # mm -> m
            ty = float(st.get('TY', 0.0)) / 1000.0
            tz = float(st.get('TZ', 0.0)) / 1000.0
            rx = float(st.get('RX_' + self.res_name, 0.0))
            cv_ = float(st.get('CV_' + self.res_name, 0.0))
            rz = float(st.get('RZ_' + self.res_name, 0.0))
            R = cv2.Rodrigues(np.array([rx, cv_, rz]))[0]
            T = np.array([-baseline, ty, tz])                  # origem da direita vista pela esquerda
            R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
                K1, D1, K2, D2, (self.w, self.h), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
            for ci, K, D, Rr, P in ((left, K1, D1, R1, P1), (right, K2, D2, R2, P2)):
                ci.k = K.flatten().tolist()
                ci.d = D.flatten().tolist()
                ci.r = Rr.flatten().tolist()
                ci.p = P.flatten().tolist()
            self.get_logger().info('calibração %s carregada: fx=%.1f baseline=%.1f mm'
                                   % (self.res_name, K1[0, 0], baseline * 1000))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('falha lendo a calibração %s: %s' % (path, e))
        return left, right

    # ------------------------------------------------------------------ captura
    def grab(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.dropped += 1
            if self.dropped in (1, 10) or self.dropped % 100 == 0:
                self.get_logger().warn('falha lendo quadro da câmera (%d)' % self.dropped)
            return
        half = frame.shape[1] // 2
        img_l, img_r = frame[:, :half], frame[:, half:]
        if self.flip:
            img_l, img_r = cv2.rotate(img_l, cv2.ROTATE_180), cv2.rotate(img_r, cv2.ROTATE_180)
        self.frames += 1
        stamp = self.get_clock().now().to_msg()
        self.last_pair = (img_l.copy(), img_r.copy())

        for side, img, info in (('left', img_l, self.info_left), ('right', img_r, self.info_right)):
            if self.publish_raw:
                m = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                m.header.stamp = stamp
                m.header.frame_id = info.header.frame_id
                self.pub[side + '_raw'].publish(m)
            if self.publish_compressed:
                ok_enc, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if ok_enc:
                    c = CompressedImage()
                    c.header.stamp = stamp
                    c.header.frame_id = info.header.frame_id
                    c.format = 'jpeg'
                    c.data = buf.tobytes()
                    self.pub[side + '_jpg'].publish(c)
            info.header.stamp = stamp
            self.pub[side + '_info'].publish(info)

    # ------------------------------------------------------------------ serviço
    def srv_save(self, req, res):
        if self.last_pair is None:
            res.success, res.message = False, 'nenhum quadro capturado ainda'
            return res
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            paths = []
            for side, img in (('left', self.last_pair[0]), ('right', self.last_pair[1])):
                path = os.path.join(self.save_dir, '%s_%s_%s.png' % (ts, self.res_name, side))
                cv2.imwrite(path, img)
                paths.append(path)
            res.success, res.message = True, ' e '.join(paths)
            self.get_logger().info('par salvo: %s' % res.message)
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
            self.get_logger().error('falha salvando o par: %s' % e)
        return res

    def log_summary(self):
        dt = time.monotonic() - self.t_start
        self.get_logger().info('%d quadros em %.0f s (%.1f fps) | falhas de leitura: %d'
                               % (self.frames, dt, self.frames / dt if dt else 0.0, self.dropped))

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedStereo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
