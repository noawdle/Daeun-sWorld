"""Perception 스텁 (통합 프로그램 템플릿) - /vertiport/perception 규격 예시 출력.

⚠️ AI 가 아니다. Gazebo 정답 좌표(ground_truth)를 카메라에 투영해 '가짜 탐지'를 만든다.
   목적: AI·통합 프로그램이 준비되기 전에 HTTP Bridge(/api/perception, /api/scene)와
   대시보드를 끝까지 연결해 보는 것. 통합 프로그램은 이 노드를 같은 출력 형식으로 대체한다.

탐지 범위 (팀 합의): landing_pad, fod  ← 사람·차량은 AI 범위가 아님 (시나리오 객체로만 존재)

입력  /camera/down/image_raw, /camera/down/camera_info, /vertiport/telemetry, /vertiport/ground_truth
출력  /vertiport/perception        (std_msgs/String JSON, INTERFACE.md §7)
      /vertiport/perception/image  (sensor_msgs/Image rgb8, bbox 표시 - HTTP /stream/perception.mjpg)
"""

import bisect
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from . import camera_geometry as cg

try:
    from PIL import Image as PILImage, ImageDraw
except ImportError:  # pragma: no cover
    PILImage = None

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SIZES = {'landing_pad': None, 'fod': 0.3}   # 물체 크기 [m] (bbox 크기 계산용), 패드는 ground_truth size_m


class PerceptionStub(Node):
    def __init__(self):
        super().__init__('vertiport_perception_stub')
        self.rate = float(self.declare_parameter('rate_hz', 5.0).value)
        self.noise_px = float(self.declare_parameter('noise_px', 2.0).value)
        self.annotate = bool(self.declare_parameter('publish_image', True).value)
        self.info = None
        self.gt = None
        self.tel_t, self.tel = [], []       # t_px4 정렬 버퍼 (영상 시각에 맞는 기체 자세 찾기)
        self._last = 0.0
        self.create_subscription(Image, '/camera/down/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/down/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/vertiport/telemetry', self._on_tel, 10)
        self.create_subscription(String, '/vertiport/ground_truth', self._on_gt, LATCHED)
        self.pub = self.create_publisher(String, '/vertiport/perception', 10)
        self.pub_img = self.create_publisher(Image, '/vertiport/perception/image', 5)
        self.get_logger().warn('perception_stub: ground truth 투영으로 만든 가짜 탐지 (AI 아님)')

    def _on_info(self, m):
        self.info = m

    def _on_gt(self, m):
        self.gt = json.loads(m.data)

    def _on_tel(self, m):
        d = json.loads(m.data)
        if d.get('orientation_q') is None:
            return
        self.tel_t.append(d['t_px4'])
        self.tel.append(d)
        if len(self.tel) > 100:
            del self.tel_t[:20], self.tel[:20]

    def _pose_at(self, t):
        """영상 시각 t 에 가장 가까운 telemetry (0.2 s 이내)."""
        if not self.tel:
            return None
        i = bisect.bisect_left(self.tel_t, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(self.tel)), key=lambda j: abs(self.tel_t[j] - t))
        return self.tel[best] if abs(self.tel_t[best] - t) < 0.2 else None

    def _on_image(self, img):
        now = time.monotonic()
        if now - self._last < 1.0 / self.rate or self.info is None or self.gt is None:
            return
        self._last = now
        t0 = time.time()
        t_img = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9
        tel = self._pose_at(t_img)
        if tel is None:
            return
        K = list(self.info.k)
        pos, q = tel['position_m'], tel['orientation_q']
        pad = self.gt['pad']
        targets = [('landing_pad', pad['id'], (pad['x'], pad['y'], 0.0), pad.get('size_m', 3.0))]
        targets += [('fod', o['id'], (o['x'], o['y'], 0.0), SIZES['fod'])
                    for o in self.gt.get('objects', []) if o['type'] == 'fod']

        out_pad, out_fod, boxes = {'detected': False}, [], []
        for cls, gid, p, size in targets:
            uv = cg.world_to_pixel(p, K, pos, q)
            if uv is None or not (0 <= uv[0] < img.width and 0 <= uv[1] < img.height):
                continue
            u = uv[0] + random.gauss(0, self.noise_px)
            v = uv[1] + random.gauss(0, self.noise_px)
            dist = max(0.3, pos['z'] - 0.14)
            half = 0.5 * size * K[0] / dist
            bbox = [round(max(0, u - half)), round(max(0, v - half)),
                    round(min(img.width - 1, u + half)), round(min(img.height - 1, v + half))]
            w = cg.pixel_to_ground(u, v, K, pos, q)
            if w is None:
                continue
            err = math.hypot(w[0] - p[0], w[1] - p[1])
            det = {'conf': 0.9, 'bbox_xyxy': bbox, 'center_px': {'u': round(u, 1), 'v': round(v, 1)},
                   'position_m': {'x': round(w[0], 3), 'y': round(w[1], 3), 'z': 0.0},
                   'gt_id': gid, 'gt_error_m': round(err, 3)}
            boxes.append((cls, bbox))
            if cls == 'landing_pad':
                out_pad = {'detected': True, **det}
            else:
                out_fod.append({'id': f'fod_{len(out_fod) + 1}', 'class': 'fod', **det})

        msg = {'type': 'perception', 'schema': '1.1', 't': round(time.time(), 3), 't_img': round(t_img, 4),
               'source': 'perception_stub', 'frame': 'ENU',
               'image': {'width': img.width, 'height': img.height, 'frame_id': img.header.frame_id},
               'drone_pose': {'position_m': pos, 'orientation_q': q, 't_px4': tel['t_px4']},
               'landing_pad': out_pad, 'fod': out_fod,
               'latency_ms': round((time.time() - t0) * 1000, 1)}
        self.pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))
        if self.annotate and PILImage is not None and img.encoding == 'rgb8':
            self._publish_annotated(img, boxes)

    def _publish_annotated(self, img, boxes):
        pil = PILImage.frombuffer('RGB', (img.width, img.height), bytes(img.data), 'raw', 'RGB', img.step, 1).copy()
        d = ImageDraw.Draw(pil)
        for cls, b in boxes:
            color = (163, 113, 247) if cls == 'landing_pad' else (248, 81, 73)
            d.rectangle(b, outline=color, width=3)
            d.text((b[0] + 4, max(0, b[1] - 12)), cls, fill=color)
        out = Image()
        out.header = img.header
        out.width, out.height, out.encoding, out.step = img.width, img.height, 'rgb8', img.width * 3
        out.data = pil.tobytes()
        self.pub_img.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
