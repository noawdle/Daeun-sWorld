"""HTTP Bridge: ROS 2 토픽 → 3D 대시보드 HTTP API v1 (대시보드 gazebo_bridge_client.py 규격).

  GET  /api/health                {"api_version", "status", "simulation_ready", ...}
  GET  /api/telemetry             대시보드 텔레메트리 (GAZEBO_WORLD_ENU, deg, ISO 8601)
  POST /api/command               set_target / hover / request_landing / cancel_mission
                                  → {"request_id", "accepted", "command_id", "code", "message"}
  GET  /camera                    하향 카메라 MJPEG (camera_stream_url, 브라우저 <img> 로 재생)
  GET  /api/camera                카메라 내부 파라미터·TF·영상 timestamp
  GET  /api/frames                좌표계 정의 (GAZEBO_WORLD_ENU ↔ VERTIPORT_LOCAL_XYZ_UP)
  GET  /api/perception            AI 탐지(landing_pad, fod) + 월드 좌표
  GET  /api/scene                 3D 장면 (드론, 착륙패드, FOD, 시나리오 객체, 판단, 카메라)
  GET  /api/camera/snapshot.jpg   최신 프레임 1장
  GET  /stream/perception.mjpg    통합 프로그램의 탐지 결과 영상
  POST /api/scenario              (테스트용) 시나리오 적용
  GET  /                          점검용 뷰어 페이지

JSON(UTF-8) + CORS 허용. 표준 라이브러리 http.server 사용 (JPEG 인코딩만 Pillow).
"""

import io
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from . import dashboard_api as dapi
from . import frames

try:
    from PIL import Image as PILImage
except ImportError:  # pragma: no cover
    PILImage = None

SCHEMA = '1.1'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
_PIL_MODES = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'), 'mono8': ('L', 'L'),
              'rgba8': ('RGBA', 'RGBA'), 'bgra8': ('RGBA', 'BGRA')}


def _stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Latest:
    """토픽별 최신 JSON + 수신 시각."""

    def __init__(self):
        self.data: Optional[dict] = None
        self.t = 0.0

    def set(self, d):
        self.data, self.t = d, time.monotonic()

    def age(self) -> Optional[float]:
        return None if self.data is None else round(time.monotonic() - self.t, 3)


class FrameStore:
    """최신 영상 1장 보관. JPEG 은 요청이 있을 때만 인코딩하고 프레임마다 1번만 캐시."""

    def __init__(self, max_width: int, quality: int):
        self.max_width, self.quality = max_width, quality
        self.cv = threading.Condition()
        self.msg = None
        self.seq = 0
        self.t = 0.0
        self.fps = 0.0
        self._jpeg_seq = -1
        self._jpeg = None
        self.error = None

    def update(self, msg):
        now = time.monotonic()
        with self.cv:
            if self.msg is not None:
                dt = now - self.t
                if dt > 0:
                    self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if self.fps else 1.0 / dt
            self.msg, self.t = msg, now
            self.seq += 1
            self.cv.notify_all()

    def age(self):
        return None if self.msg is None else round(time.monotonic() - self.t, 3)

    def info(self):
        m = self.msg
        if m is None:
            return None
        return {'stamp': round(_stamp_s(m.header), 4), 'frame_id': m.header.frame_id, 'width': m.width,
                'height': m.height, 'encoding': m.encoding, 'fps': round(self.fps, 1), 'age_s': self.age()}

    def jpeg(self):
        """(seq, jpeg_bytes, stamp) 또는 None."""
        with self.cv:
            m, seq = self.msg, self.seq
            if m is None:
                return None
            if self._jpeg_seq == seq:
                return seq, self._jpeg, _stamp_s(m.header)
        if PILImage is None:
            self.error = 'Pillow 미설치 (sudo apt install python3-pil)'
            return None
        mode = _PIL_MODES.get(m.encoding)
        if mode is None:
            self.error = f'지원하지 않는 encoding: {m.encoding}'
            return None
        img = PILImage.frombuffer(mode[0], (m.width, m.height), bytes(m.data), 'raw', mode[1], m.step, 1)
        if img.mode == 'RGBA':
            img = img.convert('RGB')
        if self.max_width and m.width > self.max_width:
            img = img.resize((self.max_width, int(m.height * self.max_width / m.width)))
        buf = io.BytesIO()
        img.save(buf, 'JPEG', quality=self.quality)
        out = buf.getvalue()
        with self.cv:
            if self.seq == seq:
                self._jpeg_seq, self._jpeg = seq, out
        return seq, out, _stamp_s(m.header)

    def wait_new(self, last_seq, timeout):
        with self.cv:
            self.cv.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.seq != last_seq


class HttpBridge(Node):
    def __init__(self):
        super().__init__('vertiport_http_bridge')
        p = self.declare_parameter
        self.host = p('http_host', '0.0.0.0').value
        self.port = int(p('http_port', 8080).value)
        self.public_base = p('public_base_url', '').value.rstrip('/')
        self.vehicle_id = p('vehicle_id', 'X500-01').value
        self.stream_fps = float(p('stream_fps', 10.0).value)
        self.cmd_timeout = float(p('command_timeout_s', 2.0).value)
        self.landing_max_age = float(p('landing_request_max_age_s', 3.0).value)
        self.stale_s = float(p('telemetry_stale_s', 1.0).value)
        self.pad_xy = [float(v) for v in p('pad_xy', [0.0, 0.0]).value]
        self.pad_z = float(p('pad_z', 0.0).value)
        self.pad_size = float(p('pad_size_m', 3.0).value)
        self.gear_h = float(p('gear_height_m', frames.BASE_LINK_HEIGHT).value)
        self.local = dapi.LocalFrame((self.pad_xy[0], self.pad_xy[1], self.pad_z),
                                     float(p('local_frame_yaw_deg', 0.0).value))
        self.cam_xyz = [float(v) for v in p('camera_mount_xyz', list(frames.CAMERA_MOUNT_XYZ)).value]
        self.cam_rpy = [float(v) for v in p('camera_mount_rpy', list(frames.CAMERA_MOUNT_RPY)).value]
        cam_topic = p('camera_topic', '/camera/down/image_raw').value
        info_topic = p('camera_info_topic', '/camera/down/camera_info').value
        perc_img_topic = p('perception_image_topic', '/vertiport/perception/image').value
        max_w = int(p('stream_max_width', 960).value)
        quality = int(p('jpeg_quality', 80).value)
        self.www = p('www_dir', os.path.join(get_package_share_directory('vertiport_sim'), 'www')).value

        self.lock = threading.Lock()
        self.tel, self.gt, self.perc, self.dec = Latest(), Latest(), Latest(), Latest()
        self.tlog = dapi.TelemetryLog()
        self.cam_info: Optional[CameraInfo] = None
        self.streams = {'camera_down': FrameStore(max_w, quality), 'perception': FrameStore(max_w, quality)}
        self.waiters = {}          # command_id → [Event, ack]
        self._cmd_n = int(time.time()) % 100000 * 100

        self.pub_cmd = self.create_publisher(String, '/vertiport/command', 10)
        self.pub_scn = self.create_publisher(String, '/vertiport/scenario_cmd', 10)
        sub = self.create_subscription
        sub(String, '/vertiport/telemetry', self._on_tel, 10)
        sub(String, '/vertiport/ground_truth', self._json(self.gt), LATCHED)
        sub(String, '/vertiport/perception', self._json(self.perc), 10)
        sub(String, '/vertiport/decision', self._json(self.dec), 10)
        sub(String, '/vertiport/command_ack', self._on_ack, 10)
        sub(Image, cam_topic, self.streams['camera_down'].update, qos_profile_sensor_data)
        sub(Image, perc_img_topic, self.streams['perception'].update, qos_profile_sensor_data)
        sub(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)

        self.httpd = ThreadingHTTPServer((self.host, self.port), _make_handler(self))
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'HTTP bridge (api v{dapi.API_VERSION}): http://{self.host}:{self.port}  '
                               f'camera={cam_topic}, local frame yaw={self.local.yaw_deg} deg')

    # ------------------------------------------------------------ ROS 입력
    def _json(self, slot: Latest):
        def cb(msg):
            try:
                slot.set(json.loads(msg.data))
            except json.JSONDecodeError:
                pass
        return cb

    def _on_tel(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.tel.set(d)
        self.tlog.add(d.get('telemetry_id'), d.get('t_sample') or d.get('t'))

    def _on_info(self, msg):
        self.cam_info = msg

    def _on_ack(self, msg):
        try:
            ack = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            w = self.waiters.get(ack.get('command_id'))
        if w:
            w[1] = ack
            w[0].set()

    # ------------------------------------------------------------ 공통
    def base_url(self, host_header: Optional[str]) -> str:
        if self.public_base:
            return self.public_base
        return f'http://{host_header or f"127.0.0.1:{self.port}"}'

    def _fresh(self):
        age = self.tel.age()
        return age is not None and age < self.stale_s

    # ------------------------------------------------------------ API
    def api_health(self, host=None):
        tel = self.tel.data
        cam = self.streams['camera_down']
        gt = self.gt.data
        px4_ok = bool(tel and tel.get('connected') and tel.get('valid'))
        cmdr_ok = bool(tel) and tel.get('state') not in (None, 'NO_COMMANDER', 'DISCONNECTED')
        world_ok = True if (tel or {}).get('source') != 'sim' else bool(gt and gt.get('world_ready'))
        comp = {
            'telemetry': {'ok': self._fresh(), 'age_s': self.tel.age()},
            'px4': {'ok': px4_ok, 'armed': (tel or {}).get('px4', {}).get('armed'),
                    'mode': (tel or {}).get('px4', {}).get('mode')},
            'commander': {'ok': cmdr_ok, 'state': (tel or {}).get('state')},
            'gazebo_world': {'ok': world_ok, 'scenario': (gt or {}).get('scenario')},
            'camera': {'ok': cam.age() is not None and cam.age() < 1.0, **(cam.info() or {'age_s': None})},
            'perception': {'ok': self.perc.age() is not None and self.perc.age() < 2.0, 'age_s': self.perc.age()},
            'decision': {'ok': self.dec.age() is not None and self.dec.age() < 2.0, 'age_s': self.dec.age()},
        }
        ready = comp['telemetry']['ok'] and px4_ok and cmdr_ok and world_ok
        if not comp['telemetry']['ok']:
            status = 'down'
        elif ready and comp['camera']['ok']:
            status = 'ok'
        else:
            status = 'degraded'
        return 200, {'api_version': dapi.API_VERSION, 'status': status, 'simulation_ready': ready,
                     'timestamp': dapi.iso_utc(time.time()), 'components': comp}

    def api_telemetry(self, host=None):
        cam_url = self.base_url(host) + '/camera'
        if self.tel.data is None:
            return 200, dapi.empty_telemetry(self.vehicle_id, cam_url)
        link_ok = self._fresh() and bool(self.tel.data.get('connected'))
        return 200, dapi.to_dashboard_telemetry(self.tel.data, self.vehicle_id, cam_url,
                                                pad_z=self.pad_z, gear_height=self.gear_h, link_ok=link_ok)

    def api_frames(self, host=None):
        d = self.local.describe()
        d['vehicle_origin']['z_offset_m'] = self.gear_h
        d['vehicle_origin']['altitude_m'] = 'Landing Pad 면 기준 기체 바닥 높이 = z - pad_z - %.2f (착지 시 ≈ 0)' % self.gear_h
        d['api_version'] = dapi.API_VERSION
        return 200, d

    def api_perception(self, host=None):
        if self.perc.data is None:
            return 200, {'type': 'perception', 'frame': dapi.FRAME, 'available': False,
                         'reason': 'no /vertiport/perception yet', 'landing_pad': None, 'fod': []}
        return 200, {'available': True, 'age_s': self.perc.age(), **self.perc.data, 'frame': dapi.FRAME}

    def camera_block(self, host=None):
        ci = self.cam_info
        q = frames.euler_to_quat_wxyz(*self.cam_rpy)
        out = {
            'topic_image': '/camera/down/image_raw', 'topic_info': '/camera/down/camera_info',
            'camera_stream_url': self.base_url(host) + '/camera',
            'snapshot_url': self.base_url(host) + '/api/camera/snapshot.jpg',
            'perception_stream_url': self.base_url(host) + '/stream/perception.mjpg',
            'image': self.streams['camera_down'].info(),
            'tf': {'tree': 'map → base_link → camera_link → camera_optical_frame',
                   'base_to_camera_link': {'xyz_m': self.cam_xyz, 'rpy_deg': [round(math.degrees(a), 3)
                                                                               for a in self.cam_rpy],
                                           'q_xyzw': frames.quat_xyzw(q)},
                   'camera_link_to_optical_q_xyzw': frames.quat_xyzw(frames.Q_CAMLINK_FROM_OPTICAL_WXYZ),
                   'R_base_from_optical': [list(r) for r in frames.R_BODY_FLU_FROM_CAM_OPTICAL],
                   'image_orientation': 'image up = vehicle nose (forward), image right = vehicle right'},
            'intrinsics': None,
        }
        if ci is not None:
            out['intrinsics'] = {'width': ci.width, 'height': ci.height, 'K': [float(v) for v in ci.k],
                                 'D': [float(v) for v in ci.d], 'distortion_model': ci.distortion_model,
                                 'frame_id': ci.header.frame_id, 'hfov_deg': self._hfov_deg()}
        return out

    def api_camera(self, host=None):
        return 200, self.camera_block(host)

    def _hfov_deg(self):
        ci = self.cam_info
        if ci is None or not ci.k[0]:
            return None
        return round(math.degrees(2 * math.atan(ci.width / 2 / ci.k[0])), 2)

    def api_scene(self, host=None):
        gt, perc, dec = self.gt.data, self.perc.data, self.dec.data
        _, drone = self.api_telemetry(host)
        pad = (gt or {}).get('pad') or {'id': 'pad_1', 'x': self.pad_xy[0], 'y': self.pad_xy[1], 'z': self.pad_z,
                                        'size_m': self.pad_size}
        lp = (perc or {}).get('landing_pad')
        return 200, {
            'api_version': dapi.API_VERSION, 'timestamp': dapi.iso_utc(time.time()),
            'frame': dapi.FRAME, 'units': {'length': 'm', 'angle': 'deg'},
            'drone': drone,
            'landing_pad': {'id': pad.get('id', 'pad_1'), 'x': pad['x'], 'y': pad['y'], 'z': pad.get('z', 0.0),
                            'yaw': 0.0, 'size_m': pad.get('size_m', self.pad_size),
                            'perceived': lp if (lp and lp.get('detected')) else None},
            'fod': [{**f, 'source': 'perception'} for f in (perc or {}).get('fod', [])],
            'scenario_objects': [{'id': o['id'], 'type': o['type'], 'x': o['x'], 'y': o['y'], 'z': o.get('z', 0.0),
                                  'yaw': round(math.degrees(o.get('yaw', 0.0)), 2), 'source': 'scenario'}
                                 for o in (gt or {}).get('objects', [])],
            'scenario': (gt or {}).get('scenario'),
            'decision': None if dec is None else {k: dec.get(k) for k in ('decision', 'label_ko', 'reasons',
                                                                          'source')},
            'camera': {'camera_stream_url': self.base_url(host) + '/camera',
                       'mount': {'parent': 'base_link', 'xyz_m': self.cam_xyz,
                                 'rpy_deg': [round(math.degrees(a), 3) for a in self.cam_rpy]},
                       'hfov_deg': self._hfov_deg()},
        }

    def api_command(self, body, host=None):
        rid = body.get('request_id') if isinstance(body, dict) else None
        with self.lock:
            self._cmd_n += 1
            cid = f'cmd-{self._cmd_n}'

        def resp(accepted, code, message, **extra):
            out = {'request_id': rid, 'accepted': accepted, 'command_id': cid, 'code': code, 'message': message}
            out.update(extra)
            return 200, out

        try:
            native, desc = dapi.translate_command(body, self.local, self.tlog, time.time(),
                                                  pad_xy=self.pad_xy, landing_max_age_s=self.landing_max_age)
        except dapi.CommandError as e:
            self.get_logger().warn(f'[cmd] {rid} {body} → {e.code}: {e.message}')
            return resp(False, e.code, e.message)
        if self.pub_cmd.get_subscription_count() == 0:
            return resp(False, 'COMMANDER_UNAVAILABLE', 'commander node is not running')
        native.update({'command_id': cid, 'request_id': rid, 'source': 'dashboard'})
        ev = threading.Event()
        with self.lock:
            self.waiters[cid] = [ev, None]
        self.pub_cmd.publish(String(data=json.dumps(native)))
        ok = ev.wait(self.cmd_timeout)
        with self.lock:
            _, ack = self.waiters.pop(cid)
        if not ok or ack is None:
            return resp(False, 'TIMEOUT', f'commander did not respond within {self.cmd_timeout:.1f}s')
        message = ack.get('reason') if not ack.get('accepted') else f'{desc}: {ack.get("reason")}'
        self.get_logger().info(f'[cmd] {rid} → {cid} {ack.get("code")}: {message}')
        return resp(bool(ack.get('accepted')), ack.get('code') or ('ACCEPTED' if ack.get('accepted') else
                                                                   'REJECTED_STATE'),
                    message, state=ack.get('state'))

    def api_scenario(self, body, host=None):
        if not isinstance(body, dict) or not body.get('op'):
            return 400, {'ok': False, 'reason': 'body must be {"op": ...}'}
        self.pub_scn.publish(String(data=json.dumps(body)))
        return 202, {'ok': True, 'forwarded': body}


def _make_handler(bridge: HttpBridge):
    GET = {'/api/health': bridge.api_health, '/api/telemetry': bridge.api_telemetry,
           '/api/perception': bridge.api_perception, '/api/scene': bridge.api_scene,
           '/api/camera': bridge.api_camera, '/api/frames': bridge.api_frames}
    POST = {'/api/command': bridge.api_command, '/api/scenario': bridge.api_scenario}
    STREAMS = {'/camera': 'camera_down', '/stream/camera_down.mjpg': 'camera_down',
               '/stream/perception.mjpg': 'perception'}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        server_version = 'VertiportBridge/' + dapi.API_VERSION

        def log_message(self, fmt, *args):  # 기본 stderr 로그 끔
            pass

        def _cors(self):
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')

        def _send(self, code, body: bytes, ctype, extra=None):
            self.send_response(code)
            self._cors()
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode('utf-8'), 'application/json; charset=utf-8')

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.send_header('Content-Length', '0')
            self.end_headers()

        def do_GET(self):
            path = urlparse(self.path).path.rstrip('/') or '/'
            host = self.headers.get('Host')
            try:
                if path in GET:
                    return self._json(*GET[path](host))
                if path in STREAMS:
                    return self._mjpeg(bridge.streams[STREAMS[path]])
                if path in ('/', '/viewer', '/index.html'):
                    with open(os.path.join(bridge.www, 'viewer.html'), 'rb') as f:
                        return self._send(200, f.read(), 'text/html; charset=utf-8')
                if path in ('/api/camera/snapshot.jpg', '/camera/snapshot.jpg'):
                    fr = bridge.streams['camera_down'].jpeg()
                    if fr is None:
                        return self._json(503, {'reason': bridge.streams['camera_down'].error or 'no image yet'})
                    return self._send(200, fr[1], 'image/jpeg', {'X-Image-Stamp': f'{fr[2]:.6f}'})
                return self._json(404, {'reason': f'not found: {path}'})
            except (BrokenPipeError, ConnectionResetError):
                return None

        def do_POST(self):
            path = urlparse(self.path).path.rstrip('/')
            if path not in POST:
                return self._json(404, {'reason': f'not found: {path}'})
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'null')
            except (ValueError, json.JSONDecodeError):
                return self._json(400, {'request_id': None, 'accepted': False, 'command_id': None,
                                        'code': 'INVALID_REQUEST', 'message': 'body is not valid JSON'})
            return self._json(*POST[path](body, self.headers.get('Host')))

        def _mjpeg(self, store: FrameStore):
            self.send_response(200)
            self._cors()
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            last, period = -1, 1.0 / max(1.0, bridge.stream_fps)
            while rclpy.ok():
                t0 = time.monotonic()
                if not store.wait_new(last, timeout=2.0):
                    continue
                fr = store.jpeg()
                if fr is None:
                    time.sleep(0.2)
                    continue
                last, jpg, stamp = fr
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                 + f'Content-Length: {len(jpg)}\r\nX-Image-Stamp: {stamp:.6f}\r\n\r\n'.encode()
                                 + jpg + b'\r\n')
                self.wfile.flush()
                time.sleep(max(0.0, period - (time.monotonic() - t0)))

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = HttpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
