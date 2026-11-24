"""WebRTC 트랙별 최신 프레임 분석 상태와 영상 오버레이."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from threading import Lock, Thread
from time import monotonic


VIDEO_IDLE = "영상이 시작되지 않았습니다. START를 누르고 카메라 권한을 허용해 주세요."
VIDEO_STOPPED = "영상 분석이 종료되었거나 카메라 연결이 끊겼습니다."
SNAPSHOT_CLEAR_SECONDS = 3.0
_SNAPSHOT_WRITER = ThreadPoolExecutor(max_workers=1, thread_name_prefix="fod-snapshot")


def draw_boxes(image, boxes):
    import cv2

    height, width = image.shape[:2]
    for (left, top, right, bottom), score in boxes:
        x1, y1 = round(left * width), round(top * height)
        x2, y2 = round(right * width), round(bottom * height)
        color = (40, 60, 235)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"FOD {score:.0%}", (x1, max(16, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


class VideoAnalysis:
    def __init__(self, model, threshold, interval, image_size, max_age, save_detection=None):
        self._lock = Lock()
        self._settings = (model, threshold, interval, image_size, max_age)
        self._generation = 0
        self._closed = False
        self._busy = False
        self._received = None
        self._started = float("-inf")
        self._result = None
        self._error = None
        self._created_at = monotonic()
        self._save_detection = save_detection
        self._snapshot_error = None
        self._episode_active = False
        self._clear_since = None
        self._last_sample = None
        self._analysis_at = None
        self._processing_ms = None

    def configure(self, settings, save_detection=None):
        with self._lock:
            self._save_detection = save_detection
            if settings != self._settings:
                self._settings = settings
                self._generation += 1
                self._result = self._error = None
                self._started = float("-inf")
                self._clear_since = self._last_sample = None

    def on_ended(self):
        with self._lock:
            self._closed = True
            self._result = self._error = None

    def _current_result(self, now):
        max_age = self._settings[-1]
        if self._result is not None:
            received, finished, boxes = self._result
            if now - finished <= max_age and now - received <= max_age:
                return boxes
        return None

    def snapshot(self):
        with self._lock:
            now = monotonic()
            if self._closed:
                return None, VIDEO_STOPPED
            if now - (self._received or self._created_at) > self._settings[-1]:
                return None, "카메라 프레임 수신이 지연되거나 중단되었습니다."
            if self._error is not None:
                return None, self._error
            boxes = self._current_result(now)
            if boxes is None:
                reason = ("최신 영상 분석 결과가 만료되었습니다." if self._result is not None
                          else "첫 영상 분석 결과를 기다리는 중입니다.")
                return None, reason
            return {
                "fod_detected": bool(boxes),
                "fod_count": len(boxes),
                "confidence": max((score for _, score in boxes), default=None),
                "analyzed_at": self._analysis_at,
                "processing_ms": self._processing_ms,
                "valid_for_ms": max(0, round((self._settings[-1] - (now - self._result[0])) * 1000)),
            }, ""

    def _new_episode(self, boxes, received, max_gap):
        if self._last_sample is not None and received - self._last_sample > max_gap:
            self._clear_since = None
        self._last_sample = received
        if boxes:
            self._clear_since = None
            if not self._episode_active:
                self._episode_active = True
                return True
        elif self._episode_active:
            if self._clear_since is None:
                self._clear_since = received
            if received - self._clear_since >= SNAPSHOT_CLEAR_SECONDS:
                self._episode_active = False
        return False

    def snapshot_error(self):
        with self._lock:
            return self._snapshot_error

    def connection_status(self):
        with self._lock:
            if self._closed:
                return VIDEO_STOPPED
            if monotonic() - (self._received or self._created_at) > self._settings[-1]:
                return "카메라 프레임 수신이 지연되거나 중단되었습니다."
            return "카메라 연결됨" if self._received is not None else "카메라 프레임 대기 중"

    def _write_snapshot(self, save_detection, image, boxes, received, captured_at):
        try:
            draw_boxes(image, boxes)
            save_detection(image, boxes, received, captured_at)
            error = None
        except Exception as exc:
            error = f"FOD 사진 또는 이벤트 기록 저장 실패 · {exc}"
        with self._lock:
            self._snapshot_error = error

    def _analyze(self, frame, received, generation, settings, save_detection, captured_at):
        model, threshold, _, image_size, _ = settings
        try:
            image = frame.to_ndarray(format="bgr24")
            result = model.predict(image, threshold, imgsz=image_size)
            boxes = tuple(
                (tuple(value / size for value, size in zip(coords, (frame.width, frame.height) * 2)),
                 score)
                for coords, score in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist())
            )
            latest = (received, monotonic(), boxes)
            error = None
        except Exception:
            latest = None
            error = "YOLO 영상 추론에 실패했습니다. 모델과 카메라 설정을 확인해 주세요."
        publish = False
        with self._lock:
            self._busy = False
            if not self._closed and generation == self._generation:
                self._result, self._error = latest, error
                if latest is not None:
                    self._processing_ms = (latest[1] - received) * 1000
                    self._analysis_at = captured_at + timedelta(milliseconds=self._processing_ms)
                if latest is None:
                    self._clear_since = self._last_sample = None
                elif latest[1] - received <= settings[-1]:
                    publish = self._new_episode(boxes, received, max(settings[-1], settings[2] * 2))
        if publish and save_detection is not None:
            _SNAPSHOT_WRITER.submit(
                self._write_snapshot, save_detection, image, boxes, received, captured_at,
            )

    def recv(self, frame):
        """추론이 끝난 뒤 도착하는 프레임만 새 작업으로 채택합니다."""
        now = monotonic()
        with self._lock:
            if self._closed:
                return frame
            self._received = now
            boxes = self._current_result(now)
            interval = self._settings[2]
            if not self._busy and now - self._started >= interval:
                self._busy = True
                self._started = now
                Thread(
                    target=self._analyze,
                    args=(frame, now, self._generation, self._settings,
                          self._save_detection, datetime.now(timezone.utc)),
                    daemon=True,
                ).start()

        if not boxes:
            return frame
        image = frame.to_ndarray(format="bgr24")
        draw_boxes(image, boxes)
        output = type(frame).from_ndarray(image, format="bgr24")
        output.pts = frame.pts
        if frame.time_base is not None:
            output.time_base = frame.time_base
        return output
