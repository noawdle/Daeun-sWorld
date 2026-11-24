"""자동 탐지 사진과 당시 판단 정보의 로컬 저장소."""

import json
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from event_logger import KST, save_event


SNAPSHOT_DIR = Path(__file__).resolve().parent / "data" / "fod_snapshots"


def _publish(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        with temporary.open("xb") as file:
            file.write(payload)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def save_snapshot(image, captured_at, detection, drone, status, directory=SNAPSHOT_DIR):
    import cv2

    captured_at = captured_at.astimezone(KST)
    stem = f"FOD_{captured_at:%Y%m%d_%H%M%S}_{uuid4().hex}"
    filename = f"{stem}.jpg"
    metadata = {
        "photo": filename,
        "captured_at": captured_at.isoformat(timespec="microseconds"),
        "drone_id": drone["id"],
        "battery": drone["battery"],
        "fod_count": detection["fod_count"],
        "confidence": detection["confidence"],
        "status": status,
    }
    ok, jpeg = cv2.imencode(".jpg", image)
    if not ok:
        raise OSError("탐지 사진을 JPEG로 변환할 수 없습니다.")
    directory.mkdir(parents=True, exist_ok=True)
    photo = directory / filename
    _publish(photo, jpeg.tobytes())
    try:
        _publish(directory / f"{stem}.json", json.dumps(metadata, ensure_ascii=False).encode("utf-8"))
    except OSError:
        photo.unlink(missing_ok=True)
        raise
    save_event(
        "FOD 사진 저장", drone_id=drone["id"], battery=drone["battery"],
        fod_count=detection["fod_count"], status=status, description=filename,
        occurred_at=captured_at,
    )


def load_snapshots(directory=SNAPSHOT_DIR):
    records = []
    for path in directory.glob("FOD_*.json"):
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
            if record["photo"] != path.with_suffix(".jpg").name:
                continue
            if not (directory / record["photo"]).is_file():
                continue
            if not {"captured_at", "drone_id", "fod_count", "confidence", "status"} <= record.keys():
                continue
            if datetime.fromisoformat(record["captured_at"]).utcoffset() != KST.utcoffset(None):
                continue
            if type(record["confidence"]) not in (float, int) or not 0 <= record["confidence"] <= 1:
                continue
        except (OSError, ValueError, TypeError, KeyError):
            continue
        records.append(record)
    return sorted(records, key=lambda record: record["captured_at"], reverse=True)
