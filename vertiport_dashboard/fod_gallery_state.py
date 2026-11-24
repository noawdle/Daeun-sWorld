"""Persist the gallery's hidden filenames separately from snapshot storage."""

import json
from pathlib import Path
from tempfile import NamedTemporaryFile
from threading import RLock


GALLERY_STATE_FILE = Path(__file__).resolve().parent / "data" / "fod_gallery_hidden.json"
_LOCK = RLock()


def load_hidden_photos():
    try:
        with _LOCK:
            payload = json.loads(GALLERY_STATE_FILE.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return set()
    if not isinstance(payload, list) or not all(isinstance(name, str) for name in payload):
        raise ValueError("사진 표시 상태 파일의 형식이 올바르지 않습니다.")
    return set(payload)


def hide_photos(photos):
    """Add the filenames visible at reset time without changing any snapshots."""
    with _LOCK:
        hidden = load_hidden_photos()
        hidden.update(photo["photo"] for photo in photos)
        GALLERY_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        handle = NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=GALLERY_STATE_FILE.parent,
            prefix=".fod_gallery_hidden-", suffix=".tmp", delete=False,
        )
        temporary = Path(handle.name)
        try:
            with handle:
                json.dump(sorted(hidden), handle)
            temporary.replace(GALLERY_STATE_FILE)
        finally:
            temporary.unlink(missing_ok=True)
