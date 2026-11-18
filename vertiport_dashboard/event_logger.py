"""스마트 버티포트 시뮬레이션의 CSV 이벤트 기록 관리.

로컬 단일 프로세스 Streamlit 시연용 구현입니다.
"""

import csv
from collections import deque
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import Lock

KST = timezone(timedelta(hours=9))
LOG_FILE = Path(__file__).resolve().parent / "logs" / "event_log.csv"
COLUMNS = [
    "시간", "이벤트", "드론 ID", "FOD 개수", "배터리 (%)", "착륙 판단", "상세 내용",
]

# Streamlit이 여러 세션에서 같은 프로세스의 로그를 기록할 때 쓰기 순서를 보호합니다.
_LOG_LOCK = Lock()
# 간헐적인 근접 중복을 제거하되, 나중에 실제로 재발한 이벤트는 기록합니다.
_DECISION_DUPLICATE_WINDOW_SECONDS = 3


def _recent_duplicate(log_path, event, now):
    """같은 착륙 판단이 직전 3초 내 저장됐는지 확인합니다."""
    if event["이벤트"] != "착륙 판단 변경" or not log_path.exists():
        return False
    with log_path.open("r", newline="", encoding="utf-8-sig") as file:
        recent = deque(csv.DictReader(file), maxlen=30)

    fields = ["이벤트", "드론 ID", "FOD 개수", "배터리 (%)", "착륙 판단", "상세 내용"]
    for prior in reversed(recent):
        if not all(str(prior.get(field, "")) == str(event[field]) for field in fields):
            continue
        try:
            prior_at = datetime.strptime(prior["시간"], "%Y-%m-%d %H:%M:%S").replace(tzinfo=KST)
        except (KeyError, TypeError, ValueError):
            continue
        age = (now - prior_at).total_seconds()
        if 0 <= age <= _DECISION_DUPLICATE_WINDOW_SECONDS:
            return True
    return False


def save_event(
    event_type,
    drone_id="DRONE-01",
    fod_count=None,
    battery=None,
    status="",
    description="",
    log_path=LOG_FILE,
):
    """CSV에 이벤트를 저장합니다. 반환 형식은 이전 버전과 동일합니다.

    동일한 착륙 판단 이벤트가 3초 안에 반복되는 경우 파일에는 한 번만 기록합니다.
    FOD 분석 이벤트는 사용자가 재분석할 때마다 별도로 저장합니다.
    """
    now = datetime.now(KST)
    log_path = Path(log_path)
    event = {
        "시간": now.strftime("%Y-%m-%d %H:%M:%S"),
        "이벤트": event_type,
        "드론 ID": drone_id,
        "FOD 개수": "" if fod_count is None else fod_count,
        "배터리 (%)": "" if battery is None else battery,
        "착륙 판단": status,
        "상세 내용": description,
    }
    with _LOG_LOCK:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        if _recent_duplicate(log_path, event, now):
            return event
        has_data = log_path.exists() and log_path.stat().st_size > 0
        encoding = "utf-8" if has_data else "utf-8-sig"
        with log_path.open("a", newline="", encoding=encoding) as file:
            writer = csv.DictWriter(file, fieldnames=COLUMNS)
            if not has_data:
                writer.writeheader()
            writer.writerow(event)
    return event


def load_events(limit=30, log_path=LOG_FILE):
    """이벤트 기록을 최신순으로 가져옵니다."""
    log_path = Path(log_path)
    if not log_path.exists():
        return []
    with _LOG_LOCK:
        with log_path.open("r", newline="", encoding="utf-8-sig") as file:
            rows = list(csv.DictReader(file))
    return list(reversed(rows))[:limit]
