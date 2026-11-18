
"""단일 드론 비행 데이터 시뮬레이터.

실제 드론이나 비행제어기와 통신하지 않습니다.
"""

import math


def get_drone_state(elapsed_seconds, scenario="normal"):
    """경과 시간과 시나리오에 따른 가상 드론 상태 반환."""

    if not isinstance(elapsed_seconds, (int, float)):
        raise ValueError("경과 시간은 숫자여야 합니다.")

    if not math.isfinite(elapsed_seconds) or elapsed_seconds < 0:
        raise ValueError("경과 시간은 0 이상의 유한한 값이어야 합니다.")

    if scenario not in ("normal", "low_battery", "emergency"):
        raise ValueError("알 수 없는 시뮬레이션 시나리오입니다.")

    elapsed = int(elapsed_seconds)

    # 1. 배터리 자동 감소
    start_battery = 22 if scenario == "low_battery" else 68
    battery = max(0, start_battery - elapsed // 10)

    # 2. 고도 변화: 30m에서 12m까지 접근한 뒤 유지
    altitude = max(12.0, 30.0 - elapsed_seconds * 0.9)

    # 3. 20초 이후 착륙 요청
    landing_requested = elapsed >= 20

    # 4. 긴급 시나리오는 30초 이후 긴급 상태 발생
    emergency = scenario == "emergency" and elapsed >= 30

    # 5. 비행 단계
    if emergency:
        flight_phase = "긴급 상태"
    elif elapsed < 20:
        flight_phase = "착륙장 접근 중"
    else:
        flight_phase = "착륙 요청 후 대기"

    return {
        "id": "DRONE-01",
        "battery": battery,
        "altitude_m": round(altitude, 1),
        "emergency": emergency,
        "landing_requested": landing_requested,
        "flight_phase": flight_phase,
        "elapsed_seconds": elapsed,
        "data_source": "simulation",
    }
