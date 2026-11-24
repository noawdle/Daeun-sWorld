
"""단일 드론 착륙 판단 기준 보완 (시뮬레이션용)."""

from smart_vertiport.main import check_landing


def evaluate_landing(drone, vertiport):
    # 기존 팀원 알고리즘 실행
    result = check_landing(drone, vertiport)

    battery_low = drone["battery"] < 20
    emergency = drone["emergency"]
    status = result["status"]

    # 1. 기체 경고 수준 결정
    if emergency:
        alert_level = "긴급"
    elif battery_low:
        alert_level = "주의"
    else:
        alert_level = "정상"

    # 2. 착륙 판단에 따른 권고 조치
    if status == "접근 제한":
        action = "착륙장 접근 제한 및 위험요소 제거·재확인"

        if battery_low or emergency:
            action += " / 기체 상태 확인 및 대체 착륙지 검토"

    elif status == "대기":
        action = "착륙장 점유 해소 후 재판단"

        if battery_low or emergency:
            action += " / 지속 대기 위험 및 대체 착륙지 검토"

    elif status == "착륙 가능":
        action = "추가 안전 조건 확인 후 운영자 판단"

        if emergency:
            action = "긴급 원인 확인 및 안전한 대응 방안 검토"
        elif battery_low:
            action = "배터리 상태 확인 및 안전한 착륙 계획 검토"

    else:
        action = "판단 상태 확인 필요"

    # 3. 기존 결과에 보완 정보 추가
    result["alert_level"] = alert_level
    result["recommended_action"] = action

    return result


def evaluate_fod_landing(detection, *, pad_confirmed, telemetry_ready, communication_ready,
                         battery, emergency, unavailable_reason="최신 카메라 분석이 필요합니다."):
    """단일 기체 관제용 판단. 호출자는 현재 연결의 유효한 분석만 전달합니다."""
    alert = "긴급" if emergency else "주의" if battery is not None and battery < 20 else "정상"
    count = detection.get("fod_count") if detection is not None else None
    if detection is None:
        status, reason = "HOLD", unavailable_reason
    elif type(count) is not int or count < 0:
        status, reason = "HOLD", "FOD 탐지 개수를 확인할 수 없습니다."
    elif count > 0:
        status, reason = "UNSAFE", f"착륙 구역 FOD {count}개 탐지"
    else:
        pending = [
            label for passed, label in (
                (pad_confirmed, "패드 확인"), (telemetry_ready, "기체 상태"),
                (communication_ready, "통신 상태"), (alert == "정상", "기체 경고 해소"),
            ) if not passed
        ]
        status = "HOLD" if pending else "SAFE"
        reason = ("FOD 0개 · 확인 필요: " + ", ".join(pending) if pending
                  else "최신 FOD 미탐지 및 착륙 사전조건 충족")
    return {"status": status, "reason": reason, "alert_level": alert}
