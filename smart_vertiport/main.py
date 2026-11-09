
# 스마트 버티포트 안전관리 시스템
# STEP 3: 다중 드론 착륙 우선순위 판단


# 1. 개별 드론 착륙 가능 여부 판단
def check_landing(drone, vertiport):

    battery = drone["battery"]
    emergency = drone["emergency"]

    warnings = []

    if battery < 20:
        warnings.append("배터리 부족")

    if emergency:
        warnings.append("긴급 상태")

    hazards = []

    if vertiport["person_detected"]:
        hazards.append("사람 탐지")

    if vertiport["vehicle_detected"]:
        hazards.append("차량 탐지")

    if vertiport["fod_detected"]:
        hazards.append("FOD 탐지")

    if hazards:
        status = "접근 제한"
        reason = ", ".join(hazards)

    elif vertiport["pad_occupied"]:
        status = "대기"
        reason = "착륙장 점유 중"

    else:
        status = "착륙 가능"
        reason = "착륙장 안전 조건 충족"

    return {
        "drone_id": drone["id"],
        "battery": battery,
        "emergency": emergency,
        "status": status,
        "reason": reason,
        "warnings": warnings
    }


# 2. 드론 우선순위 계산 기준
def priority_key(drone):

    if drone["emergency"]:
        group = 0

    elif drone["battery"] < 20:
        group = 1

    else:
        group = 2

    return (group, drone["battery"])


# 3. 전체 드론 착륙 순서 결정
def decide_all_landings(drones, vertiport):

    # 각 드론의 기본 착륙 판단
    results = []

    for drone in drones:
        result = check_landing(drone, vertiport)
        results.append(result)

    # 우선순위 정렬
    ordered = sorted(results, key=priority_key)

    # 착륙장 안전 상태 확인
    safe = not (
        vertiport["person_detected"]
        or vertiport["vehicle_detected"]
        or vertiport["fod_detected"]
    )

    free = not vertiport["pad_occupied"]

    # 우선순위 부여
    for rank, result in enumerate(ordered, start=1):

        result["priority"] = rank

        if safe and free:

            if rank == 1:
                result["status"] = "착륙 가능"
                result["reason"] = (
                    "착륙장 안전 조건 충족, 우선순위 1위"
                )

            else:
                result["status"] = "대기"
                result["reason"] = (
                    "상위 우선순위 드론 착륙 대기"
                )

    return ordered


# 4. 메인 실행
def main():

    # 가상 드론 데이터
    drones = [
        {
            "id": "Drone_A",
            "battery": 80,
            "emergency": False
        },
        {
            "id": "Drone_B",
            "battery": 15,
            "emergency": False
        },
        {
            "id": "Drone_C",
            "battery": 45,
            "emergency": True
        },
        {
            "id": "Drone_D",
            "battery": 60,
            "emergency": False
        }
    ]

    # 버티포트 환경 데이터
    vertiport = {
        "person_detected": False,
        "vehicle_detected": False,
        "fod_detected": False,
        "pad_occupied": False
    }

    # 전체 드론 착륙 판단
    results = decide_all_landings(drones, vertiport)

    print("===== 스마트 버티포트 착륙 우선순위 =====")

    for result in results:

        print("----------------------------")
        print("우선순위:", result["priority"])
        print("드론:", result["drone_id"])
        print("배터리:", result["battery"], "%")
        print("긴급 상태:", result["emergency"])
        print("착륙 판단:", result["status"])
        print("판단 이유:", result["reason"])
        print("경고:", result["warnings"])


if __name__ == "__main__":
    main()
