
# 스마트 버티포트 시뮬레이션 자동 테스트
# STEP 4: 정상, 위험, 긴급 상황 검증

from main import decide_all_landings


# 기본 드론 데이터 생성
def create_drones():
    return [
        {"id": "Drone_A", "battery": 80, "emergency": False},
        {"id": "Drone_B", "battery": 15, "emergency": False},
        {"id": "Drone_C", "battery": 45, "emergency": True},
        {"id": "Drone_D", "battery": 60, "emergency": False}
    ]


# 기본 착륙장 데이터 생성
def create_vertiport():
    return {
        "person_detected": False,
        "vehicle_detected": False,
        "fod_detected": False,
        "pad_occupied": False
    }


# 테스트 시나리오 정의
test_cases = [
    {
        "id": "T01",
        "name": "정상 착륙장",
        "changes": {},
        "expected_order": ["Drone_C", "Drone_B", "Drone_D", "Drone_A"],
        "expected_status": ["착륙 가능", "대기", "대기", "대기"]
    },
    {
        "id": "T02",
        "name": "사람 탐지",
        "changes": {"person_detected": True},
        "expected_status": ["접근 제한"] * 4
    },
    {
        "id": "T03",
        "name": "차량 탐지",
        "changes": {"vehicle_detected": True},
        "expected_status": ["접근 제한"] * 4
    },
    {
        "id": "T04",
        "name": "FOD 탐지",
        "changes": {"fod_detected": True},
        "expected_status": ["접근 제한"] * 4
    },
    {
        "id": "T05",
        "name": "착륙장 점유",
        "changes": {"pad_occupied": True},
        "expected_status": ["대기"] * 4
    },
    {
        "id": "T06",
        "name": "배터리 부족 우선",
        "changes": {},
        "drone_changes": {
            "Drone_C": {"emergency": False}
        },
        "expected_first": "Drone_B",
        "expected_status": ["착륙 가능", "대기", "대기", "대기"]
    },
    {
        "id": "T07",
        "name": "긴급 상태 최우선",
        "changes": {},
        "expected_first": "Drone_C",
        "expected_status": ["착륙 가능", "대기", "대기", "대기"]
    },
    {
        "id": "T08",
        "name": "복합 위험 상황",
        "changes": {
            "person_detected": True,
            "fod_detected": True,
            "pad_occupied": True
        },
        "expected_status": ["접근 제한"] * 4
    }
]


# 단일 시나리오 실행
def run_test(test):

    drones = create_drones()
    vertiport = create_vertiport()

    # 착륙장 조건 변경
    vertiport.update(test["changes"])

    # 드론 조건 변경
    for drone in drones:
        changes = test.get("drone_changes", {})
        drone.update(changes.get(drone["id"], {}))

    # 실제 알고리즘 실행
    results = decide_all_landings(drones, vertiport)

    actual_order = [r["drone_id"] for r in results]
    actual_status = [r["status"] for r in results]
    actual_rank = [r["priority"] for r in results]

    # 예상 결과와 비교
    passed = actual_status == test["expected_status"]

    if "expected_order" in test:
        passed = passed and (
            actual_order == test["expected_order"]
        )

    if "expected_first" in test:
        passed = passed and (
            actual_order[0] == test["expected_first"]
        )

    # 우선순위 번호 검사
    passed = passed and (
        actual_rank == list(range(1, len(results) + 1))
    )

    print("\n--------------------------------")
    print(test["id"], test["name"])
    print("--------------------------------")

    for r in results:
        print(
            f'{r["priority"]}순위 | '
            f'{r["drone_id"]} | '
            f'배터리 {r["battery"]}% | '
            f'{r["status"]}'
        )

    if passed:
        print("TEST RESULT: PASS")
    else:
        print("TEST RESULT: FAIL")
        print("예상 상태:", test["expected_status"])
        print("실제 상태:", actual_status)
        print("실제 순서:", actual_order)

    return passed


# 전체 테스트 실행
def main():

    print("===================================")
    print(" 스마트 버티포트 시뮬레이션 테스트 ")
    print("===================================")

    passed_count = 0

    for test in test_cases:
        if run_test(test):
            passed_count += 1

    total = len(test_cases)

    print("\n===================================")
    print(" 최종 테스트 결과 ")
    print("===================================")
    print("전체 테스트:", total)
    print("성공:", passed_count)
    print("실패:", total - passed_count)
    print("통과율:", f"{passed_count / total * 100:.1f}%")

    if passed_count == total:
        print("모든 테스트 통과!")
    else:
        print("일부 테스트 실패: 원인 분석 필요")


if __name__ == "__main__":
    main()
