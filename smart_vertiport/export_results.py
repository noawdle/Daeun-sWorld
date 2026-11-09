
# 스마트 버티포트 안전관리 시스템
# STEP 5: JSON 출력 및 대시보드 연동 준비

import json
from datetime import datetime, timezone
from pathlib import Path

from main import decide_all_landings


# JSON 파일 저장 위치
OUTPUT_PATH = Path(__file__).resolve().parent / "landing_results.json"


# 1. 시뮬레이션 입력 데이터
def create_input_data():

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

    vertiport = {
        "person_detected": False,
        "vehicle_detected": False,
        "fod_detected": False,
        "pad_occupied": False
    }

    return drones, vertiport


# 2. 결과를 대시보드용 구조로 변환
def build_dashboard_data(drones, vertiport):

    results = decide_all_landings(drones, vertiport)

    # 착륙장 환경 정보
    environment = {
        "person_detected": vertiport["person_detected"],
        "vehicle_detected": vertiport["vehicle_detected"],
        "fod_detected": vertiport["fod_detected"],
        "pad_occupied": vertiport["pad_occupied"]
    }

    # 착륙 가능한 드론 확인
    landing_allowed = [
        r["drone_id"]
        for r in results
        if r["status"] == "착륙 가능"
    ]

    # 현재 판단 주기의 결과 요약
    summary = {
        "total_drones": len(results),
        "landing_allowed_count": len(landing_allowed),
        "waiting_count": sum(
            r["status"] == "대기" for r in results
        ),
        "restricted_count": sum(
            r["status"] == "접근 제한" for r in results
        ),
        "selected_drone": (
            landing_allowed[0] if landing_allowed else None
        )
    }

    # 최종 JSON 데이터
    dashboard_data = {
        "system": "Smart Vertiport Safety Management",
        "schema_version": "1.0",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "simulation": True,
        "environment": environment,
        "summary": summary,
        "drones": results
    }

    return dashboard_data


# 3. JSON 파일 저장
def save_json(data):

    with OUTPUT_PATH.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            data,
            file,
            ensure_ascii=False,
            indent=4
        )

    print("JSON 파일 저장 완료!")
    print("저장 위치:", OUTPUT_PATH)


# 4. 메인 실행
def main():

    drones, vertiport = create_input_data()

    dashboard_data = build_dashboard_data(
        drones, vertiport
    )

    save_json(dashboard_data)

    print("\n===== 시뮬레이션 결과 요약 =====")

    summary = dashboard_data["summary"]

    print("전체 드론:", summary["total_drones"])
    print("착륙 가능:", summary["landing_allowed_count"])
    print("대기:", summary["waiting_count"])
    print("접근 제한:", summary["restricted_count"])
    print("선택된 드론:", summary["selected_drone"])

    print("\n===== 드론별 판단 결과 =====")

    for drone in dashboard_data["drones"]:
        print(
            f'{drone["priority"]}순위 | '
            f'{drone["drone_id"]} | '
            f'{drone["status"]}'
        )


if __name__ == "__main__":
    main()
