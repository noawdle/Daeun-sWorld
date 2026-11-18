"""단일 드론 스마트 버티포트 통합 대시보드.

프로젝트 루트에서 실행:
    python -m streamlit run vertiport_dashboard/app.py

단일 이미지 YOLO 탐지 및 수동 입력을 사용하는 시연용 화면입니다.
실제 비행 허가/자동 착륙 제어에 사용하지 마세요.
"""

import math
import sys
from datetime import timedelta, timezone
from pathlib import Path

import streamlit as st

from fod_detection import render_fod_detection

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from smart_vertiport.landing_policy import evaluate_landing

KST = timezone(timedelta(hours=9))


def validate_landing_inputs(drone, vertiport, detection):
    """시연 데이터가 비정상이면 판단을 실행하지 않습니다."""
    if not isinstance(drone, dict) or not isinstance(vertiport, dict):
        raise ValueError("드론 및 환경 입력 형식 오류")
    if not isinstance(drone.get("id"), str) or not drone["id"].strip():
        raise ValueError("드론 ID 누락")
    battery = drone.get("battery")
    if (type(battery) not in (int, float) or not math.isfinite(battery)
            or not 0 <= battery <= 100):
        raise ValueError("배터리는 0~100 범위의 유효한 숫자여야 합니다.")
    if type(drone.get("emergency")) is not bool:
        raise ValueError("긴급 상태 값이 올바르지 않습니다.")

    for key in ("person_detected", "vehicle_detected", "fod_detected", "pad_occupied"):
        if type(vertiport.get(key)) is not bool:
            raise ValueError(f"환경 입력 오류: {key}")

    if not isinstance(detection, dict):
        raise ValueError("YOLO 탐지 결과 누락")
    count = detection.get("fod_count")
    if type(count) is not int or count < 0:
        raise ValueError("YOLO 탐지 개수 오류")
    if type(detection.get("fod_detected")) is not bool:
        raise ValueError("YOLO 탐지 여부 오류")
    if detection["fod_detected"] != (count > 0):
        raise ValueError("탐지 개수와 FOD 탐지 여부가 일치하지 않습니다.")
    if vertiport["fod_detected"] != detection["fod_detected"]:
        raise ValueError("YOLO 탐지 결과와 판단 입력이 다릅니다.")
    score = detection.get("confidence")
    if count == 0:
        if score is not None:
            raise ValueError("탐지 객체가 없는데 신뢰도 값이 있습니다.")
    elif (type(score) not in (int, float) or not math.isfinite(score)
          or not 0 <= score <= 1):
        raise ValueError("YOLO 탐지 신뢰도 오류")


st.set_page_config(
    page_title="스마트 버티포트 | 통합 모니터링",
    page_icon="🛬",
    layout="wide",
)
st.markdown(
    """<style>
    .stApp {background: #f5f7fb;}
    .block-container {padding-top: 2rem; max-width: 1450px;}
    h1, h2, h3 {color: #14243c;}
    [data-testid="stMetric"] {
        background: white; border: 1px solid #e3e9f2;
        border-radius: 12px; padding: 18px;
    }
    </style>""",
    unsafe_allow_html=True,
)

with st.sidebar:
    st.title("🛬 VERTIPORT")
    st.caption("단일 드론 안전관리 · 시뮬레이션")
    st.divider()

    st.subheader("드론 상태 (수동 입력)")
    drone_id = "DRONE-01"
    battery = st.slider("배터리 (%)", 0, 100, 68)
    emergency = st.checkbox("긴급 상태", value=False)
    landing_requested = st.checkbox("착륙 요청", value=True)
    altitude_m = st.number_input(
        "고도 (m, 시뮬레이션)", min_value=0.0, max_value=100000.0,
        value=12.5, step=0.5,
    )

    st.divider()
    st.subheader("추가 환경 조건 (수동 입력)")
    person_detected = st.checkbox("사람 있음 (가정)", value=False)
    vehicle_detected = st.checkbox("차량 있음 (가정)", value=False)
    pad_occupied = st.checkbox("착륙장 점유 중", value=False)
    st.caption("FOD 여부는 사진의 YOLO 탐지 결과로만 결정합니다.")

st.title("스마트 버티포트 통합 모니터링")
st.caption("YOLO FOD 탐지 · 단일 드론 상태 · 착륙 판단 알고리즘")
st.info(
    "연동 시연용 화면입니다. 드론·추가 환경 조건은 수동 입력이며, "
    "YOLO 결과는 업로드/촬영한 정지 이미지 기준입니다. 실제 착륙 허가가 아닙니다."
)

team_weights = PROJECT_ROOT / "train_model" / "weights" / "best.pt"
standalone_weights = ROOT / "models" / "best.pt"
weights = team_weights if team_weights.is_file() else standalone_weights


# 5초마다 상태를 다시 평가하므로 60초 경과 시 자동으로 '판단 확인 필요'로 변경됩니다.
# YOLO 자체는 버튼을 눌렀을 때만 실행되며, 매번 재추론하지 않습니다.
@st.fragment(run_every="5s")
def monitoring_panel():
    left, right = st.columns([1.3, 1], gap="large")

    with left:
        detection = render_fod_detection(weights, KST)

    with right:
        st.subheader("드론 상태 정보")
        st.write("기체 ID:", drone_id)
        col_battery, col_altitude = st.columns(2)
        col_battery.metric("배터리", f"{battery}%")
        col_altitude.metric("고도 (시뮬레이션)", f"{altitude_m:g} m")
        st.write("긴급 상태:", "긴급" if emergency else "정상")
        st.write("착륙 요청:", "요청 있음" if landing_requested else "요청 없음")

        st.divider()
        st.subheader("착륙 판단")

        # 분석 실패·결과 누락·유효시간 만료 시 판단을 보류합니다.
        if detection is None:
            st.metric("판단 상태", "판단 확인 필요")
            st.warning("유효한 FOD 탐지 결과가 없습니다. 사진을 분석해 주세요.")
        elif not landing_requested:
            st.metric("판단 상태", "요청 없음")
            st.info("드론의 착륙 요청이 없어 착륙 판단을 수행하지 않습니다.")
        else:
            drone = {
                "id": drone_id,
                "battery": battery,
                "emergency": emergency,
            }
            vertiport = {
                "person_detected": person_detected,
                "vehicle_detected": vehicle_detected,
                "fod_detected": detection["fod_detected"],
                "pad_occupied": pad_occupied,
            }

            try:
                validate_landing_inputs(drone, vertiport, detection)
                result = evaluate_landing(drone, vertiport)
                if result.get("status") not in ("접근 제한", "대기", "착륙 가능"):
                    raise ValueError("알 수 없는 착륙 판단 상태")
                if not isinstance(result.get("reason"), str):
                    raise ValueError("착륙 판단 사유 오류")
                if not isinstance(result.get("warnings"), list):
                    raise ValueError("기체 경고 목록 오류")
                if not isinstance(result.get("alert_level"), str):
                    raise ValueError("기체 경고 수준 오류")
                if not isinstance(result.get("recommended_action"), str):
                    raise ValueError("권고 조치 오류")
            except Exception as error:
                # 예상치 못한 데이터·알고리즘 오류에서도 착륙 가능을 표시하지 않음.
                st.metric("판단 상태", "판단 확인 필요")
                st.error(f"착륙 판단 처리 오류: {error}")
            else:
                status = result["status"]
                st.metric("판단 상태 (시연)", status)
                if status == "접근 제한":
                    st.error(result["reason"])
                elif status == "대기":
                    st.warning(result["reason"])
                else:
                    st.success("시연 조건에서 착륙 가능: " + result["reason"])

                for warning in result["warnings"]:
                    st.warning("기체 경고: " + str(warning))

                # 반복문 밖에서 항상 표시 (경고가 없는 정상 상황도 포함).
                st.write("기체 경고 수준:", result["alert_level"])
                st.info("권고 조치: " + result["recommended_action"])

                st.caption(
                    "판단 근거: YOLO FOD 탐지 결과와 수동 입력된 드론/환경 조건. "
                    "배터리 부족·긴급 상태는 현재 팀원 알고리즘에서 경고 조건입니다."
                )
                with st.expander("판단 입출력 확인"):
                    st.json({"drone": drone, "vertiport": vertiport, "result": result})


monitoring_panel()

st.caption(
    "주의: 현재 YOLO 모델의 탐지 대상 클래스가 모두 FOD라는 가정으로 탐지 수를 해석합니다. "
    "다른 클래스가 포함된 모델이라면 FOD 클래스만 분류하도록 필터링해야 합니다. "
    "미탐지 결과가 실제 착륙장 안전을 보장하지는 않습니다. "
    "결과 유효시간은 분석 완료 시각 기준이며 사진 촬영 시각을 보증하지 않습니다."
)
