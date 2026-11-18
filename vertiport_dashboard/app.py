"""단일 드론 스마트 버티포트 통합 대시보드.

프로젝트 루트에서 실행:
    python -m streamlit run vertiport_dashboard/app.py

단일 이미지 YOLO 탐지 및 수동 입력을 사용하는 시연용 화면입니다.
실제 비행 허가/자동 착륙 제어에 사용하지 마세요.
"""

import csv
import math
import sys
from html import escape
from datetime import timedelta, timezone
from pathlib import Path

import streamlit as st

from fod_detection import render_fod_detection
from event_logger import LOG_FILE, load_events, save_event

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


def record_fod_analysis(detection, drone_id, battery):
    """새 YOLO 분석 1회당 이벤트를 1회만 기록합니다."""
    if detection is None:
        return

    output = st.session_state.get("fod_output")
    if not isinstance(output, tuple) or len(output) != 5:
        return

    # result_key + 분석 완료 시각으로 새 분석과 5초 화면 갱신을 구분.
    analysis_key = (output[0], output[4])
    if st.session_state.get("last_logged_fod_analysis") == analysis_key:
        return

    fod_count = detection["fod_count"]
    try:
        save_event(
            "FOD 분석",
            drone_id=drone_id,
            fod_count=fod_count,
            battery=battery,
            description=f"YOLO 분석 완료: FOD {fod_count}개",
        )
    except OSError as error:
        st.warning(f"이벤트 기록 파일 저장 실패: {error}")
    else:
        st.session_state["last_logged_fod_analysis"] = analysis_key


def record_decision_change(status, reason, drone_id, battery, detection=None,
                           alert_level=""):
    """착륙 판단 상태·사유·경고 수준이 달라질 때만 기록합니다."""
    event_key = (status, reason, alert_level)
    if st.session_state.get("last_logged_decision") == event_key:
        return

    count = detection["fod_count"] if detection is not None else None
    details = reason if not alert_level else f"{reason} / 기체 경고 수준: {alert_level}"
    try:
        save_event(
            "착륙 판단 변경",
            drone_id=drone_id,
            fod_count=count,
            battery=battery,
            status=status,
            description=details,
        )
    except OSError as error:
        st.warning(f"이벤트 기록 파일 저장 실패: {error}")
    else:
        st.session_state["last_logged_decision"] = event_key


# STEP 5-3: 판단 기준과 데이터 흐름은 변경하지 않고 화면 표현만 보완합니다.
_STATUS_PRESENTATION = {
    "접근 제한": ("⛔", "danger", "위험요소가 확인되어 착륙장 접근을 제한합니다."),
    "대기": ("⏳", "warning", "착륙장이 점유 중이므로 대기합니다."),
    "착륙 가능": ("✅", "success", "현재 시연 조건에서 착륙 판단 기준을 충족했습니다."),
    "판단 확인 필요": ("🔎", "neutral", "유효한 탐지 결과 또는 입력값 확인이 필요합니다."),
    "요청 없음": ("⏸️", "info", "드론의 착륙 요청이 없습니다."),
}


def render_status_card(status):
    """상태를 색상·아이콘과 설명이 있는 카드로 표시합니다."""
    icon, tone, explanation = _STATUS_PRESENTATION.get(
        status, ("❔", "neutral", "알 수 없는 상태입니다.")
    )
    st.markdown(
        f"""<div class="landing-status landing-status--{tone}" role="status">
  <span class="landing-status__icon" aria-hidden="true">{icon}</span>
  <div class="landing-status__body">
    <div class="landing-status__eyebrow">착륙 판단 상태 · 시뮬레이션</div>
    <div class="landing-status__value">{escape(status)}</div>
    <div class="landing-status__description">{escape(explanation)}</div>
  </div>
</div>""",
        unsafe_allow_html=True,
    )


def render_condition_chips(detection, pad_occupied, emergency):
    """현재 판단에 사용되는 핵심 입력을 시각적으로 요약합니다."""
    if detection is None:
        fod_label, fod_tone = "FOD · 유효 결과 없음", "neutral"
    elif detection["fod_detected"]:
        fod_label, fod_tone = f'FOD · {detection["fod_count"]}개 탐지', "danger"
    else:
        fod_label, fod_tone = "FOD · 사진에서 미탐지", "success"

    pad_label = "착륙장 · 점유" if pad_occupied else "착륙장 · 미점유 (수동)"
    pad_tone = "warning" if pad_occupied else "neutral"
    emergency_label = "긴급 상태 · 입력됨" if emergency else "긴급 상태 · 없음 (수동)"
    emergency_tone = "danger" if emergency else "neutral"

    chips = ((fod_label, fod_tone), (pad_label, pad_tone),
             (emergency_label, emergency_tone))
    chips_html = "".join(
        f'<span class="landing-chip landing-chip--{tone}">{escape(label)}</span>'
        for label, tone in chips
    )
    st.markdown(f'<div class="landing-chips">{chips_html}</div>', unsafe_allow_html=True)


st.set_page_config(
    page_title="스마트 버티포트 | 통합 모니터링",
    page_icon="🛬",
    layout="wide",
)
st.markdown(
    """<style>
    .stApp {background: #f5f7fb;}
    .block-container {padding-top: 1.6rem; max-width: 1500px;}
    h1, h2, h3 {color: #14243c;}
    [data-testid="stMetric"] {
        background: white; border: 1px solid #e3e9f2;
        border-radius: 12px; padding: 18px;
    }
    /* STEP 5-4: 요약 상태 카드와 주요 지표가 상단에서 균형 있게 보이도록 조정합니다. */
    .top-summary-title {font-size: 18px; font-weight: 700; color: #14243c; margin: 0 0 9px;}
    .section-help {color: #64748b; font-size: 12px; margin-bottom: 10px;}
    /* STEP 5-3: 상태 카드는 동일한 형태를 유지하면서 색상만 구분합니다. */
    .landing-status {
        display: flex; align-items: center; gap: 16px;
        padding: 17px 18px; margin: 2px 0 8px;
        border: 1px solid; border-left-width: 6px;
        border-radius: 14px;
    }
    .landing-status__icon {font-size: 32px; flex-shrink: 0; line-height: 1.2;}
    .landing-status__body {min-width: 0;}
    .landing-status__eyebrow {font-size: 12px; font-weight: 600; opacity: .85;}
    .landing-status__value {
        font-size: clamp(23px, 2.3vw, 31px);
        line-height: 1.35; font-weight: 750; margin: 3px 0;
    }
    .landing-status__description {font-size: 13px; line-height: 1.5;}
    .landing-status--danger {background: #fff1f2; border-color: #fda4af; color: #9f1239;}
    .landing-status--warning {background: #fffbeb; border-color: #fcd34d; color: #92400e;}
    .landing-status--success {background: #ecfdf5; border-color: #6ee7b7; color: #065f46;}
    .landing-status--neutral {background: #f1f5f9; border-color: #cbd5e1; color: #334155;}
    .landing-status--info {background: #eff6ff; border-color: #93c5fd; color: #1e40af;}
    .landing-chips {display: flex; flex-wrap: wrap; gap: 7px; margin: 0 0 18px;}
    .landing-chip {
        display: inline-block; font-size: 12px; line-height: 1.35;
        padding: 6px 10px; border-radius: 999px; border: 1px solid;
        background: #f8fafc; border-color: #cbd5e1; color: #475569;
    }
    .landing-chip--danger {background: #fff1f2; border-color: #fda4af; color: #9f1239;}
    .landing-chip--warning {background: #fffbeb; border-color: #fcd34d; color: #92400e;}
    .landing-chip--success {background: #ecfdf5; border-color: #6ee7b7; color: #065f46;}
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


# STEP 5-4: YOLO/판단/기록 로직은 유지하고 화면 순서만 재배치합니다.
# 기존과 동일하게 5초마다 결과 유효시간을 재검사합니다.
@st.fragment(run_every="5s")
def monitoring_panel():
    # 이 영역을 먼저 확보해 두면 FOD 분석 후 계산한 판단 상태를
    # 화면 맨 위에 요약 카드로 보여줄 수 있습니다.
    summary_slot = st.empty()

    st.divider()
    col_detect, col_decision = st.columns([1.25, 1], gap="large")

    with col_detect:
        detection = render_fod_detection(weights, KST)

    # 각 YOLO 실행은 1번만 기록하고, 분석 버튼 이외의 5초 갱신은 제외합니다.
    record_fod_analysis(detection, drone_id, battery)

    # 기존 입력 검증, 착륙 판단 및 이벤트 기록 기준을 변경하지 않습니다.
    outcome = evaluate_current_state(detection)

    with summary_slot.container():
        st.markdown('<div class="top-summary-title">종합 모니터링 현황</div>',
                    unsafe_allow_html=True)
        st.caption(f"{drone_id} · 단일 드론 · 수동 입력 시뮬레이션")
        col_status, col_battery, col_altitude, col_fod = st.columns(
            [2.0, 1, 1, 1], gap="small"
        )
        with col_status:
            render_status_card(outcome["status"])
        with col_battery:
            st.metric("배터리", f"{battery}%")
        with col_altitude:
            st.metric("고도 (시뮬레이션)", f"{altitude_m:g} m")
        with col_fod:
            st.metric("FOD 탐지", "미확인" if detection is None
                      else f'{detection["fod_count"]}개')
        st.caption(
            "FOD는 업로드/촬영한 정지 이미지 기준이며, 배터리·고도·착륙장 정보는 수동 입력입니다. "
            "이 화면의 판단은 실제 착륙 허가가 아닙니다."
        )

    with col_decision:
        st.subheader("판단 근거 및 권고 조치")
        st.caption("현재 기체 상태와 착륙장 조건을 종합해 판단합니다.")
        render_condition_chips(detection, pad_occupied, emergency)

        if outcome["message_kind"] == "error":
            st.error(outcome["message"])
        elif outcome["message_kind"] == "warning":
            st.warning(outcome["message"])
        elif outcome["message_kind"] == "success":
            st.success(outcome["message"])
        else:
            st.info(outcome["message"])

        for warning in outcome["warnings"]:
            st.warning("기체 경고: " + str(warning))
        st.write("기체 경고 수준:", outcome["alert_level"])
        st.info("권고 조치: " + outcome["recommended_action"])
        st.caption(
            "판단 기준: FOD 탐지 및 수동 입력한 환경·기체 조건. "
            "배터리 부족·긴급 상태는 팀 알고리즘에서 경고 조건입니다."
        )
        if outcome["debug"] is not None:
            with st.expander("판단 입출력 확인"):
                st.json(outcome["debug"])

    # 과거처럼 모든 이벤트 기록을 길게 펼치지 않고 최근 8건만 먼저 표시합니다.
    # 상세 기록은 확장 영역에서 확인할 수 있으며 다운로드는 전체 CSV입니다.
    st.divider()
    col_title, col_info = st.columns([2, 1], gap="small")
    with col_title:
        st.subheader("최근 이벤트 기록")
    with col_info:
        st.caption("새 FOD 분석 · 판단 변경 · 결과 만료 시 자동 기록")
    try:
        recent = load_events(limit=8)
        if recent:
            st.dataframe(recent, hide_index=True, width="stretch", height=305)
            with st.expander("이전 이벤트 기록 30건 보기"):
                st.dataframe(load_events(limit=30), hide_index=True, width="stretch")
                st.caption("수동 테스트 기록도 포함될 수 있습니다.")
            st.download_button(
                "전체 이벤트 CSV 다운로드",
                data=LOG_FILE.read_bytes(),
                file_name="vertiport_event_log.csv",
                mime="text/csv",
                key="download_event_log_csv",
            )
        else:
            st.info("아직 기록된 이벤트가 없습니다. FOD 분석을 실행해 주세요.")
    except (OSError, csv.Error) as error:
        st.warning(f"이벤트 기록 불러오기 실패: {error}")


def evaluate_current_state(detection):
    """기존 STEP 5-3의 분기·기록 방식 그대로 반환값만 화면용으로 묶습니다."""
    if detection is None:
        record_decision_change(
            "판단 확인 필요", "유효한 FOD 탐지 결과 없음 또는 유효시간 만료",
            drone_id, battery,
        )
        return {
            "status": "판단 확인 필요",
            "message_kind": "warning",
            "message": "유효한 FOD 탐지 결과가 없습니다. 사진을 다시 분석해 주세요.",
            "warnings": [],
            "alert_level": "판단 보류",
            "recommended_action": "사진을 재분석하여 유효한 FOD 탐지 결과 확보",
            "debug": None,
        }

    if not landing_requested:
        record_decision_change(
            "요청 없음", "착륙 요청이 없어 판단 미실행",
            drone_id, battery, detection,
        )
        return {
            "status": "요청 없음",
            "message_kind": "info",
            "message": "드론의 착륙 요청이 없어 착륙 판단을 수행하지 않습니다.",
            "warnings": [],
            "alert_level": "—",
            "recommended_action": "착륙 요청 이후 탐지 결과 유효성을 확인하여 재판단",
            "debug": None,
        }

    drone = {"id": drone_id, "battery": battery, "emergency": emergency}
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
        record_decision_change(
            "판단 확인 필요", f"착륙 판단 처리 오류: {error}",
            drone_id, battery, detection,
        )
        return {
            "status": "판단 확인 필요",
            "message_kind": "error",
            "message": f"착륙 판단 처리 오류: {error}",
            "warnings": [],
            "alert_level": "판단 보류",
            "recommended_action": "입력값과 착륙 판단 처리 오류 확인 후 재검사",
            "debug": None,
        }

    status = result["status"]
    record_decision_change(
        status, result["reason"], drone_id, battery,
        detection, result["alert_level"],
    )
    if status == "접근 제한":
        message_kind, message = "error", result["reason"]
    elif status == "대기":
        message_kind, message = "warning", result["reason"]
    else:
        message_kind, message = "success", "시연 조건에서 착륙 가능: " + result["reason"]

    return {
        "status": status,
        "message_kind": message_kind,
        "message": message,
        "warnings": result["warnings"],
        "alert_level": result["alert_level"],
        "recommended_action": result["recommended_action"],
        "debug": {"drone": drone, "vertiport": vertiport, "result": result},
    }


monitoring_panel()

st.caption(
    "주의: YOLO 모델은 FOD 클래스만 학습한 모델을 사용합니다. "
    "다른 객체 탐지를 수행하지 않으며, 미탐지 결과가 실제 착륙장 안전을 보장하지는 않습니다. "
    "결과 유효시간은 분석 완료 시각 기준이며 사진 촬영 시각을 보증하지 않습니다."
)
