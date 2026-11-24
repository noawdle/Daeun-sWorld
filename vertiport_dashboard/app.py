"""단일 드론 스마트 버티포트 통합 대시보드.

프로젝트 루트에서 실행:
    python -m streamlit run vertiport_dashboard/app.py

이미지·실시간 영상 YOLO 탐지와 드론 시뮬레이션을 사용하는 시연용 화면입니다.
실제 비행 허가/자동 착륙 제어에 사용하지 마세요.
"""

import csv
import math
import sys
import time
from html import escape
from datetime import timedelta, timezone
from pathlib import Path

import streamlit as st

from fod_detection import get_model, model_identity, render_fod_detection
from event_logger import LOG_FILE, load_events, save_event
from drone_simulator import get_drone_state
from fod_snapshots import SNAPSHOT_DIR, load_snapshots, save_snapshot
from video_detection import VIDEO_IDLE, VIDEO_STOPPED, VideoAnalysis

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from smart_vertiport.landing_policy import evaluate_landing

KST = timezone(timedelta(hours=9))
SIMULATION_SCENARIOS = {
    "정상 비행": "normal",
    "배터리 부족": "low_battery",
    "긴급 상황": "emergency",
}

SIMULATION_MAX_SECONDS = 120


def simulation_elapsed():
    """시작/일시정지/재개를 반영한 가상 드론 경과 시간 (최대 120초)."""
    elapsed = float(st.session_state.get("sim_elapsed_base", 0.0))
    started_at = st.session_state.get("sim_started_at")
    if st.session_state.get("sim_running", False) and started_at is not None:
        elapsed += max(0.0, time.monotonic() - started_at)
    return min(SIMULATION_MAX_SECONDS, max(0.0, elapsed))


def start_simulation():
    """현재 경과 시간에서 재생 시작. 종료 상태면 0초부터 재시작."""
    if st.session_state.get("sim_running", False):
        return
    if simulation_elapsed() >= SIMULATION_MAX_SECONDS:
        st.session_state["sim_elapsed_base"] = 0.0
        st.session_state["sim_seek"] = 0
    st.session_state["sim_started_at"] = time.monotonic()
    st.session_state["sim_running"] = True


def pause_simulation():
    """현재 시각을 저장하고 일시정지."""
    elapsed = simulation_elapsed()
    st.session_state["sim_elapsed_base"] = elapsed
    st.session_state["sim_started_at"] = None
    st.session_state["sim_running"] = False
    st.session_state["sim_seek"] = int(elapsed)


def reset_simulation():
    """0초로 초기화. 시나리오 변경 시에도 사용."""
    st.session_state["sim_elapsed_base"] = 0.0
    st.session_state["sim_started_at"] = None
    st.session_state["sim_running"] = False
    st.session_state["sim_seek"] = 0


def seek_simulation():
    """정지 중에 슬라이더로 시간을 직접 지정."""
    st.session_state["sim_elapsed_base"] = float(st.session_state["sim_seek"])
    st.session_state["sim_started_at"] = None
    st.session_state["sim_running"] = False




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


def record_fod_state(detection, drone_id, battery, mode, unavailable_reason):
    """영상은 탐지 여부가 바뀔 때, 이미지는 새 분석도 기록합니다."""
    count = detection["fod_count"] if detection is not None else None
    state = None if count is None else count > 0
    output = st.session_state.get("fod_output") if mode == "이미지 업로드" else None
    analysis_time = output[4] if output is not None and detection is not None else None
    event_key = (mode, state, analysis_time)
    previous = st.session_state.get("last_logged_fod_state")
    if previous == event_key or (previous is None and state is None):
        return

    details = unavailable_reason if count is None else f"YOLO 분석 완료 · FOD {count}개"
    try:
        save_event(
            "FOD 분석" if analysis_time is not None else "FOD 상태 변경",
            drone_id=drone_id,
            fod_count=count,
            battery=battery,
            description=f"{mode} · {details}",
        )
    except OSError as error:
        st.warning(f"이벤트 기록 파일 저장 실패: {error}")
    else:
        st.session_state["last_logged_fod_state"] = event_key


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


def render_condition_chips(detection, pad_occupied, emergency, data_source):
    """현재 판단에 사용되는 핵심 입력을 시각적으로 요약합니다."""
    if detection is None:
        fod_label, fod_tone = "FOD · 유효 결과 없음", "neutral"
    elif detection["fod_detected"]:
        fod_label, fod_tone = f'FOD · {detection["fod_count"]}개 탐지', "danger"
    else:
        fod_label, fod_tone = "FOD · 미탐지", "success"

    pad_label = "착륙장 · 점유" if pad_occupied else "착륙장 · 미점유 (설정값)"
    pad_tone = "warning" if pad_occupied else "neutral"
    input_type = "시뮬레이션 데이터" if data_source == "simulation" else "직접 설정"
    emergency_label = (
        f"긴급 상태 · 발생 ({input_type})" if emergency
        else f"긴급 상태 · 없음 ({input_type})"
    )
    emergency_tone = "danger" if emergency else "neutral"

    chips = (
        (fod_label, fod_tone), (pad_label, pad_tone),
        ("사람 · 있음" if person_detected else "사람 · 없음", "danger" if person_detected else "neutral"),
        ("차량 · 있음" if vehicle_detected else "차량 · 없음", "danger" if vehicle_detected else "neutral"),
        (emergency_label, emergency_tone),
    )
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
    .block-container {padding-top: 1.1rem; max-width: 1500px;}
    h1, h2, h3 {color: #14243c;}
    [data-testid="stMetric"] {
        background: white; border: 1px solid #e3e9f2;
        border-radius: 12px; padding: 9px 12px;
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

@st.fragment(run_every="1s")
def render_simulation_clock(scenario):
    """사이드바의 시계만 1초마다 갱신하고 YOLO 재추론은 하지 않음."""
    elapsed = simulation_elapsed()
    status = "진행 중" if st.session_state.get("sim_running") else "일시정지 / 대기"
    if elapsed >= SIMULATION_MAX_SECONDS:
        status = "120초 시연 종료"
    st.metric("시뮬레이션 경과 시간", f"{int(elapsed)}초")
    state = get_drone_state(elapsed, scenario=scenario)
    st.caption(f"{status} · {state['flight_phase']} · 배터리 {state['battery']}% / 고도 {state['altitude_m']:g}m")


# 시뮬레이션 상태는 세션별로 저장됩니다 (서버 전체 공용 상태 아님).
for key, default in (
    ("sim_elapsed_base", 0.0),
    ("sim_started_at", None),
    ("sim_running", False),
    ("sim_seek", 0),
):
    if key not in st.session_state:
        st.session_state[key] = default

with st.sidebar:
    st.title("🛬 VERTIPORT")
    st.caption("단일 드론 안전관리 · 시뮬레이션")
    st.divider()

    st.subheader("드론 상태 설정")
    input_mode = st.radio(
        "드론 정보 방식", ["직접 설정", "드론 시뮬레이션"],
        horizontal=False,
        on_change=reset_simulation,
    )
    if input_mode == "직접 설정":
        data_source = "manual"
        manual_battery = st.slider("배터리 (%)", 0, 100, 68)
        manual_emergency = st.checkbox("긴급 상태", value=False)
        manual_landing_requested = st.checkbox("착륙 요청", value=True)
        manual_altitude = st.number_input(
            "고도 (m, 시뮬레이션)", min_value=0.0, max_value=100000.0,
            value=12.5, step=0.5,
        )
    else:
        data_source = "simulation"
        scenario_label = st.selectbox(
            "시뮬레이션 시나리오", list(SIMULATION_SCENARIOS),
            on_change=reset_simulation,
        )
        col_start, col_pause, col_reset = st.columns(3)
        with col_start:
            st.button(
                "▶ 시작", on_click=start_simulation,
                disabled=st.session_state["sim_running"],
                use_container_width=True,
            )
        with col_pause:
            st.button(
                "⏸ 정지", on_click=pause_simulation,
                disabled=not st.session_state["sim_running"],
                use_container_width=True,
            )
        with col_reset:
            st.button("↺ 초기화", on_click=reset_simulation, use_container_width=True)

        if not st.session_state["sim_running"]:
            st.slider(
                "경과 시간 직접 조절 (초)", 0, SIMULATION_MAX_SECONDS,
                key="sim_seek", on_change=seek_simulation,
            )
        else:
            st.caption("진행 중에는 시간 슬라이더가 숨겨집니다. 일시정지 후 직접 조절할 수 있습니다.")

        render_simulation_clock(SIMULATION_SCENARIOS[scenario_label])
        st.caption(
            "실제 기체와 통신하지 않습니다. 시작 버튼을 누르면 가상 시간이 흐르며 "
            "상단 상태 정보는 약 1초마다 갱신됩니다."
        )

    st.divider()
    st.subheader("착륙장 조건 설정")
    person_detected = st.checkbox("사람 있음", value=False)
    vehicle_detected = st.checkbox("차량 있음", value=False)
    pad_occupied = st.checkbox("착륙장 점유 중", value=False)
    st.caption("FOD 여부는 선택한 입력의 YOLO 탐지 결과로 결정합니다.")

st.title("스마트 버티포트 통합 모니터링")
st.caption("시뮬레이션 · 단일 드론 상태 및 착륙장 FOD 관제")

team_weights = PROJECT_ROOT / "train_model" / "weights" / "best.pt"
standalone_weights = ROOT / "models" / "best.pt"
weights = team_weights if team_weights.is_file() else standalone_weights


def reset_detection_mode():
    st.session_state.pop("fod_output", None)
    st.session_state.pop("fod_error", None)
    context = st.session_state.pop("active_fod_video", None)
    if context is not None and context.video_processor is not None:
        context.video_processor.on_ended()
    st.session_state.pop("fod_video_was_playing", None)


def render_video_stream(save_detection):
    """연결 컴포넌트는 폴링 fragment 밖에서 렌더링합니다."""
    st.subheader("실시간 카메라")
    try:
        from streamlit_webrtc import WebRtcMode, webrtc_streamer
    except ImportError:
        return None, "영상 분석 패키지를 불러오지 못했습니다. requirements.txt를 설치해 주세요."

    if not weights.is_file():
        return None, f"YOLO 가중치를 찾지 못했습니다. 경로 · {weights}"
    model = get_model(model_identity(weights))
    with st.expander("영상 분석 설정"):
        st.caption(f"사용 모델 · {weights.parent.name}/{weights.name}")
        threshold = st.slider("최소 탐지 신뢰도", 0.05, 0.95, 0.25, 0.05, key="video_threshold")
        interval = st.number_input("분석 간격 (초)", 0.2, 5.0, 0.5, 0.1)
        image_size = st.selectbox("YOLO 입력 크기", [320, 480, 640], index=2)
        max_age = st.number_input("결과 및 프레임 유효시간 (초)", 1.0, 20.0, 3.0, 0.5)
    if max_age <= interval:
        st.warning("분석 간격보다 유효시간을 길게 설정하면 분석 사이의 판단 보류를 줄일 수 있습니다.")
    settings = (model, threshold, interval, image_size, max_age)
    mounted_context = st.session_state.get("active_fod_video")
    if mounted_context is not None:
        # streamlit-webrtc 0.79.1 can count intervening reruns as a remount.
        # This page clears the mount marker when the user leaves video mode.
        mounted_context._last_rendered_run_count = None
    context = webrtc_streamer(
        key="fod_video_stream",
        mode=WebRtcMode.SENDRECV,
        media_stream_constraints={"video": True, "audio": False},
        video_processor_factory=lambda: VideoAnalysis(*settings, save_detection=save_detection),
        video_html_attrs={
            "autoPlay": True, "controls": False, "muted": True, "playsInline": True,
            "style": {"width": "100%", "height": "auto", "maxHeight": "320px", "objectFit": "contain"},
        },
        async_processing=False,
    )
    st.session_state["active_fod_video"] = context
    processor = context.video_processor
    if processor is not None:
        processor.configure(settings, save_detection)
    if context.state.playing:
        st.session_state["fod_video_was_playing"] = True
        return processor, "카메라 프레임을 기다리는 중입니다."
    if context.state.signalling:
        return None, "카메라 연결 중입니다. 브라우저의 카메라 권한을 확인해 주세요."
    if st.session_state.get("fod_video_was_playing"):
        return None, VIDEO_STOPPED
    return None, VIDEO_IDLE


def make_snapshot_saver():
    """캡처 시점의 기체·환경 입력으로 판단해 저장하는 작업을 만듭니다."""
    elapsed = simulation_elapsed()
    baseline = time.monotonic()
    running = st.session_state["sim_running"]
    scenario = SIMULATION_SCENARIOS[scenario_label] if data_source == "simulation" else None
    manual = None if scenario else {
        "id": "DRONE-01", "battery": manual_battery, "emergency": manual_emergency,
        "landing_requested": manual_landing_requested,
    }
    environment = (person_detected, vehicle_detected, pad_occupied)

    def persist(image, boxes, received, captured_at):
        drone = manual
        if scenario:
            offset = max(0, received - baseline) if running else 0
            drone = get_drone_state(min(SIMULATION_MAX_SECONDS, elapsed + offset), scenario=scenario)
        detection = {"fod_detected": True, "fod_count": len(boxes),
                     "confidence": max(score for _, score in boxes)}
        outcome = evaluate_current_state(
            detection, drone["id"], drone["battery"], drone["emergency"],
            drone["landing_requested"], *environment, record=False,
        )
        save_snapshot(image, captured_at, detection, drone, outcome["status"])

    return persist


@st.fragment(run_every="1s")
def monitoring_panel():
    # fragment가 갱신될 때마다 가상 시간을 재계산합니다.
    # 전역 변수로 최초 1회 계산하면 시간이 흘러도 값이 바뀌지 않습니다.
    if data_source == "simulation":
        simulated = get_drone_state(
            simulation_elapsed(), scenario=SIMULATION_SCENARIOS[scenario_label]
        )
        drone_id = simulated["id"]
        battery = simulated["battery"]
        altitude_m = simulated["altitude_m"]
        emergency = simulated["emergency"]
        landing_requested = simulated["landing_requested"]
        flight_phase = simulated["flight_phase"]
        time_label = f" · 경과 시간: {simulated['elapsed_seconds']}초"
    else:
        drone_id = "DRONE-01"
        battery = manual_battery
        altitude_m = manual_altitude
        emergency = manual_emergency
        landing_requested = manual_landing_requested
        flight_phase = "직접 설정"
        time_label = ""

    if detection_mode == "이미지 업로드":
        image_col, summary_col = st.columns([1.35, 1], gap="medium")
        summary_slot = summary_col.empty()
    else:
        summary_slot = video_summary
    col_detect, col_decision = st.columns([1.35, 1], gap="medium")

    if detection_mode == "이미지 업로드":
        with image_col:
            detection = render_fod_detection(weights, KST)
            unavailable_reason = (
                "YOLO 이미지 추론에 실패했습니다." if "fod_error" in st.session_state
                else "유효한 이미지 분석 결과가 없습니다."
            )
    else:
        detection, unavailable_reason = (
            video_processor.snapshot() if video_processor is not None else (None, video_reason)
        )
        video_connection.caption(video_processor.connection_status() if video_processor else video_reason)

    with col_detect:
        st.subheader("실시간 FOD 탐지 결과" if detection_mode == "실시간 영상 분석" else "이미지 FOD 탐지 결과")
        count_col, score_col = st.columns(2)
        count_col.metric("FOD 탐지 수", f'{detection["fod_count"]}개' if detection else "미확인")
        score = detection["confidence"] if detection else None
        score_col.metric("최고 탐지 신뢰도", "—" if score is None else f"{score:.1%}")
        st.caption(unavailable_reason if detection is None else
                   ("FOD 탐지" if detection["fod_detected"] else "FOD 미탐지"))
        if detection_mode == "실시간 영상 분석" and video_processor is not None:
            if error := video_processor.snapshot_error():
                st.warning(error)

    record_fod_state(detection, drone_id, battery, detection_mode, unavailable_reason)

    outcome = evaluate_current_state(
        detection, drone_id, battery, emergency, landing_requested,
        person_detected, vehicle_detected, pad_occupied, unavailable_reason,
    )

    with summary_slot.container():
        st.markdown('<div class="top-summary-title">종합 모니터링 현황</div>',
                    unsafe_allow_html=True)
        source_label = "드론 시뮬레이션 데이터" if data_source == "simulation" else "직접 설정 데이터"
        st.caption(f"{drone_id} · 단일 드론 · {source_label} · 비행 단계: {flight_phase}{time_label}")
        render_status_card(outcome["status"])
        col_battery, col_altitude = st.columns(2, gap="small")
        col_fod, col_emergency = st.columns(2, gap="small")
        with col_battery:
            st.metric("배터리", f"{battery}%")
        with col_altitude:
            st.metric("고도 (시뮬레이션)", f"{altitude_m:g} m")
        with col_fod:
            st.metric("FOD 탐지", "미확인" if detection is None
                      else f'{detection["fod_count"]}개')
        col_emergency.metric("긴급 상태", "발생" if emergency else "없음")
        st.info("권고 조치 · " + outcome["recommended_action"])

    with col_decision:
        st.subheader("판단 근거 및 권고 조치")
        st.caption("현재 기체 상태와 착륙장 조건을 종합해 판단합니다.")
        render_condition_chips(detection, pad_occupied, emergency, data_source)

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
            "판단 기준: FOD 탐지 및 환경 조건·기체 상태 데이터. "
            "배터리 부족·긴급 상태는 팀 알고리즘에서 경고 조건입니다."
        )
        if outcome["debug"] is not None:
            with st.expander("판단 입출력 확인"):
                st.json(outcome["debug"])


@st.fragment(run_every="5s")
def render_history():
    st.divider()
    col_title, col_info = st.columns([2, 1], gap="small")
    with col_title:
        st.subheader("최근 이벤트 기록")
    with col_info:
        st.caption("이미지 분석 · FOD 상태 변경 · 착륙 판단 변경 자동 기록")
    photos = load_snapshots()
    photos_by_name = {record["photo"]: record for record in photos}
    try:
        events = load_events(limit=None)
    except (OSError, csv.Error) as error:
        st.warning(f"이벤트 기록 불러오기 실패: {error}")
        events = []
    linked = {event["상세 내용"] for event in events if event["이벤트"] == "FOD 사진 저장"}
    for photo in photos:
        if photo["photo"] not in linked:
            events.append({
                "시간": photo["captured_at"][:19].replace("T", " "), "이벤트": "FOD 사진 저장",
                "드론 ID": photo["drone_id"], "FOD 개수": photo["fod_count"],
                "배터리 (%)": photo.get("battery", ""), "착륙 판단": photo["status"],
                "상세 내용": photo["photo"],
            })
    events.sort(key=lambda event: event["시간"], reverse=True)
    for event in events[:30]:
        photo = photos_by_name.get(event["상세 내용"]) if event["이벤트"] == "FOD 사진 저장" else None
        event["최고 신뢰도"] = f'{photo["confidence"]:.1%}' if photo else ""
        event["사진"] = photo["photo"] if photo else ""
    if events:
        st.dataframe(events[:8], hide_index=True, width="stretch", height=305)
        with st.expander("최근 이벤트 30건 보기"):
            st.dataframe(events[:30], hide_index=True, width="stretch")
    else:
        st.info("아직 기록된 이벤트가 없습니다.")
    if LOG_FILE.is_file():
        try:
            st.download_button(
                "전체 이벤트 CSV 다운로드",
                data=LOG_FILE.read_bytes(),
                file_name="vertiport_event_log.csv",
                mime="text/csv",
                key="download_event_log_csv",
                on_click="ignore",
            )
        except OSError as error:
            st.warning(f"CSV 다운로드 준비 실패 · {error}")
    render_gallery(photos)


def render_gallery(photos):
    st.subheader("자동 저장된 FOD 탐지 사진")
    if not photos:
        st.info("저장된 FOD 탐지 사진이 없습니다.")
        return
    selection_key = "gallery_selected_photo"
    for column, photo in zip(st.columns(3), photos[:3]):
        with column, st.container(border=True):
            try:
                st.image((SNAPSHOT_DIR / photo["photo"]).read_bytes(), width="stretch")
                st.caption(photo_caption(photo))
                st.button(
                    "사진 크게 보기", key=f"open_{photo['photo']}",
                    on_click=st.session_state.update, args=({selection_key: photo["photo"]},),
                )
            except OSError as error:
                st.warning(f"사진을 불러올 수 없습니다 · {error}")
    with st.expander("전체 탐지 이력 및 사진 상세"):
        by_name = {photo["photo"]: photo for photo in photos}
        name = st.selectbox(
            "탐지 사진 선택", list(by_name), index=None, key=selection_key,
            format_func=lambda filename: f"{photo_caption(by_name[filename])} · {filename[-12:-4]}",
        )
    if name:
        try:
            jpeg = (SNAPSHOT_DIR / name).read_bytes()
            st.image(jpeg, caption=photo_caption(by_name[name]), width="stretch")
            st.download_button("탐지 사진 다운로드", jpeg, name, "image/jpeg", on_click="ignore")
        except OSError as error:
            st.warning(f"사진을 불러올 수 없습니다 · {error}")


def photo_caption(photo):
    time_label = photo["captured_at"][:19].replace("T", " ")
    return (f'{time_label} KST · {photo["drone_id"]} · FOD {photo["fod_count"]}개 · '
            f'{photo["confidence"]:.1%} · {photo["status"]}')


def evaluate_current_state(detection, drone_id, battery, emergency,
                           landing_requested, person_detected, vehicle_detected,
                           pad_occupied, unavailable_reason="유효한 FOD 탐지 결과가 없습니다.",
                           *, record=True):
    """직접 설정/시뮬레이션 데이터에 동일한 판단 규칙을 적용합니다."""
    log_decision = record_decision_change if record else lambda *args: None
    if detection is None:
        log_decision(
            "판단 확인 필요", unavailable_reason,
            drone_id, battery,
        )
        return {
            "status": "판단 확인 필요",
            "message_kind": "warning",
            "message": unavailable_reason,
            "warnings": [],
            "alert_level": "판단 보류",
            "recommended_action": "현재 FOD 입력을 확인하고 최신 분석 결과 확보",
            "debug": None,
        }

    if not landing_requested:
        log_decision(
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
        log_decision(
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
    log_decision(
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


detection_mode = st.radio(
    "FOD 탐지 모드", ["실시간 영상 분석", "이미지 업로드"],
    horizontal=True, on_change=reset_detection_mode,
)
if detection_mode == "실시간 영상 분석":
    camera_col, summary_col = st.columns([1.35, 1], gap="medium")
    with camera_col:
        video_processor, video_reason = render_video_stream(make_snapshot_saver())
        video_connection = st.empty()
    video_summary = summary_col.empty()

monitoring_panel()
render_history()

st.caption(
    "주의: YOLO 모델은 FOD 클래스만 학습한 모델을 사용합니다. "
    "다른 객체 탐지를 수행하지 않으며, 미탐지 결과가 실제 착륙장 안전을 보장하지는 않습니다. "
    "이미지 분석 완료 시각으로 사진 촬영 시각을 확인할 수 없습니다."
)
