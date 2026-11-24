"""단일 드론 스마트 버티포트 통합 대시보드.

프로젝트 루트에서 실행:
    python -m streamlit run vertiport_dashboard/app.py

이미지·실시간 영상 YOLO 탐지와 드론 시뮬레이션을 사용하는 시연용 화면입니다.
실제 비행 허가/자동 착륙 제어에 사용하지 마세요.
"""

import csv
import os
import sys
import platform
import time
from html import escape
from datetime import datetime
from pathlib import Path

import streamlit as st

from fod_detection import get_model, model_identity, render_fod_detection
from event_logger import KST, LOG_FILE, load_events, save_event
from drone_simulator import get_drone_state
from map_controller import MAP_LIMIT, MAX_ALTITUDE, local_target, render_map
from mission_controller import DemoMission, PAD, STAGES
from gazebo_bridge_client import BridgeError, CoordinateTransform, GazeboBridgeClient
from digital_twin import TwinTrace, bridge_view, demo_attitude
from telemetry_provider import DemoTelemetry, GazeboTelemetryProvider, unavailable_telemetry
from fod_snapshots import SNAPSHOT_DIR, load_snapshots, save_snapshot
from fod_gallery_state import hide_photos, load_hidden_photos
from video_detection import VIDEO_IDLE, VIDEO_STOPPED, VideoAnalysis

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from smart_vertiport.landing_policy import evaluate_fod_landing
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


st.set_page_config(page_title="스마트 버티포트 | 통합 관제", layout="wide", initial_sidebar_state="collapsed")
st.markdown('''<style>
.block-container {max-width: 1700px; padding-top: 1.4rem}
:is(.st-key-system_cards, .st-key-drone_cards, .st-key-fod_cards, .st-key-image_fod_cards, .st-key-simulation_cards)
    [data-testid="stMetric"], .control-card {
    background: #10233f; border: 1px solid #304967; border-radius: 10px;
}
:is(.st-key-system_cards, .st-key-drone_cards, .st-key-fod_cards, .st-key-image_fod_cards, .st-key-simulation_cards)
    [data-testid="stMetric"] {padding: 8px 10px; height: 100%;}
:is(.st-key-system_cards, .st-key-drone_cards, .st-key-fod_cards, .st-key-image_fod_cards, .st-key-simulation_cards)
    [data-testid="stMetricLabel"] :is(label, p, span) {color: #cbd5e1; font-weight: 500;}
:is(.st-key-system_cards, .st-key-drone_cards, .st-key-fod_cards, .st-key-image_fod_cards, .st-key-simulation_cards)
    [data-testid="stMetricLabel"] p {white-space: normal;}
:is(.st-key-system_cards, .st-key-drone_cards, .st-key-fod_cards, .st-key-image_fod_cards, .st-key-simulation_cards)
    [data-testid="stMetricValue"],
:is(.st-key-system_cards, .st-key-drone_cards, .st-key-fod_cards, .st-key-image_fod_cards, .st-key-simulation_cards)
    [data-testid="stMetricValue"] > div {
    color: #fff; font-size: 1.125rem; font-weight: 700; line-height: 1.3;
    white-space: normal; overflow-wrap: anywhere;
}
.control-card {padding: 12px 14px; color: #afc0d4; border-left: 4px solid var(--verdict);}
.control-card strong {display: block; color: var(--verdict); font-size: 1.35rem; margin: .2rem 0;}
.control-card small {color: #cbd5e1;}
.control-card.safe {--verdict: #22c55e;}
.control-card.hold {--verdict: #fbbf24;}
.control-card.unsafe {--verdict: #f87171;}
</style>''', unsafe_allow_html=True)

@st.fragment(run_every="1s")
def render_simulation_clock(scenario):
    """사이드바의 시계만 1초마다 갱신하고 YOLO 재추론은 하지 않음."""
    elapsed = simulation_elapsed()
    status = "진행 중" if st.session_state.get("sim_running") else "일시정지 / 대기"
    if elapsed >= SIMULATION_MAX_SECONDS:
        status = "120초 시연 종료"
    with st.container(key="simulation_cards"):
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


def set_gallery_reset_confirmation(open_):
    st.session_state["confirm_gallery_reset"] = open_
    st.session_state.pop("gallery_reset_error", None)


def confirm_gallery_reset():
    try:
        hide_photos(load_snapshots())
    except (OSError, ValueError) as error:
        st.session_state["gallery_reset_error"] = str(error)
    else:
        set_gallery_reset_confirmation(False)
        st.session_state.pop("gallery_selected_photo", None)


def render_gallery(photos):
    st.subheader("자동 저장된 FOD 탐지 사진")
    st.button("↻ 대시보드 사진 초기화", on_click=set_gallery_reset_confirmation, args=(True,))
    if st.session_state.get("confirm_gallery_reset"):
        st.warning("기존 FOD 사진을 대시보드에서 숨깁니다. 실제 저장된 사진 파일은 삭제되지 않습니다.")
        confirm, cancel = st.columns(2)
        confirm.button("초기화", key="confirm_fod_gallery_reset", on_click=confirm_gallery_reset)
        cancel.button("취소", key="cancel_fod_gallery_reset", on_click=set_gallery_reset_confirmation, args=(False,))
        if error := st.session_state.get("gallery_reset_error"):
            st.error(f"사진 표시 상태를 저장하지 못했습니다 · {error}")
    try:
        hidden = load_hidden_photos()
    except (OSError, ValueError) as error:
        st.warning(f"사진 표시 상태를 읽지 못했습니다 · {error}")
        return
    photos = [photo for photo in photos if photo["photo"] not in hidden]
    st.caption(f"표시 사진 {len(photos)}장")
    selection_key = "gallery_selected_photo"
    by_name = {photo["photo"]: photo for photo in photos}
    selected = st.session_state.get(selection_key)
    if selected is not None and selected not in by_name:
        st.session_state.pop(selection_key, None)
    if not photos:
        st.info(
            "표시할 FOD 탐지 사진이 없습니다. 기존 사진 파일은 안전하게 보관되어 있습니다."
            if hidden else "저장된 FOD 탐지 사진이 없습니다."
        )
        return
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




def log_mission(event, description, drone):
    try:
        save_event(event, drone_id=drone["id"], battery=drone["battery"], description=description)
    except OSError as error:
        st.warning(f"이벤트 기록 실패 · {error}")


def make_snapshot_saver(provider):
    elapsed, baseline = simulation_elapsed(), time.monotonic()
    running = st.session_state["sim_running"]

    def persist(image, boxes, received, captured_at):
        offset = max(0, received - baseline) if running else 0
        if isinstance(provider, GazeboTelemetryProvider):
            drone = provider.latest()
        elif provider is None:
            drone = unavailable_telemetry("미연동")
        else:
            drone = provider.read(min(SIMULATION_MAX_SECONDS, elapsed + offset))
        detection = {"fod_detected": True, "fod_count": len(boxes),
                     "confidence": max(score for _, score in boxes)}
        outcome = evaluate_fod_landing(
            detection, pad_confirmed=False, telemetry_ready=False, communication_ready=False,
            battery=drone["battery"], emergency=drone["emergency"],
        )
        save_snapshot(image, captured_at, detection, drone | {"id": drone["id"] or "—"}, outcome["status"])

    return persist


def render_mission(mission, drone, now):
    st.subheader("임무 제어")
    st.caption("DEMO · 로컬 시뮬레이션")
    if st.button("사전 점검", width="stretch", disabled=mission.stage not in (0, 8)):
        mission.prechecked = weights.is_file() and not drone["emergency"] and drone["battery"] >= 20
        log_mission("사전 점검", "DEMO 통과" if mission.prechecked else "모델 또는 기체 상태 확인 필요", drone)
    if st.button("임무 시작", width="stretch", type="primary",
                 disabled=not mission.prechecked or mission.stage not in (0, 8)):
        if mission.target[2] <= 0:
            st.warning("임무 시작에는 0 m보다 높은 목표 고도가 필요합니다.")
        elif weights.is_file() and not drone["emergency"] and drone["battery"] >= 20 and mission.start(now):
            log_mission("임무 시작", "DEMO 임무 시작", drone)
        else:
            mission.prechecked = False
            st.warning("사전 점검을 다시 수행해 주세요.")
    if st.button("임무 재개" if mission.paused_at is not None else "Hover / 일시정지", width="stretch",
                 disabled=mission.stage in (0, 8)):
        mission.toggle_pause(now)
        log_mission("임무 상태 변경", "DEMO 일시정지" if mission.paused_at is not None else "DEMO 재개", drone)
    if st.button("착륙 요청", width="stretch", disabled=mission.stage not in (4, 5) or mission.requested):
        mission.requested = True
        log_mission("착륙 요청", "DEMO 요청 대기", drone)
    if st.button("임무 취소", width="stretch", disabled=mission.stage in (0, 8)):
        mission.cancel()
        log_mission("임무 취소", "DEMO 임무 취소", drone)
    if st.button("패드 위치 확인", width="stretch", disabled=mission.stage != 4 or mission.pad_confirmed):
        mission.pad_confirmed = True
    if mission.prechecked:
        st.caption("사전 점검 통과")
    if drone["emergency"]:
        st.error("비상 대응 · 기체 긴급 상태")
    elif drone["battery"] < 20:
        st.warning("비상 대응 · 배터리 부족")
    else:
        st.caption("비상 대응 · 대기")
    for index, stage in enumerate(STAGES):
        if index == mission.stage:
            st.markdown(f"**● {stage}**" + (" · 일시정지" if mission.paused_at is not None else ""))
        else:
            st.caption(("✓ " if index < mission.stage else "○ ") + stage)


def submit_bridge_command(command, drone, action):
    try:
        if not (provider.latest()["fresh"] and drone["simulation_ready"] and bridge_client.enable_commands):
            raise BridgeError("BRIDGE_NOT_READY", "시뮬레이션 준비, 최신 텔레메트리 및 명령 설정을 확인해 주세요.")
        result = action()
        status = "승인" if result["accepted"] else "거절"
        message = f'{command} · {status} · ID {result.get("command_id") or "—"}'
        if not result["accepted"] and result.get("code"):
            message += f' · {result["code"]}'
        if detail := result.get("message"):
            message += f" · {detail}"
        st.session_state["bridge_command_result"] = (result["accepted"], message)
    except (BridgeError, ValueError) as error:
        message = f"{command} · {getattr(error, 'code', 'INVALID_COORDINATES')} · {error}"
        st.session_state["bridge_command_result"] = (False, message)
    log_mission("Bridge 명령", message, drone)


def render_bridge_controls(drone, outcome):
    st.subheader("Bridge 임무 제어")
    st.caption(drone["connection"])
    if bridge_client is None:
        return
    ready = drone["fresh"] and drone["simulation_ready"] and bridge_client.enable_commands
    if st.button("Hover", disabled=not ready):
        submit_bridge_command("Hover", drone, bridge_client.hover)
    if st.button("착륙 요청", key="bridge_land", disabled=not (
        ready and bridge_client.mock and outcome["status"] == "SAFE" and drone["telemetry_id"]
    )):
        submit_bridge_command("착륙 요청", drone, lambda: bridge_client.request_landing(
            safety=outcome["status"], telemetry_id=drone["telemetry_id"],
        ))
    if st.button("임무 취소", key="bridge_cancel", disabled=not ready):
        submit_bridge_command("임무 취소", drone, bridge_client.cancel_mission)
    if not bridge_client.mock:
        st.caption("Bridge 하향 영상의 안전 판단 연동 대기")


def render_target(mission, drone, now, detection, outcome):
    st.subheader("버티포트 디지털 트윈")
    draft = tuple(st.session_state.get(key, fallback)
                  for key, fallback in zip(("target_x", "target_y", "target_z"), mission.target))
    position, attitude, valid_for, path = None, None, None, []
    connection = drone["connection"]
    timestamp = drone["timestamp"]
    flight_mode = drone["flight_mode"]
    armed = drone["armed"]
    transform = bridge_client.transform if bridge_client else None
    trace = st.session_state["twin_trace"]
    source = (mode, drone["source"], transform)
    if mode == "DEMO":
        position, path = mission.position(now), mission.path
        attitude = demo_attitude(mission)
        timestamp = datetime.now(KST).isoformat()
        flight_mode, armed = STAGES[mission.stage], 0 < mission.stage < 8
        trace.update(source, None, None)
    elif mode == "GAZEBO":
        position, attitude, valid_for, connection = bridge_view(drone, transform)
        sampled_at = datetime.fromisoformat(timestamp) if position is not None else None
        path = trace.update(source, position, sampled_at)
    else:
        trace.update(source, None, None)
    twin = {
        "source": drone["source"], "mode": mode, "connection": connection,
        "attitude": attitude, "valid_for_ms": valid_for, "timestamp": timestamp,
        "altitude": position[2] if mode == "DEMO" else drone["altitude_m"],
        "flight_mode": flight_mode, "armed": armed,
        "fod": None if detection is None else {
            "count": detection["fod_count"], "confidence": detection["confidence"],
            "analyzed_at": detection["analyzed_at"].astimezone(KST).isoformat(),
            "valid_for_ms": detection["valid_for_ms"],
        },
        "safety": outcome["status"],
    }
    point = render_map(position, draft, path, PAD, st.session_state.get("handled_map_click"), twin=twin)
    if isinstance(point, dict) and point.get("id") and point["id"] != st.session_state.get("handled_map_click"):
        st.session_state["handled_map_click"] = point["id"]
        try:
            x, y, _ = local_target(point.get("x"), point.get("y"), draft[2])
            st.session_state.update(target_x=x, target_y=y)
            log_mission("목표 좌표 지정", f"지도 선택 X {x:.2f} / Y {y:.2f} / Z {draft[2]:.2f} m", drone)
        except ValueError as error:
            st.warning(str(error))
    st.caption("● 기체 위치 · □ 목표점 · PAD 설정 구역 · Z는 위쪽 고도")
    st.caption(f"현재 위치 · X {position[0]:.2f} / Y {position[1]:.2f} / Z {position[2]:.2f} m"
               if position is not None else "현재 지도 위치 · 미확인")
    coords = st.columns(3)
    values = [
        col.number_input(label, min_value=low, max_value=high, step=0.1, format="%.2f", key=key)
        for col, label, key, low, high in zip(
            coords, ("목표 X (m)", "목표 Y (m)", "목표 Z (m)"),
            ("target_x", "target_y", "target_z"),
            (-MAP_LIMIT, -MAP_LIMIT, 0.), (MAP_LIMIT, MAP_LIMIT, MAX_ALTITUDE),
        )
    ]
    disabled = (mission.stage not in (0, 8) if mode == "DEMO" else not (
        bridge_client and bridge_client.transform and bridge_client.enable_commands
        and drone["fresh"] and drone["simulation_ready"]
    ))
    if st.button("목표 좌표 전송", disabled=disabled):
        if mode == "DEMO":
            try:
                mission.set_target(local_target(*values))
                log_mission("목표 좌표 지정", f"DEMO 적용 · X/Y/Z {mission.target} m", drone)
            except ValueError as error:
                st.warning(str(error))
        else:
            submit_bridge_command("목표 좌표 전송", drone, lambda: bridge_client.send_target(*values))
    if mode == "DEMO":
        st.caption(f"DEMO 적용 목표 · {mission.target} m")
    elif mode == "GAZEBO":
        bridge_xyz = " / ".join(f"{drone[axis]:.2f}" if drone[axis] is not None else "—" for axis in ("x", "y", "z"))
        st.caption(f"Bridge ENU X/Y/Z · {bridge_xyz} m")
        if not bridge_client or bridge_client.transform is None:
            st.caption("목표 전송 · 좌표 변환 규칙 확인 대기")
        if bridge_client and not bridge_client.enable_commands:
            st.caption("HTTP 명령 전송 · 비활성화")


def render_status_strip(drone, mission):
    flight_mode = STAGES[mission.stage] if mode == "DEMO" else drone["flight_mode"]
    armed = 0 < mission.stage < 8 if mode == "DEMO" else drone["armed"]
    chips = (
        ("시스템", drone["connection"]), ("데이터 모드", mode), ("비행 모드", flight_mode or "—"),
        ("Armed", "—" if armed is None else "ARMED" if armed else "DISARMED"),
        ("GPS / 위성", f'{drone["gps"] or "—"} / {drone["satellites"] if drone["satellites"] is not None else "—"}'),
        ("배터리", f'{drone["battery"]}%' if drone["battery"] is not None else "—"),
        ("MAVLink", drone["mavlink"] or "미연동"), ("시스템 시간", datetime.now(KST).strftime("%H:%M:%S KST")),
    )
    with st.container(key="system_cards"):
        for column, (label, value) in zip(st.columns(len(chips)), chips):
            column.metric(label, value)
    st.caption(drone["source"])


def render_telemetry(drone, mission, camera):
    left, right = st.columns([1.5, 1])
    with left:
        st.subheader("실시간 드론 정보")
        st.caption(drone["source"])
        metrics = (
            ("드론 ID", drone["id"]), ("위도", drone["latitude"]), ("경도", drone["longitude"]),
            ("고도", f'{drone["altitude_m"]:g} m' if drone["altitude_m"] is not None else None),
            ("속도", f'{drone["speed"]:g} m/s' if drone["speed"] is not None else None),
            ("Roll / Pitch / Yaw", " / ".join(str(drone[axis]) if drone[axis] is not None else "—"
                                            for axis in ("roll", "pitch", "yaw"))),
            ("배터리 잔량", f'{drone["battery"]}%' if drone["battery"] is not None else None),
            ("전압", f'{drone["voltage"]:g} V' if drone["voltage"] is not None else None),
            ("비행 모드", STAGES[mission.stage] if mode == "DEMO" else drone["flight_mode"]), ("GPS", drone["gps"]),
        )
        with st.container(key="drone_cards"):
            for row in (metrics[:5], metrics[5:]):
                for column, (label, value) in zip(st.columns(5), row):
                    column.metric(label, "—" if value is None else value)
    with right:
        st.subheader("시스템 상태")
        st.write(f"YOLO 모델 · {'가중치 준비' if weights.is_file() else '가중치 없음'}")
        st.write(f"카메라 · {camera}")
        if mode == "GAZEBO":
            st.write(f'Bridge · {drone["connection"]}')
            if drone["error"]:
                st.caption(f'Bridge 상태 코드 · {drone["error"]}')
            st.write(f'시뮬레이션 준비 · {"확인" if drone["simulation_ready"] else "대기"}')
            if drone["camera_stream_url"]:
                st.caption("Bridge 하향 영상 주소 · 수신됨 · 분석 미연동")
                st.code(drone["camera_stream_url"], language=None)
            else:
                st.caption("Bridge 하향 영상 주소 · 미수신")
        else:
            st.write("MAVSDK / PX4 SITL / Gazebo · 미연동")
        st.write(f"AI 처리 컴퓨터 · {platform.system()} · Jetson 미연동")


@st.fragment(run_every="1s")
def render_operations():
    mission = st.session_state["demo_mission"]
    now = time.monotonic()
    drone = (provider.read(simulation_elapsed()) if provider is not None
             else unavailable_telemetry("미연동", error=bridge_config_error))
    video_mode = detection_mode == "실시간 영상 분석"
    detection, reason = (video_processor.snapshot() if video_mode and video_processor is not None
                         else (None, video_reason if video_mode else "착륙 판단에는 최신 카메라 분석이 필요합니다."))
    camera = (video_processor.connection_status() if video_mode and video_processor is not None
              else reason if video_mode else "이미지 업로드 모드")
    camera_event = None
    if video_mode and camera == VIDEO_STOPPED:
        camera_event = "카메라 종료" if video_processor is None else "통신 오류"
    elif video_mode and "중단되었습니다" in camera:
        camera_event = "통신 오류"
    if camera_event and camera_event != st.session_state.get("camera_event"):
        log_mission(camera_event, f"카메라 · {camera}", drone)
    st.session_state["camera_event"] = camera_event
    bridge_error = drone["error"] if mode == "GAZEBO" else None
    if bridge_error and bridge_error != st.session_state.get("last_bridge_error"):
        log_mission("Bridge 통신 오류", bridge_error, drone)
    st.session_state["last_bridge_error"] = bridge_error
    left, center = st.columns([1, 2.7], gap="medium")
    with left:
        if mode == "DEMO":
            render_mission(mission, drone, now)
        elif mode == "GAZEBO" and bridge_client and bridge_client.mock:
            st.checkbox("Mock 패드 확인", key="bridge_pad_confirmed")
        elif mode == "REAL":
            st.info("REAL · 미연동")
    mock_bridge = mode == "GAZEBO" and bridge_client is not None and bridge_client.mock
    pad_confirmed = mission.pad_confirmed if mode == "DEMO" else (
        mock_bridge and st.session_state.get("bridge_pad_confirmed", False)
    )
    vehicle_ready = mission.prechecked if mode == "DEMO" else (
        mock_bridge and drone["fresh"] and drone["armed"] is True and drone["id"] is not None
        and drone["battery"] is not None and drone["emergency"] is False
    )
    communication_ready = video_mode and detection is not None and (
        mode == "DEMO" or (mock_bridge and drone["fresh"] and drone["simulation_ready"])
    )
    outcome = evaluate_fod_landing(
        detection, pad_confirmed=pad_confirmed,
        telemetry_ready=vehicle_ready, communication_ready=communication_ready,
        battery=drone["battery"], emergency=drone["emergency"], unavailable_reason=reason,
    )
    if mode == "DEMO":
        if event := mission.tick(now, outcome["status"]):
            log_mission(*event, drone)
    elif mode == "GAZEBO":
        with left:
            render_bridge_controls(drone, outcome)
    with header_slot.container():
        render_status_strip(drone, mission)
    if video_mode:
        record_fod_state(detection, drone["id"], drone["battery"], detection_mode, reason)
    record_decision_change(outcome["status"], outcome["reason"], drone["id"], drone["battery"],
                           detection, outcome["alert_level"])
    with center:
        render_target(mission, drone, now, detection, outcome)
        if mode == "GAZEBO" and (result := st.session_state.get("bridge_command_result")):
            (st.success if result[0] else st.warning)(result[1])
    with ai_slot.container():
        if video_mode:
            timing = "—" if not detection else f'{detection["processing_ms"]:.0f} ms'
            timestamp = "—" if not detection else detection["analyzed_at"].astimezone(KST).strftime("%H:%M:%S KST")
            with st.container(key="fod_cards"):
                count, score, duration = st.columns(3)
                count.metric("FOD 탐지", f'{detection["fod_count"]}개' if detection else "미확인")
                score.metric("최고 신뢰도", f'{detection["confidence"]:.1%}' if detection and detection["confidence"] is not None else "—")
                duration.metric("AI 처리시간", timing)
            fod_status = "미확인" if detection is None else "탐지" if detection["fod_detected"] else "미탐지"
            st.caption(f"FOD 상태 · {fod_status}")
            st.caption(f"마지막 유효 분석 · {timestamp} · {camera}")
            if video_processor is not None and (error := video_processor.snapshot_error()):
                st.warning(error)
        status = outcome["status"]
        st.markdown(
            f'<div class="control-card {status.lower()}"><small>착륙 안전 판단 · {mode}</small>'
            f'<strong>{status}</strong>{escape(outcome["reason"])}</div>', unsafe_allow_html=True,
        )
        alert = outcome["alert_level"] if drone["battery"] is not None and drone["emergency"] is not None else "미확인"
        st.caption(f"기체 경고 · {alert}  /  패드 위치 · " + ("확인" if pad_confirmed else "확인 대기"))
    with telemetry_slot.container():
        render_telemetry(drone, mission, camera)


@st.fragment(run_every="1s")
def render_image_panel():
    with st.container(key="image_fod_cards"):
        detection = render_fod_detection(weights, KST)
    if isinstance(provider, GazeboTelemetryProvider):
        drone = provider.latest()
    else:
        drone = provider.read(simulation_elapsed()) if provider is not None else unavailable_telemetry("미연동")
    record_fod_state(detection, drone["id"], drone["battery"],
                     "이미지 업로드", "유효한 이미지 분석 결과 없음")


if "demo_mission" not in st.session_state:
    st.session_state["demo_mission"] = DemoMission()
if "twin_trace" not in st.session_state:
    st.session_state["twin_trace"] = TwinTrace()
for key, value in zip(("target_x", "target_y", "target_z"), st.session_state["demo_mission"].target):
    st.session_state.setdefault(key, value)

def clear_bridge_session():
    for key in ("bridge_command_result", "bridge_pad_confirmed", "bridge_provider"):
        st.session_state.pop(key, None)


st.title("스마트 버티포트 통합 관제")
mode = st.radio("데이터 모드", ["DEMO", "GAZEBO", "REAL"], horizontal=True, on_change=clear_bridge_session)
provider, bridge_client, bridge_config_error = None, None, None
if mode == "DEMO":
    provider = (DemoTelemetry(scenario=SIMULATION_SCENARIOS[scenario_label]) if data_source == "simulation"
                else DemoTelemetry(manual={"id": "DRONE-01", "battery": manual_battery,
                                           "altitude_m": manual_altitude, "emergency": manual_emergency}))
elif mode == "GAZEBO":
    mock_bridge = st.checkbox("Mock Bridge", on_change=clear_bridge_session)
    mock_transform = mock_bridge and st.checkbox("Mock 테스트 좌표 변환 사용", on_change=clear_bridge_session)
    try:
        if "bridge_provider" not in st.session_state:
            transform = None
            if mock_transform:
                transform = CoordinateTransform(90, (10, -2, 1))
            elif not mock_bridge and (raw := os.getenv("VERTIPORT_BRIDGE_TRANSFORM")):
                transform = CoordinateTransform.from_json(raw)
            bridge = GazeboBridgeClient(
                mock=mock_bridge, transform=transform,
                enable_commands=os.getenv("VERTIPORT_BRIDGE_ENABLE_COMMANDS") == "1",
            )
            st.session_state["bridge_provider"] = GazeboTelemetryProvider(bridge)
        provider = st.session_state["bridge_provider"]
        bridge_client = provider.client
    except (BridgeError, ValueError, OverflowError) as error:
        bridge_config_error = getattr(error, "code", "INVALID_TRANSFORM")
        st.warning(f"Bridge 설정 확인 · {error}")
header_slot = st.empty()
operations_col, camera_col = st.columns([2.6, 1.3], gap="medium")
with camera_col:
    detection_mode = st.radio("FOD 탐지 모드", ["실시간 영상 분석", "이미지 업로드"],
                              horizontal=True, on_change=reset_detection_mode)
    if detection_mode == "실시간 영상 분석":
        video_processor, video_reason = render_video_stream(make_snapshot_saver(provider))
    else:
        render_image_panel()
    ai_slot = st.empty()
telemetry_slot = st.empty()
with operations_col:
    render_operations()
render_history()
