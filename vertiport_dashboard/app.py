"""스마트 버티포트 중간발표용 화면. 실행: python -m streamlit run app.py"""
import json
import math
import sys
from datetime import datetime, timezone, timedelta
from pathlib import Path

import streamlit as st
from fod_detection import render_fod_detection

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent

# 다른 폴더에 있는 착륙 판단 알고리즘 불러오기
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from smart_vertiport.main import check_landing


KST = timezone(timedelta(hours=9))
st.set_page_config(page_title="스마트 버티포트 | FOD 모니터링", page_icon="🛬", layout="wide")
st.markdown("""<style>
.stApp {background:#f5f7fb;}
.block-container {padding-top:2rem;max-width:1450px;}
h1,h2,h3 {color:#14243c;}
[data-testid="stMetric"] {background:white;border:1px solid #e3e9f2;border-radius:12px;padding:18px;}
</style>""", unsafe_allow_html=True)



def demo_data(scenario):
    """단일 드론 착륙 판단 시뮬레이션."""

    fod = scenario == "FOD 발견"

    aircraft = [
        {
            "id": "DRONE-01",
            "battery_pct": 68,
            "altitude_m": 12.5,
            "latitude": 37.0001,
            "longitude": 127.0001,
            "landing_requested": True,
            "emergency": False
        }
    ]

    drone = {
        "id": aircraft[0]["id"],
        "battery": aircraft[0]["battery_pct"],
        "emergency": aircraft[0]["emergency"]
    }

    vertiport = {
        "person_detected": False,
        "vehicle_detected": False,
        "fod_detected": fod,
        "pad_occupied": False
    }

    result = check_landing(drone, vertiport)

    status_map = {
        "착륙 가능": "ALLOWED",
        "대기": "WAIT",
        "접근 제한": "BLOCKED"
    }

    return {
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "fod_count": 2 if fod else 0,
        "confidence": 0.93 if fod else None,
        "decision": status_map[result["status"]],
        "reason": result["reason"],
        "warnings": result["warnings"],
        "aircraft": aircraft,
        "queue": []
    }



def number(value, lo, hi):
    return type(value) in (int, float) and math.isfinite(value) and lo <= value <= hi


def validate(data):
    """수신 데이터의 기본 형식만 검사한다. 판단 결과를 계산하지 않는다."""
    if not isinstance(data, dict):
        raise ValueError("JSON 최상위는 객체여야 합니다.")
    stamp = datetime.fromisoformat(data["updated_at"].replace("Z", "+00:00"))
    if stamp.tzinfo is None:
        raise ValueError("updated_at에 시간대(+00:00 등)가 필요합니다.")
    if data["decision"] not in ("ALLOWED", "WAIT", "BLOCKED", "UNKNOWN"):
        raise ValueError("decision 값이 올바르지 않습니다.")
    if not isinstance(data["reason"], str):
        raise ValueError("reason은 문자열이어야 합니다.")
    if type(data["fod_count"]) is not int or data["fod_count"] < 0:
        raise ValueError("fod_count는 0 이상의 정수여야 합니다.")
    if data.get("confidence") is not None and not number(data["confidence"], 0, 1):
        raise ValueError("confidence는 0~1 또는 null이어야 합니다.")
    if not isinstance(data["aircraft"], list) or not isinstance(data["queue"], list):
        raise ValueError("aircraft와 queue는 배열이어야 합니다.")
    for item in data["aircraft"]:
        if not isinstance(item["id"], str) or not item["id"]:
            raise ValueError("기체 ID가 필요합니다.")
        for field, lo, hi in (("battery_pct", 0, 100), ("altitude_m", -1000, 100000),
                              ("latitude", -90, 90), ("longitude", -180, 180)):
            if item.get(field) is not None and not number(item[field], lo, hi):
                raise ValueError(f"기체 정보 {field}의 범위를 확인하세요.")
        if type(item["landing_requested"]) is not bool:
            raise ValueError("landing_requested는 true/false여야 합니다.")
    ids = [item["id"] for item in data["aircraft"]]
    if len(ids) != len(set(ids)):
        raise ValueError("기체 ID가 중복되었습니다.")
    for item in data["queue"]:
        if (type(item["rank"]) is not int or item["rank"] < 1
                or item["id"] not in ids or not isinstance(item["reason"], str)):
            raise ValueError("queue의 순위·기체 ID·사유를 확인하세요.")
    return stamp


with st.sidebar:
    st.title("🛬 VERTIPORT")
    st.caption("스마트 버티포트 안전관리")
    source = st.radio("데이터 모드", ["AI FOD 탐지", "시뮬레이션", "JSON 파일 연동"], key="data_mode")
    if source != "AI FOD 탐지":
        scenario = st.selectbox( "시연 상황", ["정상 상태", "FOD 발견", "수신 중단"],  disabled=source != "시뮬레이션", key="scenario")
        auto = st.toggle("2초마다 화면 갱신", value=True)
        st.caption("JSON 연동은 data/live_state.json을 읽습니다.")
        st.divider()
        uploaded = st.file_uploader("착륙장 사진 표시 (선택)", type=["png", "jpg", "jpeg"])
        st.caption("이 모드에서는 사진을 참고용으로 표시합니다.")

st.title("스마트 버티포트 모니터링")
st.caption("FOD 탐지 상태 · 착륙 판단 결과 · 기체 정보")
if source == "AI FOD 탐지":
    team_weights = ROOT.parent / "train_model" / "weights" / "best.pt"
    standalone_weights = ROOT / "models" / "best.pt"
    weights = team_weights if team_weights.is_file() else standalone_weights
    render_fod_detection(weights, KST, check_landing)
    st.stop()

if source == "시뮬레이션":
    st.info(
        "시뮬레이션 모드입니다. 드론과 환경 정보는 가상 데이터이며, "
        "착륙 판단 및 우선순위는 실제 알고리즘으로 계산됩니다."
    )
else:
    st.info("외부 JSON 파일 수신 모드입니다. 실제 장비 연결 여부는 데이터를 쓰는 프로그램에서 확인하세요.")


@st.fragment(run_every=2 if auto else None)
def dashboard():
    st.button("지금 새로고침")
    try:
        data = (demo_data(scenario) if source == "시뮬레이션" else
                json.loads((ROOT / "data" / "live_state.json").read_text(encoding="utf-8")))
        if source == "시뮬레이션" and scenario == "수신 중단":
            data["updated_at"] = (datetime.now(timezone.utc) - timedelta(seconds=30)).isoformat()
        stamp = validate(data)
        age = (datetime.now(timezone.utc) - stamp).total_seconds()
        fresh = -5 <= age <= 10
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        st.error(f"데이터 수신 불가 · 판단 확인 필요: {error}")
        st.caption("이전 착륙 가능 결과를 표시하지 않습니다. README의 데이터 형식을 확인하세요.")
        return

    decision = data["decision"] if fresh else "UNKNOWN"
    labels = {"ALLOWED": "착륙 가능", "WAIT": "착륙 대기", "BLOCKED": "착륙 불가", "UNKNOWN": "판단 확인 필요"}
    cols = st.columns(4)
    cols[0].metric("착륙 판단", labels[decision])
    cols[1].metric("FOD 탐지 수", f"{data['fod_count']}개" if fresh else "—")
    cols[2].metric("등록 기체", f"{len(data['aircraft'])}대")
    cols[3].metric("데이터 상태", "수신 정상" if fresh else "수신 지연")
    st.caption(f"마지막 데이터 시각: {stamp.astimezone(KST):%Y-%m-%d %H:%M:%S} KST · 경과 {max(age, 0):.1f}초 · 10초 초과 시 판단 확인 필요")
    message = data["reason"] if fresh else "데이터가 오래되었거나 시각이 맞지 않습니다. 연결 상태를 확인하세요. 아래 정보는 마지막 수신값입니다."
    {"ALLOWED": st.success, "WAIT": st.warning, "BLOCKED": st.error, "UNKNOWN": st.warning}[decision](message)

    left, right = st.columns([1.35, 1])
    with left:
        st.subheader("착륙 패드")
        if uploaded is not None:
            st.image(uploaded, caption="수동 업로드 사진 · 탐지 결과 오버레이 없음", width="stretch")
        else:
            objects = ('<rect x="205" y="115" width="32" height="24" rx="3" fill="#ffb347"/>'
                       '<rect x="385" y="235" width="25" height="20" rx="3" fill="#ffb347"/>') if source == '시뮬레이션' and data['fod_count'] else ''
            svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 360">
            <rect width="640" height="360" rx="18" fill="#162b3e"/>
            <rect x="100" y="40" width="440" height="280" rx="12" fill="#234b4b" stroke="#7de1b4" stroke-width="3"/>
            <circle cx="320" cy="180" r="100" fill="none" stroke="#d9f7ec" stroke-width="5"/>
            <path d="M285 125V235M355 125V235M285 180H355" stroke="#d9f7ec" stroke-width="14"/>
            {objects}</svg>'''
            st.image(svg, width="stretch")
            st.caption("착륙장 모식도 · 실제 카메라 영상이 아닙니다. 주황색 표식은 시연용 FOD 위치입니다." if source == "시뮬레이션" else
                       "착륙장 모식도 · 실제 FOD 위치를 나타내지 않습니다.")
        confidence = data.get("confidence")
        st.caption(f"탐지 신뢰도: {confidence:.0%}" if fresh and confidence is not None else "탐지 신뢰도: —")
    with right:
        st.subheader("기체 정보")
        if data['aircraft']:
            chosen = st.selectbox("기체 선택", [a['id'] for a in data['aircraft']], key="aircraft")
            a = next(a for a in data['aircraft'] if a['id'] == chosen)
            battery, altitude = st.columns(2)
            battery.metric("배터리", "—" if a.get('battery_pct') is None else f"{a['battery_pct']}%")
            altitude.metric("고도", "—" if a.get('altitude_m') is None else f"{a['altitude_m']} m")
            st.write("위도:", a.get('latitude'), " / 경도:", a.get('longitude'))
            st.write("착륙 요청:", "요청 있음" if a['landing_requested'] else "요청 없음")
        else:
            st.info("등록된 기체가 없습니다.")
            st.subheader("착륙 요청 순위")
        if data['queue']:
            st.dataframe([{"순위": q['rank'], "기체": q['id'], "사유": q['reason']}
                          for q in sorted(data['queue'], key=lambda q: q['rank'])], hide_index=True, width="stretch")
        else:
            st.caption("대기 중인 요청이 없습니다.")
        if source == "시뮬레이션":
            st.caption("착륙 우선순위는 판단 알고리즘으로 계산됩니다.")
        else:
            st.caption(
                "외부 알고리즘이 전달한 순위를 표시합니다. "
                "수신 지연 시 순위도 최신 정보가 아닙니다."
            )
    with st.expander("현재 수신 데이터 확인"):
        st.json(data)


dashboard()
