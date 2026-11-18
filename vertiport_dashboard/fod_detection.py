"""팀 YOLO 모델과 이미지 업로드 FOD 분석 화면."""

from datetime import datetime, timezone
from hashlib import sha256
from io import BytesIO
from threading import Lock

import streamlit as st
from PIL import Image, ImageOps, UnidentifiedImageError

# 시연용: 분석 완료 후 60초가 지나면 결과를 더 이상 착륙 판단에 사용하지 않습니다.
# 주의: 사진 자체의 촬영 시각을 검증하는 기능은 아닙니다.
MAX_RESULT_AGE_SECONDS = 60


def model_identity(weights):
    stat = weights.stat()
    return str(weights), stat.st_mtime_ns, stat.st_size


class FodModel:
    """두 입력 모드에서 공유하며 YOLO 호출을 직렬화합니다."""

    def __init__(self, path):
        self.path = path
        self.model = None
        self.lock = Lock()

    def predict(self, source, threshold, **options):
        with self.lock:
            if self.model is None:
                from ultralytics import YOLO

                self.model = YOLO(self.path, task="detect")
                if self.model.names != {0: "FOD"}:
                    self.model = None
                    raise ValueError("FOD 단일 클래스 가중치가 필요합니다.")
            result = self.model.predict(
                source=source, conf=threshold, classes=[0], device="cpu",
                verbose=False, **options,
            )[0]
        if result.boxes is None:
            raise ValueError("객체 탐지용 YOLO 체크포인트가 필요합니다.")
        return result


@st.cache_resource(max_entries=2, show_spinner=False)
def get_model(model_key):
    return FodModel(model_key[0])


def detect(model, photo, threshold):
    """주석 표시된 BGR 이미지, 탐지 표의 행, 최고 신뢰도를 반환합니다."""
    result = model.predict(photo, threshold)

    names = result.names
    class_ids = result.boxes.cls.tolist()
    scores = result.boxes.conf.tolist()
    rows = [
        {"번호": i, "탐지 클래스": names[int(cls)], "신뢰도": f"{score:.1%}"}
        for i, (cls, score) in enumerate(zip(class_ids, scores), start=1)
    ]
    return result.plot(), rows, max(scores, default=None)


def render_fod_detection(weights, display_timezone):
    """탐지 화면을 표시하고, 유효시간 내 분석 결과가 있으면 요약 dict 반환."""
    st.subheader("착륙 패드 AI FOD 탐지")
    st.caption("사진 업로드 후 'FOD 탐지 실행'을 눌러주세요.")

    if not weights.is_file():
        st.session_state.pop("fod_output", None)
        st.session_state.pop("fod_error", None)
        st.error("YOLO 가중치 best.pt 파일을 찾지 못했습니다.")
        st.code(
            "train_model/weights/best.pt  (팀 저장소)\n"
            "vertiport_dashboard/models/best.pt  (대시보드 폴더)",
            language="text",
        )
        return None

    model_key = model_identity(weights)
    st.caption(f"사용 모델: {weights.parent.name}/{weights.name}")

    picture = st.file_uploader(
        "착륙 패드 사진 올리기", type=["jpg", "jpeg", "png"]
    )
    threshold = st.slider("최소 탐지 신뢰도", 0.05, 0.95, 0.25, 0.05)

    if picture is None:
        st.session_state.pop("fod_output", None)
        st.session_state.pop("fod_error", None)
        st.info("사진이 입력되지 않아 착륙 판단을 보류합니다.")
        return None

    payload = picture.getvalue()
    try:
        with Image.open(BytesIO(payload)) as image:
            photo = ImageOps.exif_transpose(image).convert("RGB")
    except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError):
        st.session_state.pop("fod_output", None)
        st.session_state.pop("fod_error", None)
        st.error("이미지를 열 수 없습니다. 유효한 JPG/PNG를 선택해 주세요.")
        return None

    result_key = (model_key, sha256(payload).hexdigest(), threshold)
    if st.session_state.get("fod_error", (None,))[0] != result_key:
        st.session_state.pop("fod_error", None)
    previous = st.session_state.get("fod_output")
    if previous is not None and previous[0] != result_key:
        st.session_state.pop("fod_output")
    if st.button("FOD 탐지 실행", type="primary"):
        st.session_state.pop("fod_output", None)
        st.session_state.pop("fod_error", None)
        with st.spinner("YOLO 모델 분석 중…"):
            try:
                annotated, rows, top_score = detect(get_model(model_key), photo, threshold)
                st.session_state["fod_output"] = (
                    result_key, annotated, rows, top_score,
                    datetime.now(timezone.utc),
                )
            except Exception as error:
                st.session_state["fod_error"] = (result_key, str(error))

    if "fod_error" in st.session_state:
        st.error(f"YOLO 탐지 실행 실패: {st.session_state['fod_error'][1]}")
        return None

    output = st.session_state.get("fod_output")
    if output is None or output[0] != result_key:
        st.image(photo, caption="원본 사진 · 아직 분석되지 않음", width="stretch")
        st.info("현재 사진과 신뢰도 설정으로 FOD 탐지 실행이 필요합니다.")
        return None

    _, annotated, rows, top_score, finished_at = output

    # 기존 결과가 오래되었거나 시각이 잘못되면 반드시 판단 보류.
    if not isinstance(finished_at, datetime) or finished_at.tzinfo is None:
        st.session_state.pop("fod_output", None)
        st.warning("분석 시각을 확인할 수 없습니다. FOD 탐지를 다시 실행해 주세요.")
        return None

    age_seconds = (datetime.now(timezone.utc) - finished_at).total_seconds()
    if not -5 <= age_seconds <= MAX_RESULT_AGE_SECONDS:
        st.session_state.pop("fod_output", None)
        st.warning(
            f"FOD 분석 결과가 유효하지 않습니다(유효시간: {MAX_RESULT_AGE_SECONDS}초). "
            "현재 사진으로 다시 탐지해 주세요."
        )
        st.image(photo, caption="재분석이 필요한 사진", width="stretch")
        return None

    count_col, score_col = st.columns(2)
    count_col.metric("YOLO 탐지 수", f"{len(rows)}개")
    score_col.metric(
        "최고 탐지 신뢰도",
        f"{top_score:.1%}" if top_score is not None else "—",
    )
    if rows:
        st.warning("객체가 탐지되었습니다. 탐지 박스와 클래스 이름을 확인하세요.")
    else:
        st.info("설정된 신뢰도 이상에서 탐지된 객체가 없습니다.")

    original, overlay = st.columns(2)
    original.image(photo, caption="원본 사진", width="stretch")
    overlay.image(annotated, channels="BGR", caption="YOLO 탐지 결과", width="stretch")
    if rows:
        st.dataframe(rows, hide_index=True, width="stretch")
    st.caption(
        f"분석 시각: {finished_at.astimezone(display_timezone):%Y-%m-%d %H:%M:%S} KST"
        f" · 분석 결과 유효시간: {MAX_RESULT_AGE_SECONDS}초"
    )

    # 주의: 팀 YOLO 모델이 FOD만 학습했다는 전제. 클래스가 혼합되면 필터 추가 필요.
    return {
        "fod_detected": len(rows) > 0,
        "fod_count": len(rows),
        "confidence": top_score,
    }
