"""Run the team's YOLO detector on a landing pad image."""

from datetime import datetime, timezone
from hashlib import sha256
from io import BytesIO

import streamlit as st
from PIL import Image, ImageOps, UnidentifiedImageError


def detect(model, photo, threshold):
    """Return an annotated BGR frame and one row per detected box."""
    result = model.predict(source=photo, conf=threshold, verbose=False)[0]
    if result.boxes is None:
        raise ValueError("객체 탐지용 YOLO 체크포인트가 필요합니다.")

    names = result.names
    class_ids = result.boxes.cls.tolist()
    scores = result.boxes.conf.tolist()
    rows = [
        {"번호": index, "탐지 클래스": names[int(class_id)], "신뢰도": f"{score:.1%}"}
        for index, (class_id, score) in enumerate(zip(class_ids, scores), start=1)
    ]
    return result.plot(), rows, max(scores, default=None)


def render_fod_detection(weights, display_timezone):
    st.subheader("착륙 패드 AI 탐지")
    st.caption("사진 전체에서 학습된 FOD 클래스를 탐지합니다. 패드가 보이는 사진을 사용하세요.")

    if not weights.is_file():
        st.warning("best.pt를 찾지 못했습니다. 팀 저장소 안에 대시보드 폴더를 넣었는지 확인하세요.")
        st.code("../train_model/weights/best.pt  (팀 저장소)\nmodels/best.pt  (독립 실행)", language="text")
        return

    stat = weights.stat()
    model_key = (str(weights), stat.st_mtime_ns, stat.st_size)
    st.caption(f"사용 모델 · {weights.parent.name}/{weights.name}")
    source = st.radio("이미지 입력", ["사진 업로드", "웹캠 촬영"], horizontal=True)
    if source == "웹캠 촬영":
        picture = st.camera_input("착륙 패드 사진 찍기")
    else:
        picture = st.file_uploader("착륙 패드 사진 올리기", type=["jpg", "jpeg", "png"])
    threshold = st.slider("최소 탐지 신뢰도", 0.05, 0.95, 0.25, 0.05,
                          help="이 점수 이상의 탐지 결과를 표시합니다.")
    if picture is None:
        st.session_state.pop("fod_output", None)
        return

    payload = picture.getvalue()
    try:
        with Image.open(BytesIO(payload)) as image:
            photo = ImageOps.exif_transpose(image).convert("RGB")
    except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError):
        st.error("사진을 열 수 없습니다. 정상적인 JPG 또는 PNG를 선택하세요.")
        return

    result_key = (model_key, sha256(payload).hexdigest(), threshold)
    if st.button("FOD 탐지 실행", type="primary"):
        st.session_state.pop("fod_output", None)
        with st.spinner("학습 모델로 FOD를 탐지하는 중…"):
            try:
                cached = st.session_state.get("fod_model")
                if cached is None or cached[0] != model_key:
                    from ultralytics import YOLO

                    model = YOLO(str(weights), task="detect")
                    st.session_state["fod_model"] = (model_key, model)
                else:
                    model = cached[1]
                annotated, rows, top_score = detect(model, photo, threshold)
                st.session_state["fod_output"] = (result_key, annotated, rows, top_score,
                                                  datetime.now(timezone.utc))
            except ModuleNotFoundError as error:
                st.error(f"필요한 패키지를 불러오지 못했습니다. requirements.txt를 설치하세요. ({error})")
            except Exception as error:
                st.error(f"FOD 탐지를 실행하지 못했습니다. 학습에 사용한 YOLO 버전과 best.pt를 확인하세요. ({error})")

    output = st.session_state.get("fod_output")
    if output is None or output[0] != result_key:
        st.image(photo, caption="원본 사진", width="stretch")
        return

    _, annotated, rows, top_score, finished_at = output
    count, confidence, decision = st.columns(3)
    count.metric("FOD 탐지 수", f"{len(rows)}개")
    confidence.metric("최고 탐지 신뢰도", f"{top_score:.1%}" if top_score is not None else "—")
    decision.metric("착륙 판단", "확인 필요")
    if rows:
        st.warning("FOD가 감지됐습니다. 탐지 박스를 확인하세요.")
    else:
        st.info("설정한 신뢰도 기준에서 탐지된 FOD가 없습니다.")
    st.caption(f"추론 완료 {finished_at.astimezone(display_timezone):%H:%M:%S} KST · 착륙 판단 및 기체 정보는 JSON 파일 연동 모드에서 확인합니다.")
    original, overlay = st.columns(2)
    original.image(photo, caption="원본 사진", width="stretch")
    overlay.image(annotated, channels="BGR", caption="YOLO 탐지 결과", width="stretch")
    if rows:
        st.dataframe(rows, hide_index=True, width="stretch")
