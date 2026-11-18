# 스마트 버티포트 AI 대시보드

학습된 Ultralytics YOLO 모델로 FOD를 탐지하고, 별도의 연동 모드에서 착륙 판단과 기체 정보를 표시합니다.
Python 3.11 또는 3.12를 권장합니다.

## Windows / VS Code 실행

1. ZIP을 풀고 `vertiport_dashboard` 폴더를 팀 저장소 `Daeun-sWorld` 안에 넣으세요. `train_model` 폴더와 나란히 놓습니다.
2. `Daeun-sWorld`를 VS Code에서 열고 터미널 → 새 터미널을 누르세요.
3. 아래 명령을 한 줄씩 실행하세요. 앱이 `train_model/weights/best.pt`를 자동으로 사용합니다.

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\vertiport_dashboard\requirements.txt
.\.venv\Scripts\python.exe -m streamlit run .\vertiport_dashboard\app.py
```

브라우저에서 http://localhost:8501 을 여세요. 종료는 터미널에서 Ctrl+C입니다.
`py`가 없으면 Python 설치를 확인하거나 첫 명령의 `py`를 `python`으로 바꾸세요.
가상환경 활성화 명령은 필요하지 않습니다.
팀 저장소 밖에서 독립 실행하려면 `vertiport_dashboard/models/best.pt`에 가중치를 두고, 대시보드 폴더에서 명령의 `vertiport_dashboard` 경로 부분을 빼고 실행하세요.

## 학습 모델 연결 확인

1. **AI FOD 탐지**를 선택합니다.
2. 학습에 사용하지 않은 착륙 패드 사진을 업로드하거나 웹캠으로 한 장 촬영합니다.
3. **FOD 탐지 실행**을 누르면 원본과 박스가 표시된 결과, 탐지 개수, 클래스, 신뢰도가 표시됩니다.
4. FOD가 있는 사진과 빈 패드 사진으로 각각 확인하세요. 신뢰도 0.25는 화면의 초기값이며, 최종값은 검증 결과에 맞춰 조정하세요.

AI 모드는 사진 단위로 동작합니다. 사진 전체에서 모델이 학습한 모든 클래스를 집계하므로 FOD 전용 모델을 사용하세요.
`requirements.txt`의 Ultralytics 8.4.135는 `rnarnarna`의 `best.pt`에 기록된 버전입니다.
AI 모드에서 착륙 판단은 확인 필요로 표시합니다. 실시간 영상과 착륙 판단 파이프라인의 연결은 다음 단계입니다.

## 시연 순서

- 정상 상태: 샘플 착륙 가능 결과와 DRONE-01 정보 확인
- FOD 발견: FOD 2개, 샘플 착륙 불가 결과 확인
- 복수 기체 요청: DRONE-02 → DRONE-01의 시연용 순위 확인
- 수신 중단: 최신 판단 대신 '판단 확인 필요' 표시
- 선택 사항: 착륙장 사진을 업로드해 모식도를 대체

좌표, 신뢰도, 배터리, 우선순위 및 판단 결과는 모두 샘플입니다.
시뮬레이션 모드의 사진 업로드는 화면 표시용입니다. 사진에서 탐지를 실행하려면 AI FOD 탐지 모드를 사용하세요.

## 이후 팀원 프로그램 연결

사이드바에서 'JSON 파일 연동'을 선택하면 `data/live_state.json`을 2초마다 다시 읽습니다.
`data/sample_state.json`은 형식을 보여주는 예시이며 자동 갱신되지 않습니다.
팀원의 프로그램이 실제 탐지·판단·기체 데이터를 모아 아래 형식으로 계속 저장해야 합니다.
판단 알고리즘은 팀원 프로그램이 담당하고, 이 화면은 전달된 결과를 표시합니다.

```json
{
  "updated_at": "2026-10-29T12:00:00+00:00",
  "fod_count": 0,
  "confidence": null,
  "decision": "ALLOWED",
  "reason": "판단 프로그램이 전달한 사유",
  "aircraft": [{
    "id": "DRONE-01", "battery_pct": 68, "altitude_m": 12.5,
    "latitude": 37.0001, "longitude": 127.0001,
    "landing_requested": true
  }],
  "queue": [{"rank": 1, "id": "DRONE-01", "reason": "판단 프로그램이 전달한 사유"}]
}
```

- decision: ALLOWED(가능), WAIT(대기), BLOCKED(불가), UNKNOWN(미확인)
- updated_at: 화면을 읽은 시간이 아닌, 전체 상태를 생성한 시각. 반드시 시간대를 포함하세요.
- 각 원천 데이터가 최신인지 확인한 후 통합 상태를 생성하세요. 오래된 센서값에 새 시각만 붙이지 마세요.
- 데이터가 10초 이상 오래되거나 5초 넘게 미래인 경우 최신 착륙 판단을 표시하지 않습니다.
- 배터리 단위는 %, 고도는 m입니다. 고도 기준(상대고도 등)은 팀원과 합의하세요.
- 미수신 기체 수치는 0 대신 null을 사용하세요. 실제 위치가 없으면 null을 쓰세요.
- confidence는 0~1 또는 null입니다. FOD 미탐지를 안전 확률 100%로 해석하지 마세요.
- queue는 외부 알고리즘의 결과입니다. 대시보드에서 우선순위를 계산하지 않습니다.
- 쓰기 도중 읽는 것을 막기 위해 임시 JSON을 저장한 뒤 `os.replace()`로 교체하세요.
- 파일 부재·손상 시 수신 불가를 표시합니다. 기체 및 순위 정보는 수신 지연 시 마지막 값입니다.

현재는 통합된 상태의 시각만 검사합니다. 실제 연동에서는 원천별 갱신 시각·연결 상태를 추가하는 것이 다음 단계입니다.

공식 API 참고: https://docs.streamlit.io/develop/api-reference/execution-flow/st.fragment
