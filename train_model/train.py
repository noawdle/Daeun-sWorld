import sys
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "FOD.v1i.yolov11" / "data.yaml"

if __name__ == "__main__":
    # 메모리 문제 재발 시: python train.py 0  → workers=0
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 2

    model = YOLO(str(ROOT / "yolo11n.pt"))
    model.train(
        data=str(DATA),
        epochs=100,
        imgsz=640,
        batch=8,
        device=0,
        workers=workers,
        project=str(ROOT / "runs" / "detect"),
        name="fod_yolo11n",
        verbose=True,
    )
