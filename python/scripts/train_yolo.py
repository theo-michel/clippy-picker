#!/usr/bin/env python3
"""Train a YOLOv8n model on the labelled Raspberry Pi Case dataset.

Automatically creates a train/val split (80/20) from the exported
Roboflow dataset if no ``valid/`` folder exists yet.
"""

from __future__ import annotations

import random
import shutil
from pathlib import Path

import yaml
from ultralytics import YOLO

DATASET_DIR = Path(__file__).resolve().parent.parent / "dataset"
SEED = 42
VAL_RATIO = 0.2


def ensure_val_split() -> None:
    """Create a validation split from the training set if one doesn't exist."""
    val_img = DATASET_DIR / "valid" / "images"
    if val_img.exists() and any(val_img.iterdir()):
        return

    train_img = DATASET_DIR / "train" / "images"
    train_lbl = DATASET_DIR / "train" / "labels"
    val_lbl = DATASET_DIR / "valid" / "labels"

    val_img.mkdir(parents=True, exist_ok=True)
    val_lbl.mkdir(parents=True, exist_ok=True)

    images = sorted(train_img.glob("*.*"))
    random.seed(SEED)
    random.shuffle(images)

    n_val = max(1, int(len(images) * VAL_RATIO))
    for img_path in images[:n_val]:
        lbl_path = train_lbl / (img_path.stem + ".txt")
        shutil.copy2(str(img_path), str(val_img / img_path.name))
        if lbl_path.exists():
            shutil.copy2(str(lbl_path), str(val_lbl / lbl_path.name))

    print(f"Split: {len(images) - n_val} train / {n_val} val")


def main() -> None:
    ensure_val_split()

    data_yaml = DATASET_DIR / "data.yaml"
    cfg = yaml.safe_load(data_yaml.read_text())
    cfg["train"] = str(DATASET_DIR / "train" / "images")
    cfg["val"] = str(DATASET_DIR / "valid" / "images")
    cfg.pop("test", None)

    patched = DATASET_DIR / "data_train.yaml"
    patched.write_text(yaml.dump(cfg, default_flow_style=False))

    model = YOLO("yolov8n.pt")
    model.train(
        data=str(patched),
        epochs=100,
        imgsz=640,
        batch=-1,  # auto batch size
        patience=20,
        project=str(DATASET_DIR.parent / "runs"),
        name="rpi_case",
        exist_ok=True,
    )
    print(f"\nBest weights: {DATASET_DIR.parent / 'runs' / 'rpi_case' / 'weights' / 'best.pt'}")


if __name__ == "__main__":
    main()
