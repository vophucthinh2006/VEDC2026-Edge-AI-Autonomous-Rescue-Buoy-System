#!/usr/bin/env python3
"""Export weights/yolov8n_320.onnx to the TFLite model the Pi runs, weights/yolov8n_320_drq.tflite.

Why not the int8 weights/yolov8n_int8.tflite: YOLOv8 concatenates the boxes (pixels, 0 to 320)
and the class scores (0 to 1) into one output tensor. Quantized to int8 with one scale for the
whole tensor, the scores keep almost no resolution: that model's person score saturates at 0.50 to
0.61 and moves in steps of 0.04 to 0.16, where the float model gives up to 0.86.

What this does:
  1. divides the box branch by 320 in the ONNX graph, so the whole output lies in 0..1
     (the Pi's edge_model.py reads boxes either way);
  2. converts with onnx2tf, calibrating on frames of the team's videos (and, if given, of the
     simulator), into several TFLite variants;
  3. keeps the dynamic-range one: int8 weights, float activations. Scores within 0.01 of the
     float model on average, 3.3 MB, about 1.3 times the time of the full int8 model.

    pip install "tensorflow==2.19.0" "onnx2tf==1.27.10" onnx onnxruntime onnx_graphsurgeon sng4onnx \\
        psutil "tf_keras==2.19.0" ai-edge-litert onnxsim opencv-python-headless "numpy<2.1"
    python3 tools/export_tflite.py                         # from YOLO_Movenet/
    python3 tools/export_tflite.py --frames shots/*.jpg    # add simulator frames to the calibration
"""
from __future__ import annotations

import argparse
import glob
import shutil
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import onnx
from onnx import helper, numpy_helper

HERE = Path(__file__).resolve().parents[1]
SIZE = 320


def letterbox(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(SIZE / height, SIZE / width)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image, (new_w, new_h))
    top, left = (SIZE - new_h) // 2, (SIZE - new_w) // 2
    return cv2.copyMakeBorder(resized, top, SIZE - new_h - top, left, SIZE - new_w - left, cv2.BORDER_CONSTANT,
                              value=(114, 114, 114))


def calibration_frames(videos: list[str], frames: list[str], per_video: int) -> np.ndarray:
    images = []
    for path in videos:
        capture = cv2.VideoCapture(path)
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        for index in np.linspace(0, max(count - 1, 0), per_video).astype(int):
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
            ok, frame = capture.read()
            if ok:
                images.append(frame)
        capture.release()
    images += [image for image in (cv2.imread(path) for path in frames) if image is not None]
    if not images:
        sys.exit("no calibration frames")
    return np.stack([cv2.cvtColor(letterbox(image), cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0 for image in images])


def normalise_boxes(source: Path, target: Path) -> None:
    """Insert box / 320 before the output Concat of the YOLOv8 head."""
    model = onnx.load(str(source))
    graph = model.graph
    output = graph.output[0].name
    concat = next(node for node in graph.node if output in node.output)
    if concat.op_type != "Concat":
        sys.exit(f"unexpected output node {concat.op_type}: not a YOLOv8 head")
    graph.initializer.append(numpy_helper.from_array(np.array(float(SIZE), np.float32), "box_scale"))
    graph.node.insert(list(graph.node).index(concat),
                      helper.make_node("Div", [concat.input[0], "box_scale"], ["box_normalised"], name="BoxNormalise"))
    concat.input[0] = "box_normalised"
    onnx.checker.check_model(model)
    onnx.save(model, str(target))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--onnx", type=Path, default=HERE / "weights" / "yolov8n_320.onnx")
    parser.add_argument("--out", type=Path, default=HERE / "weights" / "yolov8n_320_drq.tflite")
    parser.add_argument("--videos", nargs="*", default=sorted(glob.glob(str(HERE / "videos" / "rescue_test*.mp4"))))
    parser.add_argument("--frames", nargs="*", default=[], help="more calibration pictures, e.g. simulator frames")
    parser.add_argument("--per-video", type=int, default=50)
    args = parser.parse_args()

    import onnx2tf.onnx2tf as converter
    import onnx2tf.utils.common_functions as common

    work = Path(tempfile.mkdtemp(prefix="yolo_export_"))
    calib = calibration_frames(args.videos, args.frames, args.per_video)
    np.save(work / "calib.npy", calib)
    print(f"calibration: {len(calib)} frames")
    normalise_boxes(args.onnx, work / "model.onnx")
    # onnx2tf downloads sample pictures for its own checks; use the calibration frames instead
    # (and work offline).
    converter.download_test_image_data = common.download_test_image_data = lambda: calib[:20]
    sys.argv = ["onnx2tf", "-i", str(work / "model.onnx"), "-o", str(work / "tf"), "-oiqt", "-qt", "per-channel",
                "-cind", "images", str(work / "calib.npy"), "[[[[0.0,0.0,0.0]]]]", "[[[[1.0,1.0,1.0]]]]", "-n"]
    converter.main()
    shutil.copy(work / "tf" / "model_dynamic_range_quant.tflite", args.out)
    print(f"wrote {args.out} (other variants in {work / 'tf'})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
