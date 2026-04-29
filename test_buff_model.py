#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np
from openvino import Core


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = REPO_ROOT / "assets" / "yolo11_buff_int8.xml"
DEFAULT_IMAGE = Path(
    "/Users/calebevans/Desktop/render_video/646522A6-426C-44DB-AF4A-B4926089F737_1_106_c.jpeg"
)
KEYPOINT_COUNT = 6
INPUT_SIZE = 640


def port_name(port) -> str:
    try:
        return port.any_name
    except RuntimeError:
        return "<unnamed>"


def normalize_path(path_text: str) -> Path:
    path = Path(path_text)
    if path.exists():
        return path
    users_marker = "/Users/"
    marker_index = path_text.rfind(users_marker)
    if marker_index > 0:
        users_path = Path(path_text[marker_index:])
        if users_path.exists():
            return users_path
    if path_text.startswith("/Users/"):
        linux_path = Path("/home") / Path(path_text).relative_to("/Users")
        if linux_path.exists():
            return linux_path
    return path


def fill_tensor(bgr_image: np.ndarray) -> tuple[np.ndarray, float]:
    scale = min(INPUT_SIZE / bgr_image.shape[0], INPUT_SIZE / bgr_image.shape[1])
    matrix = np.array([[scale, 0.0, 0.0], [0.0, scale, 0.0]], dtype=np.float32)
    blob_image = cv2.warpAffine(bgr_image, matrix, (INPUT_SIZE, INPUT_SIZE))
    blob_image = cv2.cvtColor(blob_image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = np.transpose(blob_image, (2, 0, 1))[np.newaxis, ...]
    return tensor, 1.0 / scale


def get_r_center(keypoints: np.ndarray, bgr_image: np.ndarray) -> tuple[float, float]:
    rough_center = (keypoints[5] - keypoints[4]) * 1.4 + keypoints[4]

    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary = cv2.dilate(binary, kernel)

    radius = np.linalg.norm(keypoints[2] - keypoints[4]) * 0.8
    mask = np.zeros(binary.shape, dtype=np.uint8)
    cv2.circle(mask, tuple(np.round(rough_center).astype(int)), int(radius), 255, -1)
    binary = cv2.bitwise_and(binary, mask)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    r_center = rough_center
    best_score = 1e9
    for contour in contours:
        if len(contour) < 5:
            continue
        rect = cv2.minAreaRect(contour)
        width = max(rect[1][0], 1.0)
        height = max(rect[1][1], 1.0)
        ratio = max(width / height, height / width)
        score = ratio + np.linalg.norm(np.array(rect[0], dtype=np.float32) - rough_center) / max(radius / 3.0, 1.0)
        if score < best_score:
            best_score = score
            r_center = np.array(rect[0], dtype=np.float32)
    return float(r_center[0]), float(r_center[1])


def run_model(model_path: Path, image_path: Path, threshold: float, device: str) -> dict | None:
    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"could not read image: {image_path}")

    core = Core()
    model = core.read_model(str(model_path))
    compiled_model = core.compile_model(model, device)
    input_port = compiled_model.input(0)
    output_port = compiled_model.output(0)

    tensor, factor = fill_tensor(image)
    result = compiled_model([tensor])[output_port]
    output = np.asarray(result, dtype=np.float32)

    if output.ndim == 3:
        det_output = output[0]
    elif output.ndim == 2:
        det_output = output
    else:
        raise RuntimeError(f"unsupported output shape: {output.shape}")

    if det_output.shape[0] < det_output.shape[1]:
        candidates = det_output
    else:
        candidates = det_output.T

    best_index = int(np.argmax(candidates[4]))
    confidence = float(candidates[4, best_index])
    print(f"input:  {port_name(input_port)} {tuple(input_port.shape)}")
    print(f"output: {port_name(output_port)} {tuple(output.shape)}")
    print(f"best:   index={best_index} confidence={confidence:.4f}")

    if confidence < threshold:
        return None

    keypoint_values = candidates[5 : 5 + KEYPOINT_COUNT * 2, best_index]
    keypoints = keypoint_values.reshape(KEYPOINT_COUNT, 2) * factor
    r_center = get_r_center(keypoints, image)
    return {
        "image": image,
        "confidence": confidence,
        "keypoints": keypoints,
        "r_center": r_center,
    }


def draw_detection(result: dict, output_path: Path) -> None:
    image = result["image"].copy()
    keypoints = result["keypoints"]
    colors = [
        (0, 255, 255),
        (0, 180, 255),
        (0, 255, 0),
        (255, 180, 0),
        (255, 0, 255),
        (255, 0, 0),
    ]
    for idx, point in enumerate(keypoints):
        center = tuple(np.round(point).astype(int))
        cv2.circle(image, center, 5, colors[idx % len(colors)], -1)
        cv2.putText(image, str(idx), (center[0] + 7, center[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[idx], 2)

    r_center = tuple(np.round(result["r_center"]).astype(int))
    cv2.drawMarker(image, r_center, (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(
        image,
        f"conf={result['confidence']:.3f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 255),
        2,
    )
    cv2.imwrite(str(output_path), image)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run yolo11 buff OpenVINO model on one image.")
    parser.add_argument("image", nargs="?", default=str(DEFAULT_IMAGE), help="test image path")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="OpenVINO .xml model path")
    parser.add_argument("--threshold", type=float, default=0.7, help="confidence threshold")
    parser.add_argument("--device", default="CPU", help="OpenVINO device")
    parser.add_argument("--output", default="", help="output visualization path")
    args = parser.parse_args()

    image_path = normalize_path(args.image)
    model_path = normalize_path(args.model)
    output_path = Path(args.output) if args.output else Path.cwd() / f"{image_path.stem}_buff_result.jpg"

    result = run_model(model_path, image_path, args.threshold, args.device)
    if result is None:
        print(f"not detected above threshold={args.threshold}")
        return

    print("keypoints:")
    for idx, point in enumerate(result["keypoints"]):
        print(f"  {idx}: x={point[0]:.1f}, y={point[1]:.1f}")
    print(f"r_center: x={result['r_center'][0]:.1f}, y={result['r_center'][1]:.1f}")
    draw_detection(result, output_path)
    print(f"saved: {output_path}")


if __name__ == "__main__":
    main()
