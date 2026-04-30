#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
usage:
 python3 calibrate.py capture_FGV22100004 \
  --sizes 6-14,6-14 --sample 5 --min-hits 2 \
  --detect-scale 0.5 --lite-pre --jobs 8 --sb-first --log debug

"""

import argparse, glob, os, time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np

# ---------------------- Logging ----------------------
LEVELS = {"silent": 0, "info": 1, "debug": 2, "trace": 3}
def make_logger(level: str):
    lvl = LEVELS.get(level, 1)
    def log(l, *a, **k):
        if LEVELS[l] <= lvl:
            print(*a, **k)
    return log

# ---------------------- IO ----------------------
def collect_images(inputs: List[str], exts: List[str]) -> List[str]:
    files: List[str] = []
    norm_exts = tuple("." + e.lower().lstrip(".") for e in exts)
    for inp in inputs:
        p = Path(inp)
        if p.is_dir():
            for ext in norm_exts:
                files += glob.glob(str(p / f"*{ext}"))
        else:
            files += glob.glob(inp)
    files = sorted({f for f in files if Path(f).suffix.lower() in norm_exts})
    return files

# ---------------------- Vision helpers ----------------------
def preprocess_gray_fast(gray: np.ndarray, lite: bool) -> np.ndarray:
    if lite:
        return cv2.GaussianBlur(gray, (3, 3), 0)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    return cv2.GaussianBlur(clahe.apply(gray), (3, 3), 0)

def detect_on_scaled(gray: np.ndarray, size: Tuple[int,int], sb_first: bool,
                     scale: float, for_hit_count: bool = True) -> Tuple[bool, Optional[np.ndarray], str]:
    """缩放后检测；命中则把角点放大回原尺度；for_hit_count=True 时不做亚像素"""
    small = gray if scale == 1.0 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    # 先用 SB（若可用且开启）
    if sb_first and hasattr(cv2, "findChessboardCornersSB"):
        ret, corners = cv2.findChessboardCornersSB(small, size, None)
        method = "SB"
    else:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(small, size, flags)
        method = "classic"

    if not ret:
        return False, None, method

    corners = corners.astype(np.float32)
    if scale != 1.0:
        corners /= scale
    return True, corners, method

def fmt_list(nums, nd=6) -> str:
    return ", ".join(f"{float(x):.{nd}f}" for x in nums)

# ---------------------- Main ----------------------
def main():
    ap = argparse.ArgumentParser(description="快速棋盘标定（多尺寸试探 + 小图检测 + 并行）")
    ap.add_argument("inputs", nargs="+", help="图片目录或通配符，可多个")
    ap.add_argument("--ext", nargs="+", default=["png", "jpg", "jpeg", "bmp"])
    ap.add_argument("--sizes", type=str, default="6-14,6-14",
                    help="尺寸范围：cols_min-cols_max,rows_min-rows_max（默认 6-14,6-14）")
    ap.add_argument("--prefer", type=str, default="", help="优先尺寸，如 '11x8'")
    ap.add_argument("--known-size", dest="known_size", type=str, default="", help="已知内角点尺寸，如 11x8（跳过扫描）")
    ap.add_argument("--sample", type=int, default=5, help="扫描阶段抽样图片数")
    ap.add_argument("--min-hits", type=int, default=2, help="某尺寸命中达到 N 即提前选用")
    ap.add_argument("--detect-scale", type=float, default=0.5, help="检测缩放系数（0.4~0.7 常用）")
    ap.add_argument("--jobs", type=int, default=8, help="并行线程数（选定尺寸后使用）")
    ap.add_argument("--vis", type=str, default="", help="保存角点可视化到该目录")
    ap.add_argument("--dump-pre", type=str, default="", help="保存预处理图到该目录（gray/clahe/blur）")
    ap.add_argument("--out", type=str, default="", help="保存 YAML 到文件")
    ap.add_argument("--log", type=str, default="info", choices=list(LEVELS.keys()), help="日志等级：silent|info|debug|trace")
    ap.add_argument("--sb-first", action="store_true", help="优先使用 findChessboardCornersSB")
    ap.add_argument("--no-sb", action="store_true", help="禁用 SB，仅用经典法")
    ap.add_argument("--lite-pre", action="store_true", help="扫描阶段轻预处理（只高斯）")
    args = ap.parse_args()

    if args.no_sb:
        args.sb_first = False
    log = make_logger(args.log)

    # 读图
    images = collect_images(args.inputs, args.ext)
    if not images:
        raise SystemExit("未找到图片。")
    vis_dir: Optional[Path] = Path(args.vis) if args.vis else None
    dump_dir: Optional[Path] = Path(args.dump_pre) if args.dump_pre else None
    if vis_dir: vis_dir.mkdir(parents=True, exist_ok=True)
    if dump_dir: dump_dir.mkdir(parents=True, exist_ok=True)

    # 已知尺寸 → 直接用
    if args.known_size:
        cols, rows = [int(x) for x in args.known_size.lower().split("x")]
        log("info", f"[SELECT] 使用已知尺寸: {cols}x{rows}")
    else:
        # 扫描阶段：仅对前 sample 张做预处理与试探
        sample_imgs = images[:max(1, args.sample)]
        pre_cache: Dict[str, np.ndarray] = {}
        for f in sample_imgs:
            img = cv2.imread(f, cv2.IMREAD_COLOR)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            pre_cache[f] = preprocess_gray_fast(gray, lite=args.lite_pre)

        c_rng, r_rng = args.sizes.split(",")
        c0, c1 = [int(x) for x in c_rng.split("-")]
        r0, r1 = [int(x) for x in r_rng.split("-")]
        prefer_list: List[Tuple[int,int]] = []
        if args.prefer:
            a, b = args.prefer.lower().split("x")
            prefer_list.append((int(a), int(b)))

        scan = prefer_list + [(c, r) for c in range(c1, c0-1, -1) for r in range(r1, r0-1, -1)]
        seen = set()
        sizes_to_try = [s for s in scan if (s not in seen and not seen.add(s))]
        log("info", f"[SCAN] sample={len(sample_imgs)} sizes={len(sizes_to_try)} scale={args.detect_scale}")

        ok_sizes: Dict[Tuple[int,int], int] = {}
        for cols_i, rows_i in sizes_to_try:
            hits = 0
            for f in sample_imgs:
                g = pre_cache.get(f)
                if g is None:
                    continue
                ret, _, _ = detect_on_scaled(g, (cols_i, rows_i), args.sb_first, args.detect_scale, True)
                if ret:
                    hits += 1
                    if hits >= args.min_hits:
                        break
            if hits > 0:
                ok_sizes[(cols_i, rows_i)] = hits
                log("debug", f"  size {cols_i}x{rows_i} -> hits={hits}")
                if (cols_i, rows_i) in prefer_list and hits >= args.min_hits:
                    log("info", f"[STOP] prefer {cols_i}x{rows_i} 达到 min_hits={args.min_hits}，提前结束扫描")
                    break

        if not ok_sizes:
            raise SystemExit("扫描未命中任何尺寸；请确认“内角点数”（不是方块数）、对比度与清晰度。")
        cols, rows = max(ok_sizes.items(), key=lambda kv: kv[1])[0]
        log("info", f"[SELECT] 使用命中最多的尺寸: {cols}x{rows}")

    # 选定尺寸后：并行处理全部图片
    objpoints, imgpoints = [], []
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)

    def process_one(f: str):
        img = cv2.imread(f, cv2.IMREAD_COLOR)
        if img is None:
            return None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        g = preprocess_gray_fast(gray, lite=False)  # 全量预处理更稳
        ret, corners, method = detect_on_scaled(g, (cols, rows), args.sb_first, args.detect_scale, True)
        if not ret:
            return None
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        return (f, img, gray.shape[::-1], corners, method)

    used = 0
    image_size: Optional[Tuple[int,int]] = None
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as ex:
        futures = {ex.submit(process_one, f): f for f in images}
        for fu in as_completed(futures):
            res = fu.result()
            if not res:
                continue
            f, img, sz, corners, method = res
            image_size = sz
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            used += 1
            if vis_dir:
                vis = cv2.drawChessboardCorners(img.copy(), (cols, rows), corners, True)
                outp = vis_dir / (Path(f).stem + f"_corners_{cols}x{rows}.png")
                cv2.imwrite(str(outp), vis)
            log("info", f"[USE] {Path(f).name}: OK ({method}); total_used={used}")

    if used < 3:
        raise SystemExit(f"有效标定图过少（{used}）。建议至少 8~12 张不同姿态。")

    t0 = time.perf_counter()
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)
    t1 = time.perf_counter()
    log("info", f"[CALIB] ret={ret} time={(t1 - t0) * 1e3:.1f} ms")

    # 重投影误差
    total_err, total_pts = 0.0, 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        total_err += err * err
        total_pts += len(imgpoints[i])
    rmse = (total_err / total_pts) ** 0.5 if total_pts > 0 else float("nan")
    log("info", f"[RMSE] {rmse:.4f}px over {total_pts} points")

    # YAML 输出
    k_list = [K[0,0], K[0,1], K[0,2], K[1,0], K[1,1], K[1,2], K[2,0], K[2,1], K[2,2]]
    d_list = dist.reshape(-1).tolist()
    yaml_text = (
        "calibration:\n"
        f"  k: [{fmt_list(k_list, 4)}]\n"
        f"  d: [{fmt_list(d_list, 6)}]\n"
        '  model: "plumb_bob"\n'
        "  compute_rectified_p: true\n"
        f"# inner_corners: [{cols}, {rows}]\n"
        f"# image_size: [{image_size[0]}, {image_size[1]}]\n"
        f"# reprojection_rmse: {rmse:.4f}\n"
        f"# used_images: {used} / {len(images)}\n"
    )
    print("\n===== YAML =====\n" + yaml_text)

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(yaml_text)
        log("info", f"[OK] 保存到 {args.out}")

if __name__ == "__main__":
    main()
