#!/usr/bin/env python3
"""Plot angular velocity from CSV for the first 0~1.5s segment.

The script tries to auto-detect time/velocity columns in common CSV formats.
If no header exists, it uses the first two columns as:
column-0 -> time(s), column-1 -> angular velocity(rad/s).
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _to_float(text: str) -> float:
    return float(text.strip())


def _is_float(text: str) -> bool:
    try:
        _to_float(text)
        return True
    except (TypeError, ValueError, AttributeError):
        return False


def _pick_column(fieldnames: list[str], candidates: list[str]) -> int | None:
    normalized = [name.strip().lower() for name in fieldnames]
    for cand in candidates:
        if cand in normalized:
            return normalized.index(cand)
    return None


def load_time_velocity(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    with csv_path.open("r", newline="") as f:
        rows = list(csv.reader(f))

    if not rows:
        raise ValueError("CSV is empty")

    first_row = rows[0]
    has_header = not all(_is_float(c) for c in first_row[:2])

    times: list[float] = []
    velocities: list[float] = []

    if has_header:
        header = [c.strip() for c in first_row]

        time_idx = _pick_column(
            header,
            [
                "median_time",
                "time_since_start",
                "time_since_start_",
                "time_from_start_s",
                "time_s",
                "time_sec",
                "time",
            ],
        )
        vel_idx = _pick_column(
            header,
            [
                "mean_anglevelocity",
                "angular_velocity",
                "angular_velocity_rad_s",
                "angle_velocity",
                "velocity",
                "omega",
                "w",
            ],
        )

        if time_idx is None or vel_idx is None:
            if len(header) < 2:
                raise ValueError("CSV columns are not enough to read time and angular velocity")
            time_idx, vel_idx = 0, 1

        for row in rows[1:]:
            if max(time_idx, vel_idx) >= len(row):
                continue
            if not _is_float(row[time_idx]) or not _is_float(row[vel_idx]):
                continue
            times.append(_to_float(row[time_idx]))
            velocities.append(_to_float(row[vel_idx]))
    else:
        for row in rows:
            if len(row) < 2:
                continue
            if not _is_float(row[0]) or not _is_float(row[1]):
                continue
            times.append(_to_float(row[0]))
            velocities.append(_to_float(row[1]))

    if len(times) < 2:
        raise ValueError("Not enough valid rows for plotting")

    return np.asarray(times, dtype=float), np.asarray(velocities, dtype=float)


def extract_first_segment_0_to_1p5(
    times: np.ndarray,
    velocities: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    eps = 1e-9
    started = False
    seg_t: list[float] = []
    seg_v: list[float] = []
    prev_t: float | None = None

    for t, v in zip(times, velocities):
        if not started:
            if t < -eps or t > 1.5 + eps:
                continue
            started = True
            seg_t.append(float(t))
            seg_v.append(float(v))
            prev_t = float(t)
            continue

        if prev_t is not None and t < prev_t - eps:
            break

        if t > 1.5 + eps:
            break

        if t >= -eps:
            seg_t.append(float(t))
            seg_v.append(float(v))
            prev_t = float(t)

    if len(seg_t) < 2:
        raise ValueError("No valid first segment found in [0, 1.5]s")

    return np.asarray(seg_t, dtype=float), np.asarray(seg_v, dtype=float)


def plot_segment(times: np.ndarray, velocities: np.ndarray, output_path: Path, show: bool) -> None:
    plt.figure(figsize=(10, 5))
    plt.plot(times, velocities, "o-", lw=1.8, ms=3.5, color="#1f77b4")
    plt.title("First Segment Angular Velocity (0 ~ 1.5 s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.xlim(0.0, 1.5)
    plt.grid(alpha=0.25)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot first 0~1.5s angular velocity segment from CSV"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("buff_predictor_debug.csv"),
        help="CSV path (default: buff_predictor_debug.csv)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("log/first_segment_0_1p5.png"),
        help="Output image path (default: log/first_segment_0_1p5.png)",
    )
    parser.add_argument("--show", action="store_true", help="Show window after saving")
    args = parser.parse_args()

    all_t, all_v = load_time_velocity(args.csv)
    seg_t, seg_v = extract_first_segment_0_to_1p5(all_t, all_v)
    plot_segment(seg_t, seg_v, args.output, args.show)

    print(f"Saved figure to: {args.output}")
    print(f"All valid rows: {len(all_t)}")
    print(f"First segment rows [0,1.5]: {len(seg_t)}")
    print(f"Segment time range: [{seg_t.min():.6f}, {seg_t.max():.6f}] s")


if __name__ == "__main__":
    main()
