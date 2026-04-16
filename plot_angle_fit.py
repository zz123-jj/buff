#!/usr/bin/env python3
"""Plot angular velocity from CSV for all 0~1.5s segments.

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
from scipy.optimize import curve_fit


FIT_OFFSET_SUM = 2.090
LOWER_BOUNDS = (0.780, 1.884, -np.pi)
UPPER_BOUNDS = (1.045, 2.000, np.pi)


def sine_func(t: np.ndarray, A: float, omega: float, phi: float) -> np.ndarray:
    return A * np.sin(omega * t + phi) + (FIT_OFFSET_SUM - A)


def fit_segment(times: np.ndarray, velocities: np.ndarray) -> tuple[float, float, float] | None:
    if len(times) < 4:
        return None

    A_guess = (LOWER_BOUNDS[0] + UPPER_BOUNDS[0]) / 2.0
    omega_guess = (LOWER_BOUNDS[1] + UPPER_BOUNDS[1]) / 2.0
    phi_guess = 0.0
    p0 = [A_guess, omega_guess, phi_guess]

    try:
        popt, _ = curve_fit(
            sine_func,
            times,
            velocities,
            p0=p0,
            bounds=(LOWER_BOUNDS, UPPER_BOUNDS),
        )
        return float(popt[0]), float(popt[1]), float(popt[2])
    except RuntimeError:
        return None


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


def extract_all_segments_0_to_1p5(
    times: np.ndarray,
    velocities: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    eps = 1e-9
    segments: list[tuple[np.ndarray, np.ndarray]] = []
    cur_t: list[float] = []
    cur_v: list[float] = []
    prev_t: float | None = None

    for t, v in zip(times, velocities):
        # Detect a new segment when time decreases.
        if prev_t is not None and t < prev_t - eps:
            if len(cur_t) >= 2:
                segments.append(
                    (np.asarray(cur_t, dtype=float), np.asarray(cur_v, dtype=float))
                )
            cur_t = []
            cur_v = []

        if -eps <= t <= 1.5 + eps:
            cur_t.append(float(t))
            cur_v.append(float(v))

        prev_t = float(t)

    if len(cur_t) >= 2:
        segments.append((np.asarray(cur_t, dtype=float), np.asarray(cur_v, dtype=float)))

    if not segments:
        raise ValueError("No valid segment found in [0, 1.5]s")

    return segments


def plot_segments(
    segments: list[tuple[np.ndarray, np.ndarray]],
    output_path: Path,
    show: bool,
) -> list[tuple[int, tuple[float, float, float] | None]]:
    plt.figure(figsize=(10, 5))
    cmap = plt.get_cmap("tab20")
    fit_results: list[tuple[int, tuple[float, float, float] | None]] = []

    for idx, (times, velocities) in enumerate(segments, start=1):
        color = cmap((idx - 1) % 20)
        plt.scatter(
            times,
            velocities,
            s=12,
            color=color,
            alpha=0.9,
            label=f"segment {idx} raw",
        )

        fit_params = fit_segment(times, velocities)
        fit_results.append((idx, fit_params))
        if fit_params is None:
            continue

        A_fit, omega_fit, phi_fit = fit_params
        b_fit = FIT_OFFSET_SUM - A_fit
        t_fit = np.linspace(float(times.min()), max(float(times.max()) * 2.0, 1.5), 200)
        y_fit = sine_func(t_fit, A_fit, omega_fit, phi_fit)

        plt.plot(
            t_fit,
            y_fit,
            "-",
            lw=1.8,
            color=color,
            alpha=0.85,
            label=(
                f"segment {idx} fit: "
                f"A={A_fit:.3f}, w={omega_fit:.3f}, phi={phi_fit:.3f}, b={b_fit:.3f}"
            ),
        )

    plt.title("All Segments Angular Velocity (0 ~ 1.5 s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.xlim(0.0, 1.5)
    plt.grid(alpha=0.25)
    if len(segments) <= 12:
        plt.legend(loc="best")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    plt.close()
    return fit_results


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot all 0~1.5s angular velocity segments from CSV"
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
        default=Path("log/all_segments_0_1p5.png"),
        help="Output image path (default: log/all_segments_0_1p5.png)",
    )
    parser.add_argument("--show", action="store_true", help="Show window after saving")
    args = parser.parse_args()

    all_t, all_v = load_time_velocity(args.csv)
    segments = extract_all_segments_0_to_1p5(all_t, all_v)
    fit_results = plot_segments(segments, args.output, args.show)

    print(f"Saved figure to: {args.output}")
    print(f"All valid rows: {len(all_t)}")
    print(f"Segments in [0,1.5]: {len(segments)}")
    for idx, (seg_t, _) in enumerate(segments, start=1):
        print(
            f"segment {idx}: rows={len(seg_t)}, "
            f"time_range=[{seg_t.min():.6f}, {seg_t.max():.6f}] s"
        )
    for idx, fit_params in fit_results:
        if fit_params is None:
            print(f"segment {idx}: fit failed (try adjusting phi initial guess)")
            continue
        A_fit, omega_fit, phi_fit = fit_params
        b_fit = FIT_OFFSET_SUM - A_fit
        print(
            f"segment {idx} fit: A={A_fit:.4f}, omega={omega_fit:.4f}, "
            f"phi={phi_fit:.4f}, b={b_fit:.4f}"
        )


if __name__ == "__main__":
    main()
