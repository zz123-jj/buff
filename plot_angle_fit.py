#!/usr/bin/env python3
"""Plot differential velocity and recorded fitted velocity function from CSV.

Expected CSV columns (new format):
- stamp_sec,time_from_start_s,continues_angle_rad,continues_angle_deg
- is_tracking,fit_success,sin_a,sin_omega,sin_phi,sin_b,fit_start_time_sec

If fit-related columns are missing, the script still plots raw differential
velocity and reports that no recorded fit curve can be reconstructed.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _parse_float(value: str | None, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    return float(value)


def load_csv(csv_path: Path) -> tuple[dict[str, np.ndarray], bool]:
    """Load CSV data and report whether fit columns are available."""
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    stamp_sec: list[float] = []
    time_from_start_s: list[float] = []
    continues_angle_rad: list[float] = []

    is_tracking: list[float] = []
    fit_success: list[float] = []
    sin_a: list[float] = []
    sin_omega: list[float] = []
    sin_phi: list[float] = []
    sin_b: list[float] = []
    fit_start_time_sec: list[float] = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV header is empty")

        base_required = {"stamp_sec", "time_from_start_s", "continues_angle_rad"}
        if not base_required.issubset(set(reader.fieldnames)):
            raise ValueError(
                "CSV must contain stamp_sec,time_from_start_s,continues_angle_rad"
            )

        has_fit_columns = {
            "is_tracking",
            "fit_success",
            "sin_a",
            "sin_omega",
            "sin_phi",
            "sin_b",
            "fit_start_time_sec",
        }.issubset(set(reader.fieldnames))

        for row in reader:
            s = _parse_float(row.get("stamp_sec"))
            # Keep strictly increasing absolute timestamps.
            if stamp_sec and s <= stamp_sec[-1] + 1e-9:
                continue

            stamp_sec.append(s)
            time_from_start_s.append(_parse_float(row.get("time_from_start_s")))
            continues_angle_rad.append(_parse_float(row.get("continues_angle_rad")))

            if has_fit_columns:
                is_tracking.append(_parse_float(row.get("is_tracking")))
                fit_success.append(_parse_float(row.get("fit_success")))
                sin_a.append(_parse_float(row.get("sin_a")))
                sin_omega.append(_parse_float(row.get("sin_omega")))
                sin_phi.append(_parse_float(row.get("sin_phi")))
                sin_b.append(_parse_float(row.get("sin_b")))
                fit_start_time_sec.append(_parse_float(row.get("fit_start_time_sec")))
            else:
                is_tracking.append(0.0)
                fit_success.append(0.0)
                sin_a.append(0.0)
                sin_omega.append(0.0)
                sin_phi.append(0.0)
                sin_b.append(0.0)
                fit_start_time_sec.append(0.0)

    if len(stamp_sec) < 3:
        raise ValueError("Need at least 2 valid rows")

    data = {
        "stamp_sec": np.asarray(stamp_sec, dtype=float),
        "time_from_start_s": np.asarray(time_from_start_s, dtype=float),
        "continues_angle_rad": np.asarray(continues_angle_rad, dtype=float),
        "is_tracking": np.asarray(is_tracking, dtype=float),
        "fit_success": np.asarray(fit_success, dtype=float),
        "sin_a": np.asarray(sin_a, dtype=float),
        "sin_omega": np.asarray(sin_omega, dtype=float),
        "sin_phi": np.asarray(sin_phi, dtype=float),
        "sin_b": np.asarray(sin_b, dtype=float),
        "fit_start_time_sec": np.asarray(fit_start_time_sec, dtype=float),
    }
    return data, has_fit_columns


def build_raw_velocity(data: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Compute raw differential velocity dy/dt from logged continuous angle."""
    stamp = data["stamp_sec"]
    x = data["time_from_start_s"]
    y = data["continues_angle_rad"]

    dt = np.diff(stamp)
    dy = np.diff(y)

    x_mid = 0.5 * (x[:-1] + x[1:])
    stamp_mid = 0.5 * (stamp[:-1] + stamp[1:])

    vel_raw = np.full_like(dy, np.nan, dtype=float)
    valid_dt = dt > 1e-6
    vel_raw[valid_dt] = dy[valid_dt] / dt[valid_dt]

    return {
        "x_mid": x_mid,
        "stamp_mid": stamp_mid,
        "vel_raw": vel_raw,
    }


def reconstruct_recorded_velocity_fit(
    data: dict[str, np.ndarray],
) -> tuple[np.ndarray, list[dict[str, float | int]]]:
    """Reconstruct recorded velocity fit on midpoint samples.

    Logged model is dy/dt = a*sin(omega*t + phi) + b, where
    t = (stamp_sec - fit_start_time_sec).
    """
    stamp = data["stamp_sec"]
    fit_success = data["fit_success"]
    sin_a = data["sin_a"]
    sin_omega = data["sin_omega"]
    sin_phi = data["sin_phi"]
    sin_b = data["sin_b"]
    fit_start = data["fit_start_time_sec"]

    # Use midpoint timestamps to align with finite-difference velocity samples.
    stamp_mid = 0.5 * (stamp[:-1] + stamp[1:])

    # Use parameters from the right endpoint of each diff interval.
    fit_success_mid = fit_success[1:]
    sin_a_mid = sin_a[1:]
    sin_omega_mid = sin_omega[1:]
    sin_phi_mid = sin_phi[1:]
    sin_b_mid = sin_b[1:]
    fit_start_mid = fit_start[1:]

    vel_fit = np.full_like(stamp_mid, np.nan, dtype=float)

    valid = (
        (fit_success_mid > 0.5)
        & (fit_start_mid > 0.0)
        & (np.abs(sin_omega_mid) > 1e-6)
    )
    valid_idx = np.where(valid)[0]
    if valid_idx.size == 0:
        return vel_fit, []

    segments: list[tuple[int, int]] = []
    seg_start = int(valid_idx[0])
    seg_prev = int(valid_idx[0])
    seg_t0 = float(fit_start_mid[seg_start])

    for idx_raw in valid_idx[1:]:
        idx = int(idx_raw)
        split = (idx != seg_prev + 1) or (abs(float(fit_start_mid[idx]) - seg_t0) > 1e-6)
        if split:
            segments.append((seg_start, seg_prev))
            seg_start = idx
            seg_t0 = float(fit_start_mid[idx])
        seg_prev = idx
    segments.append((seg_start, seg_prev))

    segment_meta: list[dict[str, float | int]] = []

    for seg_id, (i0, i1) in enumerate(segments, start=1):
        a = float(sin_a_mid[i0])
        omega = float(sin_omega_mid[i0])
        phi = float(sin_phi_mid[i0])
        b = float(sin_b_mid[i0])
        t0_abs = float(fit_start_mid[i0])

        if abs(omega) < 1e-6:
            continue

        tau = stamp_mid[i0 : i1 + 1] - t0_abs
        vel_fit[i0 : i1 + 1] = a * np.sin(omega * tau + phi) + b

        segment_meta.append(
            {
                "segment_id": seg_id,
                "i0": i0,
                "i1": i1,
                "a": a,
                "omega": omega,
                "phi": phi,
                "b": b,
                "t0_abs": t0_abs,
            }
        )

    return vel_fit, segment_meta


def plot_velocity_and_recorded_fit(
    diff_data: dict[str, np.ndarray],
    vel_fit: np.ndarray,
    segments: list[dict[str, float | int]],
    output_path: Path,
    show: bool,
) -> None:
    x_mid = diff_data["x_mid"]
    vel_raw = diff_data["vel_raw"]

    plt.figure(figsize=(12, 6))
    plt.plot(x_mid, vel_raw, "o", ms=3, alpha=0.45, label="dy/dt (raw diff)")

    if np.isfinite(vel_fit).any():
        plt.plot(x_mid, vel_fit, "r-", lw=2.2, label="recorded velocity fit")
    else:
        plt.plot([], [], "r-", lw=2.2, label="recorded velocity fit (not available)")

    for seg in segments:
        i0 = int(seg["i0"])
        plt.axvline(x=x_mid[i0], color="gray", linestyle="--", alpha=0.25)

    title = "Differential Velocity + Recorded Fit Function"
    if segments:
        first = segments[0]
        eq = (
            f"dy/dt = {float(first['a']):.5f}*sin({float(first['omega']):.5f}*t + "
            f"{float(first['phi']):.5f}) + {float(first['b']):.5f}"
        )
    else:
        eq = "No valid recorded fit parameters found in CSV"

    plt.title(title)
    plt.xlabel("time_from_start_s")
    plt.ylabel("d(continues_angle_rad)/dt [rad/s]")
    plt.ylim(-3, 5)
    plt.grid(alpha=0.25)
    plt.legend(loc="best")
    plt.text(0.01, 0.02, eq, transform=plt.gca().transAxes, fontsize=10, va="bottom")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot differential velocity and recorded fitted velocity function"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("log/angle_time.csv"),
        help="Path to CSV file (default: log/angle_time.csv)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("log/angle_recorded_velocity_fit.png"),
        help="Output image path (default: log/angle_recorded_velocity_fit.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plot window after saving",
    )
    args = parser.parse_args()

    data, has_fit_columns = load_csv(args.csv)
    diff_data = build_raw_velocity(data)
    vel_fit, segments = reconstruct_recorded_velocity_fit(data)
    plot_velocity_and_recorded_fit(diff_data, vel_fit, segments, args.output, args.show)

    print(f"Saved figure to: {args.output}")
    print(f"Total samples: {len(data['stamp_sec'])}")
    print(f"Differential points: {len(diff_data['x_mid'])}")
    print(f"Fit segments reconstructed: {len(segments)}")
    if segments:
        for seg in segments:
            period = 2.0 * math.pi / float(seg["omega"]) if abs(float(seg["omega"])) > 1e-9 else 0.0
            print(
                "segment "
                f"{int(seg['segment_id'])}: a={float(seg['a']):.6f}, "
                f"omega={float(seg['omega']):.6f}, phi={float(seg['phi']):.6f}, "
                f"b={float(seg['b']):.6f}, t0={float(seg['t0_abs']):.6f}, "
                f"period={period:.6f}"
            )
    else:
        if has_fit_columns:
            print("Fit columns exist but no valid fit_success rows; plotted raw velocity only.")
        else:
            print("No recorded fit columns found; plotted raw velocity only.")


if __name__ == "__main__":
    main()
