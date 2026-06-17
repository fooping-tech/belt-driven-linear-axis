#!/usr/bin/env python3
"""Analyze a real-hardware balance log CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite_float(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}: {row[key]}")
    return value


def summarize(rows: list[dict[str, str]]) -> dict[str, float | int]:
    if not rows:
        return {
            "samples": 0,
            "lqr_count": 0,
            "duration_s": 0.0,
            "mean_dt_s": float("nan"),
            "min_abs_phi_rad": float("nan"),
            "max_abs_phi_rad": float("nan"),
            "max_abs_x_m": float("nan"),
            "max_abs_xdot_mps": float("nan"),
            "max_abs_phidot_rad_s": float("nan"),
            "max_abs_cmd_lag_m": float("nan"),
            "mean_abs_cmd_lag_m": float("nan"),
            "max_abs_accel_mps2": float("nan"),
        }

    t = [finite_float(row, "t_s") for row in rows]
    phi = [abs(finite_float(row, "phi_rad")) for row in rows]
    x = [abs(finite_float(row, "x_m")) for row in rows]
    xdot = [abs(finite_float(row, "xdot_mps")) for row in rows]
    phidot = [abs(finite_float(row, "phidot_rad_s")) for row in rows]
    lag = [abs(finite_float(row, "cmd_lag_m")) for row in rows]
    accel = [abs(finite_float(row, "accel_mps2")) for row in rows]
    dts = [b - a for a, b in zip(t, t[1:]) if b >= a]
    return {
        "samples": len(rows),
        "lqr_count": sum(1 for row in rows if row.get("mode") == "lqr"),
        "duration_s": t[-1] - t[0] if len(t) > 1 else 0.0,
        "mean_dt_s": sum(dts) / len(dts) if dts else float("nan"),
        "min_abs_phi_rad": min(phi),
        "max_abs_phi_rad": max(phi),
        "max_abs_x_m": max(x),
        "max_abs_xdot_mps": max(xdot),
        "max_abs_phidot_rad_s": max(phidot),
        "max_abs_cmd_lag_m": max(lag),
        "mean_abs_cmd_lag_m": sum(lag) / len(lag),
        "max_abs_accel_mps2": max(accel),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--json", action="store_true", help="print JSON")
    args = parser.parse_args()

    summary = summarize(load_rows(args.csv))
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        for key, value in summary.items():
            if isinstance(value, float):
                print(f"{key}: {value:.6g}")
            else:
                print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
