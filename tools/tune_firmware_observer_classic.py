#!/usr/bin/env python3
"""Tune classic controller parameters on the firmware-observer MuJoCo sim."""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "sim" / "belt_cartpole"
sys.path.insert(0, str(SIM_DIR))

from classic import HybridController  # noqa: E402
from env import BeltCartPoleSim  # noqa: E402
from observer import FirmwareObserver  # noqa: E402
from params import DEFAULT  # noqa: E402


@dataclass(frozen=True)
class Candidate:
    v_max: float
    a_max: float
    ctrl_hz: float
    kx_scale: float
    kxd_scale: float
    kphi_scale: float
    kphid_scale: float
    k_energy: float
    catch_phi: float
    catch_phid: float
    release_phi: float
    center_kx: float
    center_kd: float
    rail_guard: float
    top_brake_phi: float
    top_brake_phid: float
    top_brake_ratio: float
    lqr_center_kx: float
    lqr_center_kd: float


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def simulate(candidate: Candidate, duration_s: float, seed: int) -> dict[str, float | int]:
    p = replace(DEFAULT, v_max=candidate.v_max, a_max=candidate.a_max, ctrl_hz=candidate.ctrl_hz)
    sim = BeltCartPoleSim(p, seed=seed, randomize=False)
    controller = HybridController(p, k_energy=candidate.k_energy)
    controller.K[0] *= candidate.kx_scale
    controller.K[1] *= candidate.kxd_scale
    controller.K[2] *= candidate.kphi_scale
    controller.K[3] *= candidate.kphid_scale
    controller.catch_phi = candidate.catch_phi
    controller.catch_phid = candidate.catch_phid
    controller.release_phi = candidate.release_phi
    controller.center_hold_kx = candidate.center_kx
    controller.center_hold_kd = candidate.center_kd
    controller.swing_rail_guard_x = candidate.rail_guard
    controller.swing_top_brake_phi = candidate.top_brake_phi
    controller.swing_top_brake_phid = candidate.top_brake_phid
    controller.swing_top_brake_ratio = candidate.top_brake_ratio
    controller.lqr_center_kx = candidate.lqr_center_kx
    controller.lqr_center_kd = candidate.lqr_center_kd

    observer = FirmwareObserver(p)
    true_state = sim.reset(theta0=0.0)
    observed = observer.reset(true_state)
    samples = int(duration_s * p.ctrl_hz)
    upright_count = 0
    lqr_count = 0
    first_lqr_s = math.nan
    min_abs_phi = float("inf")
    max_abs_phi = 0.0
    max_abs_x = 0.0
    max_abs_cmd_lag = 0.0
    max_abs_phidot = 0.0
    max_abs_accel = 0.0
    stall_count = 0

    for sample in range(samples):
        accel = controller(observed)
        sim.set_accel(accel)
        next_true_state = sim.control_step()
        if next_true_state["stalled"]:
            stall_count += 1
        if controller.mode == "lqr":
            lqr_count += 1
            if not math.isfinite(first_lqr_s):
                first_lqr_s = sample * p.ctrl_dt
        abs_phi = abs(observed["phi"])
        min_abs_phi = min(min_abs_phi, abs_phi)
        max_abs_phi = max(max_abs_phi, abs_phi)
        max_abs_x = max(max_abs_x, abs(observed["x"]))
        max_abs_cmd_lag = max(max_abs_cmd_lag, abs(next_true_state["cmd_pos"] - observed["x"]))
        max_abs_phidot = max(max_abs_phidot, abs(observed["phidot"]))
        max_abs_accel = max(max_abs_accel, abs(accel))
        if abs_phi < 0.2:
            upright_count += 1
        observed = observer.observe(next_true_state, p.ctrl_dt)

    final_phi = observed["phi"]
    final_x = observed["x"]
    upright_s = upright_count * p.ctrl_dt
    # Prefer real upright hold, then early LQR entry, while avoiding rail use.
    score = (
        100.0 * upright_s
        + 0.5 * lqr_count
        - 10.0 * min_abs_phi
        - 30.0 * max(0.0, max_abs_x - 0.14)
        - 100.0 * stall_count
    )
    return {
        "score": score,
        "upright_s": upright_s,
        "lqr_count": lqr_count,
        "first_lqr_s": first_lqr_s,
        "min_abs_phi": min_abs_phi,
        "max_abs_phi": max_abs_phi,
        "max_abs_x": max_abs_x,
        "max_abs_cmd_lag": max_abs_cmd_lag,
        "max_abs_phidot": max_abs_phidot,
        "max_abs_accel": max_abs_accel,
        "stall_count": stall_count,
        "final_phi": final_phi,
        "final_x": final_x,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("reports/classic_firmware_observer_tuning.csv"))
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--v-max", default=str(DEFAULT.v_max))
    parser.add_argument("--a-max", default=str(DEFAULT.a_max))
    parser.add_argument("--ctrl-hz", default=str(DEFAULT.ctrl_hz))
    parser.add_argument("--kx-scale", default="1")
    parser.add_argument("--kxd-scale", default="1")
    parser.add_argument("--kphi-scale", default="1")
    parser.add_argument("--kphid-scale", default="1")
    parser.add_argument("--k-energy", default="220,300,420,600")
    parser.add_argument("--catch-phi", default="0.5,0.7,0.9,1.1,1.3")
    parser.add_argument("--catch-phid", default="4,6,8,10,12")
    parser.add_argument("--release-phi", default="0.8,1.0,1.2,1.5")
    parser.add_argument("--center-kx", default="0,2,4")
    parser.add_argument("--center-kd", default="0,1,2")
    parser.add_argument("--rail-guard", default="0,0.1,0.12,0.14")
    parser.add_argument("--top-brake-phi", default="0")
    parser.add_argument("--top-brake-phid", default="0")
    parser.add_argument("--top-brake-ratio", default="0")
    parser.add_argument("--lqr-center-kx", default="0,10,20")
    parser.add_argument("--lqr-center-kd", default="0,3,5")
    args = parser.parse_args()

    rows: list[dict[str, float | int]] = []
    grids = [
        parse_float_list(args.v_max),
        parse_float_list(args.a_max),
        parse_float_list(args.ctrl_hz),
        parse_float_list(args.kx_scale),
        parse_float_list(args.kxd_scale),
        parse_float_list(args.kphi_scale),
        parse_float_list(args.kphid_scale),
        parse_float_list(args.k_energy),
        parse_float_list(args.catch_phi),
        parse_float_list(args.catch_phid),
        parse_float_list(args.release_phi),
        parse_float_list(args.center_kx),
        parse_float_list(args.center_kd),
        parse_float_list(args.rail_guard),
        parse_float_list(args.top_brake_phi),
        parse_float_list(args.top_brake_phid),
        parse_float_list(args.top_brake_ratio),
        parse_float_list(args.lqr_center_kx),
        parse_float_list(args.lqr_center_kd),
    ]
    for values in itertools.product(*grids):
        candidate = Candidate(*values)
        result = simulate(candidate, args.duration_s, args.seed)
        rows.append({**candidate.__dict__, **result})

    rows.sort(key=lambda row: float(row["score"]), reverse=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(Candidate.__dataclass_fields__) + [
        "score",
        "upright_s",
        "lqr_count",
        "first_lqr_s",
        "min_abs_phi",
        "max_abs_phi",
        "max_abs_x",
        "max_abs_cmd_lag",
        "max_abs_phidot",
        "max_abs_accel",
        "stall_count",
        "final_phi",
        "final_x",
    ]
    with args.out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows[: args.top])

    print(f"evaluated={len(rows)}")
    print(f"top_results={args.out}")
    for rank, row in enumerate(rows[: min(args.top, 5)], start=1):
        print(f"rank={rank} {row}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
