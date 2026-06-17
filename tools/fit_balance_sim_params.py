#!/usr/bin/env python3
"""Fit coarse MuJoCo/sim parameters to a real balance log."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "sim" / "belt_cartpole"
sys.path.insert(0, str(SIM_DIR))

from analyze_balance_log import load_rows, summarize  # noqa: E402
from classic import HybridController  # noqa: E402
from env import BeltCartPoleSim  # noqa: E402
from observer import FirmwareObserver  # noqa: E402
from params import DEFAULT, Params, _load_shared_config  # noqa: E402


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def apply_initial_row(sim: BeltCartPoleSim, row: dict[str, str]) -> dict:
    x = float(row["x_m"])
    xdot = float(row["xdot_mps"])
    phi = float(row["phi_rad"])
    phidot = float(row["phidot_rad_s"])
    theta = phi + math.pi
    sim.data.qpos[0] = x
    sim.data.qpos[1] = theta
    sim.data.qvel[0] = xdot
    sim.data.qvel[1] = phidot
    sim.stepper.reset(x)
    sim.stepper.cmd_pos = float(row["cmd_x_m"])
    sim.stepper.cmd_vel = float(row["cmd_v_mps"])
    sim.stepper.issued_pos = x
    sim.stepper.issued_vel = xdot
    sim.data.ctrl[sim._aid] = sim.stepper.cmd_pos
    mujoco.mj_forward(sim.model, sim.data)
    return sim.state()


def simulate_summary(p: Params, duration_s: float, seed: int,
                     firmware_observer: bool = True,
                     k_energy: float | None = None,
                     initial_row: dict[str, str] | None = None) -> dict[str, float | int]:
    sim = BeltCartPoleSim(p, seed=seed, randomize=False)
    controller = HybridController(p, k_energy=k_energy)
    true_state = sim.reset(theta0=0.0)
    if initial_row is not None:
        true_state = apply_initial_row(sim, initial_row)
    observer = FirmwareObserver(p) if firmware_observer else None
    state = observer.reset(true_state) if observer else true_state
    rows: list[dict[str, str]] = []
    sample_count = int(duration_s * p.ctrl_hz)
    stall_count = 0
    max_abs_physical_lag = 0.0
    for sample in range(sample_count):
        accel = controller(state)
        sim.set_accel(accel)
        next_true_state = sim.control_step()
        if next_true_state["stalled"]:
            stall_count += 1
        physical_lag = abs(next_true_state["issued_pos"] - next_true_state["x"])
        max_abs_physical_lag = max(max_abs_physical_lag, physical_lag)
        rows.append(
            {
                "sample": str(sample),
                "t_s": f"{sample * p.ctrl_dt:.9f}",
                "mode": controller.mode,
                "x_m": f"{state['x']:.9f}",
                "xdot_mps": f"{state['xdot']:.9f}",
                "phi_rad": f"{state['phi']:.9f}",
                "phidot_rad_s": f"{state['phidot']:.9f}",
                "cmd_x_m": f"{next_true_state['cmd_pos']:.9f}",
                "cmd_v_mps": f"{next_true_state['cmd_vel']:.9f}",
                "accel_mps2": f"{accel:.9f}",
                "cmd_lag_m": f"{next_true_state['cmd_pos'] - state['x']:.9f}",
            }
        )
        state = observer.observe(next_true_state, p.ctrl_dt) if observer else next_true_state
    summary = summarize(rows)
    summary["stall_count"] = stall_count
    summary["max_abs_physical_lag_m"] = max_abs_physical_lag
    return summary


def cost(real: dict[str, float | int], sim: dict[str, float | int]) -> float:
    terms = [
        ("min_abs_phi_rad", 1.0, 0.5),
        ("max_abs_x_m", 1.0, 0.05),
        ("max_abs_cmd_lag_m", 0.4, 0.03),
        ("mean_abs_cmd_lag_m", 0.4, 0.01),
        ("max_abs_phidot_rad_s", 0.2, 5.0),
    ]
    total = 0.0
    for key, weight, scale in terms:
        rv = float(real[key])
        sv = float(sim[key])
        if not (math.isfinite(rv) and math.isfinite(sv)):
            total += 1.0e6
            continue
        total += weight * ((sv - rv) / scale) ** 2
    total += 0.1 * abs(int(sim["lqr_count"]) - int(real["lqr_count"]))
    total += 5.0 * int(sim.get("stall_count", 0))
    return total


def write_results(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "rank",
        "cost",
        "hinge_damping",
        "hinge_frictionloss",
        "servo_kp",
        "servo_kv",
        "issued_v_max",
        "k_energy",
        "rod_mass",
        "pole_len",
        "sim_min_abs_phi_rad",
        "sim_max_abs_x_m",
        "sim_max_abs_cmd_lag_m",
        "sim_mean_abs_cmd_lag_m",
        "sim_lqr_count",
        "sim_max_abs_phidot_rad_s",
        "sim_stall_count",
        "sim_max_abs_physical_lag_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="real balance log CSV")
    parser.add_argument("--out", type=Path, default=Path("reports/sim_param_fit.csv"))
    parser.add_argument("--duration-s", type=float, default=0.0, help="default: match real CSV duration")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--ideal-observer", action="store_true",
                        help="fit using true MuJoCo state instead of firmware-like observations")
    parser.add_argument("--match-initial-row", action="store_true",
                        help="initialize sim from the first row of the real CSV")
    parser.add_argument("--hinge-damping", default="0.00015,0.0004,0.001,0.002,0.004")
    parser.add_argument("--hinge-frictionloss", default="0.0005,0.001,0.002,0.004,0.008")
    parser.add_argument("--servo-kp", default="1000,2000,4000,6000")
    parser.add_argument("--servo-kv", default="20,45,90")
    parser.add_argument("--issued-v-max", default="0.20,0.24,0.26,0.30", help="comma-separated m/s values")
    parser.add_argument("--k-energy", default="", help="optional comma-separated values; default current config")
    parser.add_argument("--rod-mass", default="", help="optional comma-separated values; default current config only")
    parser.add_argument("--pole-len", default="", help="optional comma-separated values; default current config only")
    args = parser.parse_args()

    real_rows = load_rows(args.csv)
    real_summary = summarize(real_rows)
    duration = args.duration_s if args.duration_s > 0 else float(real_summary["duration_s"])
    if duration <= 0:
        raise SystemExit("Cannot infer duration from CSV; pass --duration-s.")
    initial_row = real_rows[0] if args.match_initial_row and real_rows else None

    hinge_damping_values = parse_float_list(args.hinge_damping)
    hinge_friction_values = parse_float_list(args.hinge_frictionloss)
    servo_kp_values = parse_float_list(args.servo_kp)
    servo_kv_values = parse_float_list(args.servo_kv)
    issued_v_max_values = parse_float_list(args.issued_v_max)
    k_energy_values = parse_float_list(args.k_energy) if args.k_energy else [float(_load_shared_config()["controller"]["k_energy"])]
    rod_mass_values = parse_float_list(args.rod_mass) if args.rod_mass else [DEFAULT.rod_mass]
    pole_len_values = parse_float_list(args.pole_len) if args.pole_len else [DEFAULT.pole_len]

    candidates: list[dict[str, float | int]] = []
    count = 0
    for hinge_damping in hinge_damping_values:
        for hinge_frictionloss in hinge_friction_values:
            for servo_kp in servo_kp_values:
                for servo_kv in servo_kv_values:
                    for issued_v_max in issued_v_max_values:
                        for k_energy in k_energy_values:
                            for rod_mass in rod_mass_values:
                                for pole_len in pole_len_values:
                                    count += 1
                                    p = replace(
                                        DEFAULT,
                                        hinge_damping=hinge_damping,
                                        hinge_frictionloss=hinge_frictionloss,
                                        servo_kp=servo_kp,
                                        servo_kv=servo_kv,
                                        issued_v_max=issued_v_max,
                                        rod_mass=rod_mass,
                                        pole_len=pole_len,
                                    )
                                    sim_summary = simulate_summary(
                                        p,
                                        duration,
                                        args.seed,
                                        firmware_observer=not args.ideal_observer,
                                        k_energy=k_energy,
                                        initial_row=initial_row,
                                    )
                                    candidates.append(
                                        {
                                            "rank": 0,
                                            "cost": cost(real_summary, sim_summary),
                                            "hinge_damping": hinge_damping,
                                            "hinge_frictionloss": hinge_frictionloss,
                                            "servo_kp": servo_kp,
                                            "servo_kv": servo_kv,
                                            "issued_v_max": issued_v_max,
                                            "k_energy": k_energy,
                                            "rod_mass": rod_mass,
                                            "pole_len": pole_len,
                                            "sim_min_abs_phi_rad": float(sim_summary["min_abs_phi_rad"]),
                                            "sim_max_abs_x_m": float(sim_summary["max_abs_x_m"]),
                                            "sim_max_abs_cmd_lag_m": float(sim_summary["max_abs_cmd_lag_m"]),
                                            "sim_mean_abs_cmd_lag_m": float(sim_summary["mean_abs_cmd_lag_m"]),
                                            "sim_lqr_count": int(sim_summary["lqr_count"]),
                                            "sim_max_abs_phidot_rad_s": float(sim_summary["max_abs_phidot_rad_s"]),
                                            "sim_stall_count": int(sim_summary["stall_count"]),
                                            "sim_max_abs_physical_lag_m": float(sim_summary["max_abs_physical_lag_m"]),
                                        }
                                    )

    candidates.sort(key=lambda row: float(row["cost"]))
    top_rows = candidates[: args.top]
    for index, row in enumerate(top_rows, start=1):
        row["rank"] = index
    write_results(args.out, top_rows)

    print("Real summary:")
    print(json.dumps(real_summary, indent=2, sort_keys=True))
    print(f"Evaluated candidates: {count}")
    print(f"Top results: {args.out}")
    if top_rows:
        print("Best:")
        print(json.dumps(top_rows[0], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
