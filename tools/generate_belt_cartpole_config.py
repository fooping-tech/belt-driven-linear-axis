#!/usr/bin/env python3
import json
import os
from pathlib import Path


if "__file__" in globals():
    ROOT = Path(__file__).resolve().parents[1]
else:
    ROOT = Path(os.getcwd()).resolve()
CONFIG = ROOT / "sim_config" / "belt_cartpole.json"
OUT = ROOT / "include" / "generated" / "BeltCartpoleConfig.h"


def f(value):
    text = f"{float(value):.9g}"
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return f"{text}F"


def main():
    data = json.loads(CONFIG.read_text())
    mechanics = data["mechanics"]
    limits = data["stepper_limits"]
    stage = data["stage_pole"]
    timing = data["timing"]
    controller = data["controller"]
    real = data["real"]
    gains = controller["lqr_gains"]

    content = f"""#pragma once

// Generated from sim_config/belt_cartpole.json. Do not edit by hand.
namespace BeltCartpoleConfig {{
constexpr float kBeltPitchM = {f(mechanics["belt_pitch_m"])};
constexpr int kPulleyTeeth = {int(mechanics["pulley_teeth"])};
constexpr float kXLimM = {f(mechanics["x_lim_m"])};
constexpr float kSoftMarginM = {f(mechanics["soft_margin_m"])};
constexpr float kVMaxMps = {f(limits["v_max_mps"])};
constexpr float kAMaxMps2 = {f(limits["a_max_mps2"])};
constexpr float kStallThresholdM = {f(limits["stall_threshold_m"])};
constexpr float kPoleLenM = {f(stage["pole_len_m"])};
constexpr float kRodMassKg = {f(stage["rod_mass_kg"])};
constexpr float kTipMassKg = {f(stage["tip_mass_kg"])};
constexpr float kHingeDampingNms = {f(stage["hinge_damping_nms"])};
constexpr float kCtrlHz = {f(timing["ctrl_hz"])};
constexpr float kCtrlDtS = 1.0F / kCtrlHz;
constexpr float kKEnergy = {f(controller["k_energy"])};
constexpr float kSwingSatRatio = {f(controller["swing_sat_ratio"])};
constexpr float kDeadlockPhidotRadS = {f(controller["deadlock_phidot_rad_s"])};
constexpr float kDeadlockAccelMps2 = {f(controller["deadlock_accel_mps2"])};
constexpr float kCenterHoldKx = {f(controller["center_hold_kx"])};
constexpr float kCenterHoldKd = {f(controller["center_hold_kd"])};
constexpr float kSwingRailGuardXM = {f(controller.get("swing_rail_guard_x_m", 0.0))};
constexpr float kSwingTopBrakePhiRad = {f(controller.get("swing_top_brake_phi_rad", 0.0))};
constexpr float kSwingTopBrakePhidotRadS = {f(controller.get("swing_top_brake_phidot_rad_s", 0.0))};
constexpr float kSwingTopBrakeRatio = {f(controller.get("swing_top_brake_ratio", 0.0))};
constexpr float kCatchPhiRad = {f(controller["catch_phi_rad"])};
constexpr float kCatchPhidotRadS = {f(controller["catch_phidot_rad_s"])};
constexpr float kReleasePhiRad = {f(controller["release_phi_rad"])};
constexpr float kLqrKx = {f(gains[0])};
constexpr float kLqrKxd = {f(gains[1])};
constexpr float kLqrKphi = {f(gains[2])};
constexpr float kLqrKphid = {f(gains[3])};
constexpr float kLqrCenterKx = {f(controller.get("lqr_center_kx", 0.0))};
constexpr float kLqrCenterKd = {f(controller.get("lqr_center_kd", 0.0))};
constexpr float kRealXMinMm = {f(real["x_min_mm"])};
constexpr float kRealXCenterMm = {f(real["x_center_mm"])};
constexpr float kRealAngleDownDeg = {f(real["angle_down_deg"])};
constexpr int kRealAngleDirection = {int(real["angle_direction"])};
constexpr float kAngleFilterAlpha = {f(real["angle_filter_alpha"])};
constexpr unsigned int kBalanceCurrentMa = {int(real["balance_current_ma"])};
constexpr bool kBalanceSpreadCycle = {"true" if bool(real["balance_spreadcycle"]) else "false"};
constexpr unsigned long kBalanceLogIntervalMs = {int(real["balance_log_interval_ms"])}UL;
constexpr unsigned int kMaxStepsPerUpdate = {int(real["max_steps_per_update"])};
}}  // namespace BeltCartpoleConfig
"""
    OUT.parent.mkdir(parents=True, exist_ok=True)
    if not OUT.exists() or OUT.read_text() != content:
        OUT.write_text(content)
    print(f"Generated {OUT.relative_to(ROOT)} from {CONFIG.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
