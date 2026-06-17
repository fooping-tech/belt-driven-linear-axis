"""Firmware-like observation model for sim-to-real comparison.

The real balance controller does not see MuJoCo's ideal state. It observes
the stepper-issued axis position and AS5600 angle samples, then derives
velocities by finite differences. This module mirrors that path for classic
controller tests and parameter fitting.
"""

from __future__ import annotations

import math

import numpy as np

from env import wrap_pi
from params import Params, _load_shared_config


class FirmwareObserver:
    """Convert true sim state into the state seen by the firmware controller."""

    def __init__(self, p: Params):
        self.p = p
        real_cfg = _load_shared_config().get("real", {})
        self.angle_filter_alpha = float(real_cfg.get("angle_filter_alpha", 0.35))
        self.max_abs_phidot = float(real_cfg.get("max_abs_measured_phidot_rad_s", 15.0))
        self.angle_quant_rad = 2.0 * math.pi / 4096.0
        self.x_step_m = p.issued_step_m
        self._last_theta: float | None = None
        self._last_x: float | None = None
        self._filtered_phidot = 0.0

    def reset(self, true_state: dict) -> dict:
        theta = self._measure_theta(true_state["theta"])
        x = self._measure_x(true_state)
        self._last_theta = theta
        self._last_x = x
        self._filtered_phidot = 0.0
        return self._compose(true_state, x, theta, 0.0, 0.0)

    def observe(self, true_state: dict, dt: float) -> dict:
        theta = self._measure_theta(true_state["theta"])
        x = self._measure_x(true_state)
        if self._last_theta is None or self._last_x is None or dt <= 0.0:
            self._last_theta = theta
            self._last_x = x
            return self._compose(true_state, x, theta, 0.0, self._filtered_phidot)

        raw_phidot = wrap_pi(theta - self._last_theta) / dt
        raw_phidot = float(np.clip(raw_phidot, -self.max_abs_phidot, self.max_abs_phidot))
        self._filtered_phidot += self.angle_filter_alpha * (raw_phidot - self._filtered_phidot)
        xdot = (x - self._last_x) / dt
        self._last_theta = theta
        self._last_x = x
        return self._compose(true_state, x, theta, xdot, self._filtered_phidot)

    def _measure_theta(self, theta: float) -> float:
        quantized = round(theta / self.angle_quant_rad) * self.angle_quant_rad
        return wrap_pi(quantized)

    def _measure_x(self, true_state: dict) -> float:
        issued_x = float(true_state.get("issued_pos", true_state["x"]))
        if self.x_step_m <= 0.0:
            return issued_x
        return round(issued_x / self.x_step_m) * self.x_step_m

    def _compose(self, true_state: dict, x: float, theta: float, xdot: float, phidot: float) -> dict:
        observed = dict(true_state)
        observed["x"] = x
        observed["xdot"] = xdot
        observed["theta"] = theta
        observed["phi"] = wrap_pi(theta - math.pi)
        observed["phidot"] = phidot
        observed["true_x"] = true_state["x"]
        observed["true_xdot"] = true_state["xdot"]
        observed["true_theta"] = true_state["theta"]
        observed["true_phi"] = true_state["phi"]
        observed["true_phidot"] = true_state["phidot"]
        observed["cmd_lag"] = true_state["cmd_pos"] - x
        return observed
