#!/usr/bin/env python3
"""Run step-loss parameter sweeps over the firmware serial command interface."""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import glob
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Iterable


REQUIRED_COLUMNS = ("speed_mm_s", "accel_mm_s2", "current_ma", "chop_mode", "microsteps")
RESULT_COLUMNS = (
    "timestamp",
    "speed_mm_s",
    "accel_mm_s2",
    "current_ma",
    "chop_mode",
    "microsteps",
    "test1_result",
    "test2_result",
    "final_result",
    "failure_reason",
    "final_limit_state",
    "final_limit_timing",
    "final_score",
    "final_error_mm",
    "final_remaining_steps",
    "elapsed_sec",
    "sg_min",
    "sg_max",
    "sg_avg",
    "sg_count",
    "sg_min_valid",
    "sg_max_valid",
    "sg_avg_valid",
    "sg_valid_count",
    "sg_zero_count",
    "sg_low_count_50",
    "sg_low_count_100",
    "sg_low_count_150",
    "sg_low_ratio_50",
    "sg_low_ratio_100",
    "sg_low_ratio_150",
    "motion_update_count",
    "max_update_gap_us",
    "avg_update_gap_us",
    "diag_triggered",
    "sgthrs",
    "tcoolthrs",
    "test_condition_id",
    "repeat_index",
    "sg_enabled",
    "sg_interval_ms",
    "sglog_enabled",
    "move_start_limit_state",
    "move_end_limit_state",
    "home_end_limit_state",
    "timeout_limit_state",
    "limit_transition_count",
    "limit_first_trigger_timing",
    "timeout_current_position",
    "timeout_target_position",
    "timeout_remaining_steps",
    "requested_current_ma",
    "applied_current_ma",
    "tmc_uart_ok",
    "config_tmc_uart_ok",
    "driver_status",
    "validate_tmc_uart_ok",
    "validate_driver_status",
    "validate_ifcnt_before",
    "validate_ifcnt_after",
    "validate_ifcnt_delta",
    "validate_gstat",
    "validate_drv_status",
    "validate_requested_current_ma",
    "validate_applied_current_ma",
    "validate_current_error_ma",
    "validate_current_error_ratio",
    "validate_current_tolerance_ma",
    "validate_fail_reason_detail",
    "re_stage",
    "re_reason",
    "current_before_re",
    "position_before_re",
    "limit_state_before_re",
    "motion_state_before_re",
    "failure_detail",
    "failure_stage",
    "failure_evidence",
    "timeout_stage",
    "timeout_elapsed_ms",
    "timeout_motion_state",
    "timeout_last_step_time_ms",
    "timeout_expected_duration_ms",
    "timeout_phase",
    "timeout_command",
    "timeout_firmware_responsive",
    "timeout_diag_probe",
    "timeout_status_probe",
    "timeout_mt_probe",
    "expected_position",
    "actual_position",
    "abs_error_mm",
    "error_threshold_mm",
    "remaining_steps_threshold",
    "limit_expected_state",
    "limit_actual_state",
    "limit_failure_detail",
    "current_error_ma",
    "current_error_ratio",
    "current_tolerance_ma",
    "sg_health_state",
    "diagnostic_tags",
    "diagnostic_comment",
)

FAILURE_LEGEND = {
    "OK": "pass",
    "SL": "suspected step loss",
    "TO": "timeout",
    "SETUP_TO": "setup command timeout",
    "HE": "homing error",
    "ME": "motion error",
    "RE": "command rejected",
    "LOFF": "final limit OFF",
    "LPOS": "final limit ON too far from zero",
}

SETUP_EXPECTS = {
    "microstep": ("TMC2209 microstep set:", "microstep command ignored:", "sim:"),
    "current": ("TMC2209 current set:", "Current command ignored:", "sim:"),
    "mode": ("TMC2209 chop mode set:", "Chop mode command ignored:", "sim:"),
    "profile": ("Motion profile set:", "sim:"),
    "accel": ("Acceleration set:", "sim:"),
    "speed": ("Default move speed set:", "sim:"),
}


@dataclasses.dataclass(frozen=True)
class SweepParam:
    speed_mm_s: float
    accel_mm_s2: float
    current_ma: int
    chop_mode: str
    microsteps: int
    test_condition_id: str = "NA"
    repeat_index: int | None = None
    sg_enabled: int | None = None
    sg_interval_ms: int | None = None
    sglog_enabled: int | None = None
    validate_only: int | None = None


@dataclasses.dataclass
class SgStats:
    sg_min: int | None = None
    sg_max: int | None = None
    sg_avg: float | None = None
    sg_count: int | None = None
    sg_min_valid: int | None = None
    sg_max_valid: int | None = None
    sg_avg_valid: float | None = None
    sg_valid_count: int | None = None
    sg_zero_count: int | None = None
    sg_low_count_50: int | None = None
    sg_low_count_100: int | None = None
    sg_low_count_150: int | None = None
    sg_low_ratio_50: float | None = None
    sg_low_ratio_100: float | None = None
    sg_low_ratio_150: float | None = None
    motion_update_count: int | None = None
    max_update_gap_us: int | None = None
    avg_update_gap_us: float | None = None
    move_start_limit_state: str | None = None
    move_end_limit_state: str | None = None
    home_end_limit_state: str | None = None
    timeout_limit_state: str | None = None
    limit_transition_count: int | None = None
    limit_first_trigger_timing: str | None = None
    timeout_current_position: float | None = None
    timeout_target_position: float | None = None
    timeout_remaining_steps: int | None = None
    requested_current_ma: int | None = None
    applied_current_ma: int | None = None
    tmc_uart_ok: str | None = None
    config_tmc_uart_ok: str | None = None
    driver_status: str | None = None
    validate_tmc_uart_ok: str | None = None
    validate_driver_status: str | None = None
    validate_ifcnt_before: int | None = None
    validate_ifcnt_after: int | None = None
    validate_ifcnt_delta: int | None = None
    validate_gstat: int | None = None
    validate_drv_status: int | None = None
    validate_requested_current_ma: int | None = None
    validate_applied_current_ma: int | None = None
    validate_current_error_ma: int | None = None
    validate_current_error_ratio: float | None = None
    validate_current_tolerance_ma: int | None = None
    validate_fail_reason_detail: str | None = None
    re_stage: str | None = None
    re_reason: str | None = None
    current_before_re: int | None = None
    position_before_re: float | None = None
    limit_state_before_re: str | None = None
    motion_state_before_re: str | None = None
    diag_triggered: str | None = None
    sgthrs: int | None = None
    tcoolthrs: int | None = None


@dataclasses.dataclass
class TestOutcome:
    name: str
    result: str
    failure_reason: str
    final_limit_state: str
    final_limit_timing: str
    score: float
    final_error_mm: float | None
    final_remaining_steps: int | None
    sg: SgStats
    final_status: str
    log_excerpt: list[str]
    timeout_diag: "TimeoutDiagnostic | None" = None


@dataclasses.dataclass
class TimeoutDiagnostic:
    phase: str
    command: str
    elapsed_ms: int
    serial_tail: list[str]
    diag_code: str
    diag_lines: list[str]
    status_code: str
    status_lines: list[str]
    mt_code: str
    mt_lines: list[str]

    @property
    def firmware_responsive(self) -> bool:
        return self.diag_code == "OK" or self.status_code == "OK" or self.mt_code == "OK"


@dataclasses.dataclass
class SweepResult:
    timestamp: str
    param: SweepParam
    test1: TestOutcome
    test2: TestOutcome
    final_result: str
    failure_reason: str
    final_limit_state: str
    final_limit_timing: str
    final_score: float
    final_error_mm: float | None
    final_remaining_steps: int | None
    sg: SgStats
    elapsed_sec: float


class SerialRunner:
    def __init__(self, port: str, baudrate: int, line_timeout_sec: float = 0.1) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise SystemExit("pyserial is required for hardware sweeps: python3 -m pip install pyserial") from exc

        self.serial = serial.Serial(port, baudrate=baudrate, timeout=line_timeout_sec)
        time.sleep(2.0)
        self.drain(1.0)

    def close(self) -> None:
        self.serial.close()

    def drain(self, duration_sec: float) -> list[str]:
        deadline = time.monotonic() + duration_sec
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is not None:
                lines.append(line)
        return lines

    def read_line(self) -> str | None:
        raw = self.serial.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def send(self, command: str) -> None:
        self.serial.write((command + "\n").encode("utf-8"))
        self.serial.flush()

    def send_and_wait(
        self,
        command: str,
        expect: Iterable[str],
        timeout_sec: float,
        reject_as_error: bool = True,
        phase: str | None = None,
        no_line_probe_interval_sec: float | None = None,
    ) -> tuple[bool, str, list[str]]:
        self.send(command)
        expected = tuple(expect)
        deadline = time.monotonic() + timeout_sec
        last_line_at = time.monotonic()
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                now = time.monotonic()
                if phase is not None and no_line_probe_interval_sec is not None and now - last_line_at >= no_line_probe_interval_sec:
                    no_serial_ms = int(round((now - last_line_at) * 1000.0))
                    probe_line = f"[probe] phase={phase} command={command} no_serial_for={no_serial_ms}ms"
                    print(probe_line, flush=True)
                    lines.append(probe_line)
                    self.send("diag")
                    last_line_at = now
                continue
            last_line_at = time.monotonic()
            lines.append(line)
            lowered = line.lower()
            if reject_as_error and ("rejected" in lowered or lowered.startswith("unknown command")):
                return False, "RE", lines
            if "motion error" in lowered:
                return False, "ME", lines
            if "homing error" in lowered:
                return False, "HE", lines
            if any(marker in line for marker in expected):
                return True, "OK", lines
        return False, "TO", lines

    def send_and_wait_raw_no_probe(
        self,
        command: str,
        expect: Iterable[str],
        timeout_sec: float,
    ) -> tuple[bool, str, list[str]]:
        self.send(command)
        expected = tuple(expect)
        deadline = time.monotonic() + timeout_sec
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            lines.append(line)
            if any(marker in line for marker in expected):
                return True, "OK", lines
        return False, "TO", lines

    def wait_for_homing(self, timeout_sec: float) -> tuple[bool, str, list[str]]:
        return self.send_and_wait("h", ("Homing complete",), timeout_sec)

    def wait_for_move(self, command: str, timeout_sec: float) -> tuple[bool, str, list[str]]:
        return self.send_and_wait(command, ("Move complete",), timeout_sec)

    def status(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        ok, code, lines = self.send_and_wait("s", ("state=",), timeout_sec, reject_as_error=False)
        if not ok:
            return code, lines
        return "OK", lines

    def status_no_probe(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        ok, code, lines = self.send_and_wait_raw_no_probe("s", ("state=",), timeout_sec)
        if not ok:
            return code, lines
        return "OK", lines

    def motion_timing_no_probe(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        ok, code, lines = self.send_and_wait_raw_no_probe("mt", ("MOTION_TIMING_SUMMARY",), timeout_sec)
        if not ok:
            return code, lines
        return "OK", lines

    def diag_no_probe(self, timeout_sec: float = 1.0) -> tuple[str, list[str]]:
        ok, code, lines = self.send_and_wait_raw_no_probe("diag", ("DIAG,",), timeout_sec)
        if not ok:
            return code, lines
        return "OK", lines


class SimulatedRunner:
    """Small deterministic runner for report-generation smoke tests."""

    def __init__(self) -> None:
        self.position_units = 0

    def close(self) -> None:
        return

    def drain(self, duration_sec: float) -> list[str]:
        _ = duration_sec
        return []

    def send_and_wait(
        self,
        command: str,
        expect: Iterable[str],
        timeout_sec: float,
        reject_as_error: bool = True,
        phase: str | None = None,
        no_line_probe_interval_sec: float | None = None,
    ) -> tuple[bool, str, list[str]]:
        _ = expect, timeout_sec, reject_as_error, phase, no_line_probe_interval_sec
        if command.startswith("microstep ") or command.startswith("i ") or command.startswith("mode "):
            return True, "OK", [f"sim: {command}"]
        if command.startswith("profile "):
            return True, "OK", ["Motion profile set: trap"]
        if command.startswith("a "):
            return True, "OK", ["Acceleration set: sim"]
        if command.startswith("v "):
            return True, "OK", ["Default move speed set: sim"]
        return True, "OK", [f"sim: {command}"]

    def wait_for_homing(self, timeout_sec: float) -> tuple[bool, str, list[str]]:
        _ = timeout_sec
        self.position_units = 0
        return True, "OK", ["Homing complete. X=0.00 mm", "state=Ready limitDebounced=ON"]

    def wait_for_move(self, command: str, timeout_sec: float) -> tuple[bool, str, list[str]]:
        _ = timeout_sec
        if command == "1":
            self.position_units += 1
        elif command == "5":
            self.position_units += 5
        elif command.lower() == "b":
            self.position_units -= 1
        return True, "OK", ["Move complete", self._status_line()]

    def send_and_wait_raw_no_probe(
        self,
        command: str,
        expect: Iterable[str],
        timeout_sec: float,
    ) -> tuple[bool, str, list[str]]:
        return self.send_and_wait(command, expect, timeout_sec, reject_as_error=False)

    def status(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        _ = timeout_sec
        return "OK", [self._status_line()]

    def status_no_probe(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        return self.status(timeout_sec)

    def motion_timing_no_probe(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        _ = timeout_sec
        return "OK", [
            "MOTION_TIMING_SUMMARY,motion_update_count=1,max_update_gap_us=0,avg_update_gap_us=0.0,move_start_limit_state=OFF,move_end_limit_state=ON,home_end_limit_state=ON,timeout_limit_state=NA,limit_transition_count=1,limit_first_trigger_timing=DURING_MOVE,timeout_current_position=NA,timeout_target_position=NA,timeout_remaining_steps=NA"
        ]

    def diag_no_probe(self, timeout_sec: float = 1.0) -> tuple[str, list[str]]:
        _ = timeout_sec
        return "OK", [
            "DIAG,app_state=Ready,homing_state=Done,motion_state=Idle,current_position_steps=0,target_steps=0,remaining_steps=0,limit_raw=ON,limit_debounced=ON,motion_current_speed_steps_s=0.00,motion_step_interval_us=0,motion_last_step_us=0,now_us=0,last_step_pulse_us=0,step_pulse_count=0,last_no_step_reason=NOT_MOVING,motion_no_step_reason=NOT_MOVING,homing_no_step_reason=DONE,last_move_reject_reason=NONE,heartbeat_enabled=1,max_loop_gap_us=0,max_motion_update_gap_us=0"
        ]

    def _status_line(self) -> str:
        limit = "ON" if self.position_units <= 0 else "OFF"
        return f"state=Ready homing=Done motion=Idle pos={self.position_units * 10}.00mm limitDebounced={limit}"


def main() -> int:
    args = parse_args()
    params = load_params(args.csv)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    reports_dir = Path(args.reports_dir)
    reports_dir.mkdir(parents=True, exist_ok=True)
    report_stem = report_stem_for_params(params)
    report_path = Path(args.report) if args.report else reports_dir / f"{report_stem}_{timestamp}.md"
    result_csv_path = report_path.with_suffix(".csv") if args.report else reports_dir / f"{report_stem}_{timestamp}.csv"

    selected_port = "simulate" if args.simulate else resolve_port(args.port)
    runner = SimulatedRunner() if args.simulate else SerialRunner(selected_port, args.baudrate)

    results: list[SweepResult] = []
    try:
        for index, param in enumerate(params, start=1):
            runner.drain(0.2)
            prefix = f"[{index}/{len(params)}] speed={fmt_num(param.speed_mm_s)} accel={fmt_num(param.accel_mm_s2)} current={param.current_ma} mode={param.chop_mode} microsteps={param.microsteps}"
            print(prefix, flush=True)
            result = run_param(runner, param, args.return_speed, args.command_timeout, args.homing_timeout, args.limit_position_tolerance, args.limit_settle_sec, args.no_line_probe_interval, timestamp)
            if result.final_result != "PASS":
                runner.drain(0.5)
            results.append(result)
            print(f"  test1 => {format_test_outcome(result.test1)}", flush=True)
            print(f"  test2 => {format_test_outcome(result.test2)}", flush=True)
            print(f"  => {result.final_result} score={result.final_score:.1f} reason={result.failure_reason} limit={result.final_limit_state} timing={result.final_limit_timing} error={format_optional_mm(result.final_error_mm)} remainingSteps={format_optional_int(result.final_remaining_steps)} tests={result.test1.result}/{result.test2.result}", flush=True)
    finally:
        runner.close()

    write_results_csv(result_csv_path, results)
    if args.legacy_heatmaps:
        heatmap_paths, heatmap_warning = write_optional_heatmaps(reports_dir, timestamp, results)
    else:
        heatmap_paths = []
        heatmap_warning = "Legacy flat PNG heatmaps are disabled; see Plot Report for generated figures."
    write_markdown_report(
        report_path=report_path,
        result_csv_path=result_csv_path,
        input_csv=Path(args.csv),
        timestamp=timestamp,
        port=selected_port,
        baudrate=args.baudrate,
        return_speed=args.return_speed,
        limit_position_tolerance=args.limit_position_tolerance,
        results=results,
        heatmap_paths=heatmap_paths,
        heatmap_warning=heatmap_warning,
    )
    plot_report_dir: Path | None = None
    plot_report_ok = False
    if not args.skip_plot_report:
        plot_report_dir, plot_report_ok, plot_output = run_plot_report(
            result_csv_path=result_csv_path,
            reports_dir=reports_dir,
            timestamp=timestamp,
            show_cell_labels=not args.no_plot_cell_labels,
            error_scale=args.plot_error_scale,
            figure_format=args.plot_format,
        )
        append_plot_report_section(report_path, plot_report_dir, plot_report_ok, plot_output, args.plot_format)
    print(f"Report: {report_path}")
    print(f"Results CSV: {result_csv_path}")
    if plot_report_dir is not None and plot_report_ok:
        print(f"Plot report: {plot_report_dir / 'report.md'}")
    elif plot_report_dir is not None:
        print(f"Plot report: not generated; see {report_path}")
    return 0


def has_sg_overhead_params(params: list[SweepParam]) -> bool:
    for param in params:
        if param.test_condition_id != "NA":
            return True
        if param.repeat_index is not None or param.sg_enabled is not None or param.sg_interval_ms is not None or param.sglog_enabled is not None:
            return True
    return False


def report_stem_for_params(params: list[SweepParam]) -> str:
    condition_ids = {param.test_condition_id for param in params}
    if condition_ids == {"B_REPRO"}:
        return "b_repro"
    if condition_ids and all(condition_id.startswith("CUR_") for condition_id in condition_ids):
        return "current_sweep"
    if has_sg_overhead_params(params):
        return "sg_overhead_validation"
    return "step_loss_sweep"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep stepper parameters and generate step-loss reports.")
    parser.add_argument("--csv", default="tools/step_loss_params.csv", help="input CSV path")
    parser.add_argument("--params", dest="csv", default=argparse.SUPPRESS, help="input CSV path alias")
    parser.add_argument("--port", default="auto", help="serial port, or auto")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baudrate")
    parser.add_argument("--return-speed", type=float, default=30.0, help="slow return speed before b moves")
    parser.add_argument("--reports-dir", default="reports", help="report output directory")
    parser.add_argument("--report", default="", help="explicit Markdown report path")
    parser.add_argument("--command-timeout", type=float, default=30.0, help="timeout for one normal move")
    parser.add_argument("--homing-timeout", type=float, default=45.0, help="timeout for homing")
    parser.add_argument("--limit-position-tolerance", type=float, default=0.5, help="max final position error in mm when the limit turns ON")
    parser.add_argument("--limit-settle-sec", type=float, default=0.2, help="extra wait after final b when raw limit is ON but debounced limit is still OFF")
    parser.add_argument("--no-line-probe-interval", type=float, default=1.0, help="send lightweight diag probe after this many seconds without serial lines during long waits; <=0 disables")
    parser.add_argument("--skip-plot-report", action="store_true", help="do not run tools/plot_step_loss_sweep.py after writing the sweep CSV")
    parser.add_argument("--no-plot-cell-labels", action="store_true", help="do not label cells in generated plot heatmaps")
    parser.add_argument("--plot-error-scale", choices=("linear", "log"), default="linear", help="error scale for generated plot report")
    parser.add_argument("--plot-format", choices=("png", "svg", "pdf"), default="png", help="figure format for generated plot report")
    parser.add_argument("--legacy-heatmaps", action="store_true", help="also write the old OK/NG heatmap PNG files directly under --reports-dir")
    parser.add_argument("--simulate", action="store_true", help="run without hardware and generate sample reports")
    return parser.parse_args()


def load_params(path: str) -> list[SweepParam]:
    csv_path = Path(path)
    if not csv_path.exists():
        raise SystemExit(f"Input CSV not found: {csv_path}")

    params: list[SweepParam] = []
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = [name for name in REQUIRED_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(f"Input CSV is missing required columns: {', '.join(missing)}")
        for row_number, row in enumerate(reader, start=2):
            params.append(parse_param_row(row, row_number))

    if not params:
        raise SystemExit("Input CSV has no parameter rows")
    return params


def parse_param_row(row: dict[str, str], row_number: int) -> SweepParam:
    try:
        speed = float_required(row, "speed_mm_s", row_number)
        accel = float_required(row, "accel_mm_s2", row_number)
        current = int_required(row, "current_ma", row_number)
        microsteps = int_required(row, "microsteps", row_number)
        chop_mode = row["chop_mode"].strip().lower()
        test_condition_id = optional_text(row.get("test_condition_id")) or "NA"
        repeat_index = optional_csv_int(row, "repeat_index", row_number, min_value=1)
        sg_enabled = optional_csv_int(row, "sg_enabled", row_number, min_value=0, max_value=1)
        sg_interval_ms = optional_csv_int(row, "sg_interval_ms", row_number, min_value=1, max_value=60000)
        sglog_enabled = optional_csv_int(row, "sglog_enabled", row_number, min_value=0, max_value=1)
        validate_only = optional_csv_int(row, "validate_only", row_number, min_value=0, max_value=1)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if speed <= 0:
        raise SystemExit(f"Row {row_number}: speed_mm_s must be positive")
    if accel <= 0:
        raise SystemExit(f"Row {row_number}: accel_mm_s2 must be positive")
    if current <= 0:
        raise SystemExit(f"Row {row_number}: current_ma must be positive")
    if chop_mode not in ("stealth", "stealthchop", "spread", "spreadcycle"):
        raise SystemExit(f"Row {row_number}: chop_mode must be stealth or spread")
    if microsteps not in (8, 16):
        raise SystemExit(f"Row {row_number}: microsteps must be 8 or 16")

    if chop_mode == "stealthchop":
        chop_mode = "stealth"
    if chop_mode == "spreadcycle":
        chop_mode = "spread"

    return SweepParam(
        speed,
        accel,
        current,
        chop_mode,
        microsteps,
        test_condition_id,
        repeat_index,
        sg_enabled,
        sg_interval_ms,
        sglog_enabled,
        validate_only,
    )


def float_required(row: dict[str, str], name: str, row_number: int) -> float:
    value = row.get(name, "").strip()
    if not value:
        raise ValueError(f"Row {row_number}: {name} is required")
    return float(value)


def int_required(row: dict[str, str], name: str, row_number: int) -> int:
    value = row.get(name, "").strip()
    if not value:
        raise ValueError(f"Row {row_number}: {name} is required")
    return int(value)


def optional_csv_int(
    row: dict[str, str],
    name: str,
    row_number: int,
    min_value: int | None = None,
    max_value: int | None = None,
) -> int | None:
    text = optional_text(row.get(name))
    if text is None:
        return None
    try:
        value = int(text)
    except ValueError as exc:
        raise ValueError(f"Row {row_number}: {name} must be an integer or NA") from exc
    if min_value is not None and value < min_value:
        raise ValueError(f"Row {row_number}: {name} must be >= {min_value}")
    if max_value is not None and value > max_value:
        raise ValueError(f"Row {row_number}: {name} must be <= {max_value}")
    return value


def resolve_port(port: str) -> str:
    if port != "auto":
        return port

    ports: list[str] = []
    try:
        from serial.tools import list_ports  # type: ignore

        ports = [item.device for item in list_ports.comports()]
    except ImportError:
        patterns = ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*")
        for pattern in patterns:
            ports.extend(glob.glob(pattern))

    ports = sorted(set(ports))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise SystemExit("No serial ports found. Pass --port /dev/cu... or use --simulate.")
    raise SystemExit("Multiple serial ports found; pass --port explicitly:\n" + "\n".join(f"  {p}" for p in ports))


def run_param(
    runner: SerialRunner | SimulatedRunner,
    param: SweepParam,
    return_speed: float,
    command_timeout: float,
    homing_timeout: float,
    limit_position_tolerance: float,
    limit_settle_sec: float,
    no_line_probe_interval_sec: float,
    timestamp: str,
) -> SweepResult:
    start = time.monotonic()
    if param.validate_only == 1:
        test1 = run_current_validate_only(runner, param, "validate", limit_position_tolerance)
        test2 = TestOutcome(
            name="skip",
            result="PASS",
            failure_reason="OK",
            final_limit_state="UNKNOWN",
            final_limit_timing="UNKNOWN",
            score=100.0,
            final_error_mm=None,
            final_remaining_steps=None,
            sg=SgStats(),
            final_status="",
            log_excerpt=[],
        )
    else:
        test1 = run_test(runner, param, "test1", ["1", "1", "1", "1", "1"], return_speed, command_timeout, homing_timeout, limit_position_tolerance, limit_settle_sec, no_line_probe_interval_sec)
        test2 = run_test(runner, param, "test2", ["5"], return_speed, command_timeout, homing_timeout, limit_position_tolerance, limit_settle_sec, no_line_probe_interval_sec)
    final_result = "PASS" if test1.result == "PASS" and test2.result == "PASS" else "FAIL"
    if final_result == "PASS":
        failure_reason = "OK"
        final_limit_state = test2.final_limit_state if test2.final_limit_state != "UNKNOWN" else test1.final_limit_state
        final_limit_timing = test2.final_limit_timing if test2.final_limit_timing != "UNKNOWN" else test1.final_limit_timing
        final_score = min(test1.score, test2.score)
        final_error = worst_error_mm(test1, test2)
        final_remaining = worst_remaining_steps(test1, test2)
    else:
        failed_test = first_failed_test(test1, test2)
        failure_reason = failed_test.failure_reason
        final_limit_state = failed_test.final_limit_state
        final_limit_timing = failed_test.final_limit_timing
        final_score = failed_test.score
        final_error = failed_test.final_error_mm
        final_remaining = failed_test.final_remaining_steps
    return SweepResult(
        timestamp=timestamp,
        param=param,
        test1=test1,
        test2=test2,
        final_result=final_result,
        failure_reason=failure_reason,
        final_limit_state=final_limit_state,
        final_limit_timing=final_limit_timing,
        final_score=final_score,
        final_error_mm=final_error,
        final_remaining_steps=final_remaining,
        sg=aggregate_sg_stats(test1.sg, test2.sg),
        elapsed_sec=time.monotonic() - start,
    )


def run_current_validate_only(
    runner: SerialRunner | SimulatedRunner,
    param: SweepParam,
    name: str,
    limit_position_tolerance: float,
) -> TestOutcome:
    log: list[str] = []
    setup_commands = [
        (f"microstep {param.microsteps}", SETUP_EXPECTS["microstep"]),
        (f"i {param.current_ma}", SETUP_EXPECTS["current"]),
        (f"mode {param.chop_mode}", SETUP_EXPECTS["mode"]),
        (f"v {fmt_num(param.speed_mm_s)}", SETUP_EXPECTS["speed"]),
    ]
    for command, expect in setup_commands:
        started = time.monotonic()
        ok, code, lines = runner.send_and_wait(command, expect, 5.0)
        elapsed_ms = elapsed_ms_since(started)
        log.extend(tag_lines(command, lines))
        if not ok:
            if code == "TO":
                diag = diagnose_timeout(runner, f"{name}:setup:{command.split()[0]}", command, elapsed_ms, lines, log)
                return outcome(name, "FAIL", "SETUP_TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
            return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)
    started = time.monotonic()
    ok, code, lines = runner.send_and_wait("tmcv", ("TMC validate", "sim:"), 5.0, reject_as_error=False)
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines("tmcv", lines))
    if not ok:
        if code == "TO":
            diag = diagnose_timeout(runner, f"{name}:setup:tmcv", "tmcv", elapsed_ms, lines, log)
            return outcome(name, "FAIL", "TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
        return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)
    started = time.monotonic()
    ok, code, lines = runner.send_and_wait("s", ("state=", "sim:"), 2.0, reject_as_error=False)
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines("s", lines))
    final_status = last_status(lines)
    timeout_diag = None
    if not ok and code == "TO":
        timeout_diag = diagnose_timeout(runner, f"{name}:status:validate", "s", elapsed_ms, lines, log)
        final_status = last_status(timeout_diag.status_lines)
    return TestOutcome(
        name=name,
        result="PASS" if ok else "FAIL",
        failure_reason="OK" if ok else code,
        final_limit_state=parse_limit_state(timeout_diag.status_lines if timeout_diag is not None else lines),
        final_limit_timing="VALIDATE_ONLY",
        score=100.0 if ok else 0.0,
        final_error_mm=final_error_mm(final_status, log),
        final_remaining_steps=final_remaining_steps(log),
        sg=parse_sg_stats(log),
        final_status=final_status,
        log_excerpt=trim_log(log),
        timeout_diag=timeout_diag,
    )


def run_test(
    runner: SerialRunner | SimulatedRunner,
    param: SweepParam,
    name: str,
    forward_commands: list[str],
    return_speed: float,
    command_timeout: float,
    homing_timeout: float,
    limit_position_tolerance: float,
    limit_settle_sec: float,
    no_line_probe_interval_sec: float,
) -> TestOutcome:
    log: list[str] = []
    probe_interval = no_line_probe_interval_sec if no_line_probe_interval_sec > 0.0 else None

    setup_commands = [
        (f"microstep {param.microsteps}", SETUP_EXPECTS["microstep"]),
        (f"i {param.current_ma}", SETUP_EXPECTS["current"]),
        (f"mode {param.chop_mode}", SETUP_EXPECTS["mode"]),
        ("profile trap", SETUP_EXPECTS["profile"]),
        (f"a {fmt_num(param.accel_mm_s2)}", SETUP_EXPECTS["accel"]),
    ]
    for command, expect in setup_commands:
        started = time.monotonic()
        ok, code, lines = runner.send_and_wait(command, expect, 5.0, phase=f"{name}:setup:{command.split()[0]}", no_line_probe_interval_sec=probe_interval)
        elapsed_ms = elapsed_ms_since(started)
        log.extend(tag_lines(command, lines))
        if not ok:
            if code == "TO":
                diag = diagnose_timeout(runner, f"{name}:setup:{command.split()[0]}", command, elapsed_ms, lines, log)
                return outcome(name, "FAIL", "SETUP_TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
            return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", "", log, limit_position_tolerance)

    started = time.monotonic()
    ok, code, lines = runner.send_and_wait("h", ("Homing complete",), homing_timeout, phase=f"{name}:homing", no_line_probe_interval_sec=probe_interval)
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines("h", lines))
    if not ok:
        if code == "TO":
            diag = diagnose_timeout(runner, f"{name}:homing", "h", elapsed_ms, lines, log)
            return outcome(name, "FAIL", "TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
        return outcome(name, "FAIL", "HE" if code == "HE" else code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    speed_command = f"v {fmt_num(param.speed_mm_s)}"
    started = time.monotonic()
    ok, code, lines = runner.send_and_wait(speed_command, SETUP_EXPECTS["speed"], 5.0, phase=f"{name}:setup:v", no_line_probe_interval_sec=probe_interval)
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines(speed_command, lines))
    if not ok:
        if code == "TO":
            diag = diagnose_timeout(runner, f"{name}:setup:v", speed_command, elapsed_ms, lines, log)
            return outcome(name, "FAIL", "SETUP_TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
        return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    sg_control_commands: list[tuple[str, tuple[str, ...]]] = []
    if param.sglog_enabled is not None:
        sg_control_commands.append((f"sglog {param.sglog_enabled}", ("SG log", "sim:")))
    if param.sg_enabled is not None:
        sg_control_commands.append((f"sgen {param.sg_enabled}", ("SG sampling", "sim:")))
    if param.sg_interval_ms is not None:
        sg_control_commands.append((f"sgint {param.sg_interval_ms}", ("SG sample interval set", "sim:")))

    for command, expect in sg_control_commands:
        started = time.monotonic()
        ok, code, lines = runner.send_and_wait(command, expect, 5.0, phase=f"{name}:setup:{command.split()[0]}", no_line_probe_interval_sec=probe_interval)
        elapsed_ms = elapsed_ms_since(started)
        log.extend(tag_lines(command, lines))
        if not ok:
            if code == "TO":
                diag = diagnose_timeout(runner, f"{name}:setup:{command.split()[0]}", command, elapsed_ms, lines, log)
                return outcome(name, "FAIL", "SETUP_TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
            return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    started = time.monotonic()
    ok, code, lines = runner.send_and_wait("sgreset", ("SG stats reset", "sim:"), 5.0, phase=f"{name}:setup:sgreset", no_line_probe_interval_sec=probe_interval)
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines("sgreset", lines))
    if not ok:
        if code == "TO":
            diag = diagnose_timeout(runner, f"{name}:setup:sgreset", "sgreset", elapsed_ms, lines, log)
            return outcome(name, "FAIL", "TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
        return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    for index, command in enumerate(forward_commands, start=1):
        started = time.monotonic()
        phase = f"{name}:move:{index}"
        ok, code, lines = runner.send_and_wait(command, ("Move complete",), command_timeout, phase=phase, no_line_probe_interval_sec=probe_interval)
        elapsed_ms = elapsed_ms_since(started)
        log.extend(tag_lines(command, lines))
        if not ok:
            if code == "TO":
                diag = diagnose_timeout(runner, f"{name}:move:{index}", command, elapsed_ms, lines, log)
                final_lines = diag.status_lines or lines
                return outcome(name, "FAIL", "TO", parse_limit_state(final_lines), "UNKNOWN", last_status(final_lines), log, limit_position_tolerance, diag)
            poll_motion_timing(runner, log)
            return outcome(name, "FAIL", code, parse_limit_state(lines), "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    poll_motion_timing(runner, log)

    return_speed_command = f"v {fmt_num(return_speed)}"
    started = time.monotonic()
    ok, code, lines = runner.send_and_wait(return_speed_command, SETUP_EXPECTS["speed"], 5.0, phase=f"{name}:setup:return_speed", no_line_probe_interval_sec=probe_interval)
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines(return_speed_command, lines))
    if not ok:
        if code == "TO":
            diag = diagnose_timeout(runner, f"{name}:setup:return_speed", return_speed_command, elapsed_ms, lines, log)
            return outcome(name, "FAIL", "SETUP_TO", parse_limit_state(diag.status_lines), "UNKNOWN", last_status(diag.status_lines), log, limit_position_tolerance, diag)
        return outcome(name, "FAIL", code, parse_limit_state(lines), "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    final_lines: list[str] = []
    for index in range(5):
        started = time.monotonic()
        phase = f"{name}:return:b:{index + 1}"
        ok, code, lines = runner.send_and_wait("b", ("Move complete",), command_timeout, phase=phase, no_line_probe_interval_sec=probe_interval)
        elapsed_ms = elapsed_ms_since(started)
        log.extend(tag_lines("b", lines))
        final_lines = lines
        if not ok:
            if code == "TO":
                diag = diagnose_timeout(runner, f"{name}:return:b:{index + 1}", "b", elapsed_ms, lines, log)
                final_lines = diag.status_lines or lines
                return outcome(name, "FAIL", "TO", parse_limit_state(final_lines), "UNKNOWN", last_status(final_lines), log, limit_position_tolerance, diag)
            poll_motion_timing(runner, log)
            if index == 4 and code == "ME":
                started = time.monotonic()
                status_code, status_lines = runner.status()
                elapsed_ms = elapsed_ms_since(started)
                log.extend(tag_lines("s", status_lines))
                final_lines = status_lines or final_lines
                final_status = last_status(final_lines)
                final_limit = parse_limit_state(final_lines)
                if status_code == "TO":
                    diag = diagnose_timeout(runner, f"{name}:status:after_motion_error", "s", elapsed_ms, status_lines, log)
                    final_lines = diag.status_lines or final_lines
                    return outcome(name, "FAIL", "TO", parse_limit_state(final_lines), "UNKNOWN", last_status(final_lines), log, limit_position_tolerance, diag)
                if final_limit == "ON":
                    if not final_position_within_tolerance(final_status, log, limit_position_tolerance):
                        return outcome(name, "FAIL", "LPOS", final_limit, "DURING_MOVE", final_status, log, limit_position_tolerance)
                    return outcome(name, "PASS", "OK", final_limit, "DURING_MOVE", final_status, log, limit_position_tolerance)
            early_limit = parse_limit_state(lines)
            early_status = last_status(lines)
            early_timing = "EARLY_LIMIT" if code == "ME" and early_limit == "ON" else "UNKNOWN"
            if early_timing == "EARLY_LIMIT" and final_position_within_tolerance(early_status, log, limit_position_tolerance):
                return outcome(name, "PASS", "OK", early_limit, early_timing, early_status, log, limit_position_tolerance)
            return outcome(name, "FAIL", code, early_limit, early_timing, early_status, log, limit_position_tolerance)

    started = time.monotonic()
    status_code, status_lines = runner.status()
    elapsed_ms = elapsed_ms_since(started)
    log.extend(tag_lines("s", status_lines))
    final_lines = status_lines or final_lines
    final_status = last_status(final_lines)
    final_limit = parse_limit_state(final_lines)
    if final_limit == "OFF" and parse_limit_raw(final_lines) == "ON" and final_position_within_tolerance(final_status, log, limit_position_tolerance):
        time.sleep(limit_settle_sec)
        settle_code, settle_lines = runner.status()
        log.extend(tag_lines("s", settle_lines))
        if settle_code != "TO":
            final_lines = settle_lines or final_lines
            final_status = last_status(final_lines)
            final_limit = parse_limit_state(final_lines)
            if final_limit == "ON" and final_position_within_tolerance(final_status, log, limit_position_tolerance):
                return outcome(name, "PASS", "OK", final_limit, "AFTER_SETTLE", final_status, log, limit_position_tolerance)
    if status_code == "TO":
        diag = diagnose_timeout(runner, f"{name}:status:final", "s", elapsed_ms, status_lines, log)
        final_lines = diag.status_lines or final_lines
        return outcome(name, "FAIL", "TO", parse_limit_state(final_lines), "UNKNOWN", last_status(final_lines), log, limit_position_tolerance, diag)
    if final_limit == "ON":
        if not final_position_within_tolerance(final_status, log, limit_position_tolerance):
            return outcome(name, "FAIL", "LPOS", final_limit, "AFTER_COMPLETE", final_status, log, limit_position_tolerance)
        return outcome(name, "PASS", "OK", final_limit, "AFTER_COMPLETE", final_status, log, limit_position_tolerance)
    if final_limit == "OFF":
        return outcome(name, "FAIL", "LOFF", final_limit, "NOT_REACHED", final_status, log, limit_position_tolerance)
    return outcome(name, "FAIL", "SL", "UNKNOWN", "UNKNOWN", final_status, log, limit_position_tolerance)


def outcome(
    name: str,
    result: str,
    failure_reason: str,
    final_limit_state: str,
    final_limit_timing: str,
    final_status: str,
    log: list[str],
    tolerance_mm: float = 0.5,
    timeout_diag: TimeoutDiagnostic | None = None,
) -> TestOutcome:
    error_mm = final_error_mm(final_status, log)
    remaining_steps = final_remaining_steps(log)
    return TestOutcome(
        name=name,
        result=result,
        failure_reason=failure_reason,
        final_limit_state=final_limit_state,
        final_limit_timing=final_limit_timing,
        score=score_result(result, final_limit_state, error_mm, tolerance_mm),
        final_error_mm=error_mm,
        final_remaining_steps=remaining_steps,
        sg=parse_sg_stats(log),
        final_status=final_status,
        log_excerpt=trim_log(log),
        timeout_diag=timeout_diag,
    )


def poll_motion_timing(runner: SerialRunner | SimulatedRunner, log: list[str]) -> None:
    ok, _code, lines = runner.send_and_wait("mt", ("MOTION_TIMING_SUMMARY", "sim:"), 1.0, reject_as_error=False)
    _ = ok
    log.extend(tag_lines("mt", lines))


def elapsed_ms_since(started: float) -> int:
    return int(round((time.monotonic() - started) * 1000.0))


def diagnose_timeout(
    runner: SerialRunner | SimulatedRunner,
    phase: str,
    command: str,
    elapsed_ms: int,
    timeout_lines: list[str],
    log: list[str],
) -> TimeoutDiagnostic:
    serial_tail = timeout_lines[-10:]
    diag_code, diag_lines = runner.diag_no_probe(1.0)
    status_code, status_lines = runner.status_no_probe(2.0)
    mt_code, mt_lines = runner.motion_timing_no_probe(2.0)
    diag = TimeoutDiagnostic(
        phase=phase,
        command=command,
        elapsed_ms=elapsed_ms,
        serial_tail=serial_tail,
        diag_code=diag_code,
        diag_lines=diag_lines,
        status_code=status_code,
        status_lines=status_lines,
        mt_code=mt_code,
        mt_lines=mt_lines,
    )
    log.append(
        "TO_DIAG,"
        f"phase={phase},command={command},elapsed_ms={elapsed_ms},"
        f"firmware_responsive={'yes' if diag.firmware_responsive else 'no'},"
        f"diag={diag_code},status={status_code},mt={mt_code}"
    )
    log.extend(tag_lines("TO serial tail", serial_tail))
    log.extend(tag_lines("diag probe", diag_lines))
    log.extend(tag_lines("s probe", status_lines))
    log.extend(tag_lines("mt probe", mt_lines))
    return diag


def parse_sg_stats(lines: list[str]) -> SgStats:
    stats = SgStats()
    for line in lines:
        parsed = parse_sg_key_values(line)
        if not parsed:
            continue
        apply_sg_key_values(stats, parsed)
    return stats


def parse_sg_key_values(line: str) -> dict[str, str]:
    text = line.strip()
    if not (
        text.startswith("TEST_SG_SUMMARY,")
        or text.startswith("MOTION_TIMING_SUMMARY,")
        or text.startswith("CURRENT_STATUS,")
        or text.startswith("VALIDATE_TMC,")
        or text.startswith("RE_DETAIL,")
        or text.startswith("SGLOG,")
        or text.startswith("SG_RESULT=")
    ):
        return {}

    values: dict[str, str] = {}
    for part in text.split(","):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key.strip().lower()] = value.strip()
    return values


def apply_sg_key_values(stats: SgStats, values: dict[str, str]) -> None:
    sg_min = optional_int(values.get("sg_min"))
    sg_max = optional_int(values.get("sg_max"))
    sg_avg = optional_float(values.get("sg_avg"))
    sg_count = optional_int(values.get("sg_count"))
    sg_min_valid = optional_int(values.get("sg_min_valid"))
    sg_max_valid = optional_int(values.get("sg_max_valid"))
    sg_avg_valid = optional_float(values.get("sg_avg_valid"))
    sg_valid_count = optional_int(values.get("sg_valid_count"))
    sg_zero_count = optional_int(values.get("sg_zero_count"))
    sg_low_count_50 = optional_int(values.get("sg_low_count_50"))
    sg_low_count_100 = optional_int(values.get("sg_low_count_100"))
    sg_low_count_150 = optional_int(values.get("sg_low_count_150"))
    sg_low_ratio_50 = optional_float(values.get("sg_low_ratio_50"))
    sg_low_ratio_100 = optional_float(values.get("sg_low_ratio_100"))
    sg_low_ratio_150 = optional_float(values.get("sg_low_ratio_150"))
    motion_update_count = optional_int(values.get("motion_update_count"))
    max_update_gap_us = optional_int(values.get("max_update_gap_us"))
    avg_update_gap_us = optional_float(values.get("avg_update_gap_us"))
    move_start_limit_state = optional_text(values.get("move_start_limit_state"))
    move_end_limit_state = optional_text(values.get("move_end_limit_state"))
    home_end_limit_state = optional_text(values.get("home_end_limit_state"))
    timeout_limit_state = optional_text(values.get("timeout_limit_state"))
    limit_transition_count = optional_int(values.get("limit_transition_count"))
    limit_first_trigger_timing = optional_text(values.get("limit_first_trigger_timing"))
    timeout_current_position = optional_float(values.get("timeout_current_position"))
    timeout_target_position = optional_float(values.get("timeout_target_position"))
    timeout_remaining_steps = optional_int(values.get("timeout_remaining_steps"))
    requested_current_ma = optional_int(values.get("requested_current_ma"))
    applied_current_ma = optional_int(values.get("applied_current_ma"))
    tmc_uart_ok = optional_text(values.get("tmc_uart_ok"))
    config_tmc_uart_ok = optional_text(values.get("config_tmc_uart_ok"))
    driver_status = optional_text(values.get("driver_status"))
    validate_tmc_uart_ok = optional_text(values.get("validate_tmc_uart_ok"))
    validate_driver_status = optional_text(values.get("validate_driver_status"))
    validate_ifcnt_before = optional_int(values.get("validate_ifcnt_before"))
    validate_ifcnt_after = optional_int(values.get("validate_ifcnt_after"))
    validate_ifcnt_delta = optional_int(values.get("validate_ifcnt_delta"))
    validate_gstat = optional_int(values.get("validate_gstat"))
    validate_drv_status = optional_int(values.get("validate_drv_status"))
    validate_requested_current_ma = optional_int(values.get("validate_requested_current_ma"))
    validate_applied_current_ma = optional_int(values.get("validate_applied_current_ma"))
    validate_current_error_ma = optional_int(values.get("validate_current_error_ma"))
    validate_current_error_ratio = optional_float(values.get("validate_current_error_ratio"))
    validate_current_tolerance_ma = optional_int(values.get("validate_current_tolerance_ma"))
    validate_fail_reason_detail = optional_text(values.get("validate_fail_reason_detail"))
    re_stage = optional_text(values.get("re_stage"))
    re_reason = optional_text(values.get("re_reason"))
    current_before_re = optional_int(values.get("current_before_re"))
    position_before_re = optional_float(values.get("position_before_re"))
    limit_state_before_re = optional_text(values.get("limit_state_before_re"))
    motion_state_before_re = optional_text(values.get("motion_state_before_re"))
    sgthrs = optional_int(values.get("sgthrs"))
    tcoolthrs = optional_int(values.get("tcoolthrs"))
    diag_triggered = optional_diag_text(values.get("diag_triggered"))

    if sg_min is not None:
        stats.sg_min = sg_min
    if sg_max is not None:
        stats.sg_max = sg_max
    if sg_avg is not None:
        stats.sg_avg = sg_avg
    if sg_count is not None:
        stats.sg_count = sg_count
    if sg_min_valid is not None:
        stats.sg_min_valid = sg_min_valid
    if sg_max_valid is not None:
        stats.sg_max_valid = sg_max_valid
    if sg_avg_valid is not None:
        stats.sg_avg_valid = sg_avg_valid
    if sg_valid_count is not None:
        stats.sg_valid_count = sg_valid_count
    if sg_zero_count is not None:
        stats.sg_zero_count = sg_zero_count
    if sg_low_count_50 is not None:
        stats.sg_low_count_50 = sg_low_count_50
    if sg_low_count_100 is not None:
        stats.sg_low_count_100 = sg_low_count_100
    if sg_low_count_150 is not None:
        stats.sg_low_count_150 = sg_low_count_150
    if sg_low_ratio_50 is not None:
        stats.sg_low_ratio_50 = sg_low_ratio_50
    if sg_low_ratio_100 is not None:
        stats.sg_low_ratio_100 = sg_low_ratio_100
    if sg_low_ratio_150 is not None:
        stats.sg_low_ratio_150 = sg_low_ratio_150
    if motion_update_count is not None:
        stats.motion_update_count = motion_update_count
    if max_update_gap_us is not None:
        stats.max_update_gap_us = max_update_gap_us
    if avg_update_gap_us is not None:
        stats.avg_update_gap_us = avg_update_gap_us
    if move_start_limit_state is not None:
        stats.move_start_limit_state = move_start_limit_state
    if move_end_limit_state is not None:
        stats.move_end_limit_state = move_end_limit_state
    if home_end_limit_state is not None:
        stats.home_end_limit_state = home_end_limit_state
    if timeout_limit_state is not None:
        stats.timeout_limit_state = timeout_limit_state
    if limit_transition_count is not None:
        stats.limit_transition_count = limit_transition_count
    if limit_first_trigger_timing is not None:
        stats.limit_first_trigger_timing = limit_first_trigger_timing
    if timeout_current_position is not None:
        stats.timeout_current_position = timeout_current_position
    if timeout_target_position is not None:
        stats.timeout_target_position = timeout_target_position
    if timeout_remaining_steps is not None:
        stats.timeout_remaining_steps = timeout_remaining_steps
    if requested_current_ma is not None:
        stats.requested_current_ma = requested_current_ma
    if applied_current_ma is not None:
        stats.applied_current_ma = applied_current_ma
    if tmc_uart_ok is not None:
        stats.tmc_uart_ok = tmc_uart_ok
    if config_tmc_uart_ok is not None:
        stats.config_tmc_uart_ok = config_tmc_uart_ok
    if driver_status is not None:
        stats.driver_status = driver_status
    if validate_tmc_uart_ok is not None:
        stats.validate_tmc_uart_ok = validate_tmc_uart_ok
    if validate_driver_status is not None:
        stats.validate_driver_status = validate_driver_status
    if validate_ifcnt_before is not None:
        stats.validate_ifcnt_before = validate_ifcnt_before
    if validate_ifcnt_after is not None:
        stats.validate_ifcnt_after = validate_ifcnt_after
    if validate_ifcnt_delta is not None:
        stats.validate_ifcnt_delta = validate_ifcnt_delta
    if validate_gstat is not None:
        stats.validate_gstat = validate_gstat
    if validate_drv_status is not None:
        stats.validate_drv_status = validate_drv_status
    if validate_requested_current_ma is not None:
        stats.validate_requested_current_ma = validate_requested_current_ma
    if validate_applied_current_ma is not None:
        stats.validate_applied_current_ma = validate_applied_current_ma
    if validate_current_error_ma is not None:
        stats.validate_current_error_ma = validate_current_error_ma
    if validate_current_error_ratio is not None:
        stats.validate_current_error_ratio = validate_current_error_ratio
    if validate_current_tolerance_ma is not None:
        stats.validate_current_tolerance_ma = validate_current_tolerance_ma
    if validate_fail_reason_detail is not None:
        stats.validate_fail_reason_detail = validate_fail_reason_detail
    if re_stage is not None:
        stats.re_stage = re_stage
    if re_reason is not None:
        stats.re_reason = re_reason
    if current_before_re is not None:
        stats.current_before_re = current_before_re
    if position_before_re is not None:
        stats.position_before_re = position_before_re
    if limit_state_before_re is not None:
        stats.limit_state_before_re = limit_state_before_re
    if motion_state_before_re is not None:
        stats.motion_state_before_re = motion_state_before_re
    if sgthrs is not None:
        stats.sgthrs = sgthrs
    if tcoolthrs is not None:
        stats.tcoolthrs = tcoolthrs
    if diag_triggered is not None:
        stats.diag_triggered = diag_triggered


def optional_int(value: str | None) -> int | None:
    text = optional_text(value)
    if text is None:
        return None
    return int(text)


def optional_float(value: str | None) -> float | None:
    text = optional_text(value)
    if text is None:
        return None
    return float(text)


def optional_text(value: str | None) -> str | None:
    if value is None:
        return None
    text = value.strip()
    if not text or text.upper() == "NA":
        return None
    return text


def optional_diag_text(value: str | None) -> str | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    return text.upper() if text.upper() == "NA" else text


def aggregate_sg_stats(*items: SgStats) -> SgStats:
    sample_counts = [item.sg_count for item in items if item.sg_count is not None]
    total_count = sum(sample_counts) if sample_counts else None
    valid_counts = [item.sg_valid_count for item in items if item.sg_valid_count is not None]
    total_valid_count = sum(valid_counts) if valid_counts else None
    zero_counts = [item.sg_zero_count for item in items if item.sg_zero_count is not None]
    total_zero_count = sum(zero_counts) if zero_counts else None
    low_counts_50 = [item.sg_low_count_50 for item in items if item.sg_low_count_50 is not None]
    total_low_count_50 = sum(low_counts_50) if low_counts_50 else None
    low_counts_100 = [item.sg_low_count_100 for item in items if item.sg_low_count_100 is not None]
    total_low_count_100 = sum(low_counts_100) if low_counts_100 else None
    low_counts_150 = [item.sg_low_count_150 for item in items if item.sg_low_count_150 is not None]
    total_low_count_150 = sum(low_counts_150) if low_counts_150 else None
    motion_counts = [item.motion_update_count for item in items if item.motion_update_count is not None]
    total_motion_count = sum(motion_counts) if motion_counts else None
    max_gaps = [item.max_update_gap_us for item in items if item.max_update_gap_us is not None]
    avg_update_gap = weighted_motion_gap_avg(items)
    weighted_avg = weighted_sg_avg(items)
    diag_values = [item.diag_triggered for item in items if item.diag_triggered not in (None, "NA")]

    if any(value == "1" for value in diag_values):
        diag_triggered = "1"
    elif any(value == "0" for value in diag_values):
        diag_triggered = "0"
    else:
        diag_triggered = "NA" if any(item.diag_triggered == "NA" for item in items) else None

    valid_mins = [item.sg_min_valid for item in items if item.sg_min_valid is not None]
    valid_maxes = [item.sg_max_valid for item in items if item.sg_max_valid is not None]
    return SgStats(
        sg_min=min(valid_mins) if valid_mins else None,
        sg_max=max(valid_maxes) if valid_maxes else None,
        sg_avg=weighted_avg,
        sg_count=total_count,
        sg_min_valid=min(valid_mins) if valid_mins else None,
        sg_max_valid=max(valid_maxes) if valid_maxes else None,
        sg_avg_valid=weighted_avg,
        sg_valid_count=total_valid_count,
        sg_zero_count=total_zero_count,
        sg_low_count_50=total_low_count_50,
        sg_low_count_100=total_low_count_100,
        sg_low_count_150=total_low_count_150,
        sg_low_ratio_50=ratio_or_none(total_low_count_50, total_valid_count),
        sg_low_ratio_100=ratio_or_none(total_low_count_100, total_valid_count),
        sg_low_ratio_150=ratio_or_none(total_low_count_150, total_valid_count),
        motion_update_count=total_motion_count,
        max_update_gap_us=max(max_gaps) if max_gaps else None,
        avg_update_gap_us=avg_update_gap,
        move_start_limit_state=first_available_text([item.move_start_limit_state for item in items]),
        move_end_limit_state=last_available_text([item.move_end_limit_state for item in items]),
        home_end_limit_state=last_available_text([item.home_end_limit_state for item in items]),
        timeout_limit_state=last_available_text([item.timeout_limit_state for item in items]),
        limit_transition_count=sum(item.limit_transition_count for item in items if item.limit_transition_count is not None)
        if any(item.limit_transition_count is not None for item in items)
        else None,
        limit_first_trigger_timing=first_non_none_trigger([item.limit_first_trigger_timing for item in items]),
        timeout_current_position=last_available_float([item.timeout_current_position for item in items]),
        timeout_target_position=last_available_float([item.timeout_target_position for item in items]),
        timeout_remaining_steps=last_available_int([item.timeout_remaining_steps for item in items]),
        requested_current_ma=last_available_int([item.requested_current_ma for item in items]),
        applied_current_ma=last_available_int([item.applied_current_ma for item in items]),
        tmc_uart_ok=last_available_text([item.tmc_uart_ok for item in items]),
        config_tmc_uart_ok=last_available_text([item.config_tmc_uart_ok for item in items]),
        driver_status=last_available_text([item.driver_status for item in items]),
        validate_tmc_uart_ok=last_available_text([item.validate_tmc_uart_ok for item in items]),
        validate_driver_status=last_available_text([item.validate_driver_status for item in items]),
        validate_ifcnt_before=last_available_int([item.validate_ifcnt_before for item in items]),
        validate_ifcnt_after=last_available_int([item.validate_ifcnt_after for item in items]),
        validate_ifcnt_delta=last_available_int([item.validate_ifcnt_delta for item in items]),
        validate_gstat=last_available_int([item.validate_gstat for item in items]),
        validate_drv_status=last_available_int([item.validate_drv_status for item in items]),
        validate_requested_current_ma=last_available_int([item.validate_requested_current_ma for item in items]),
        validate_applied_current_ma=last_available_int([item.validate_applied_current_ma for item in items]),
        validate_current_error_ma=last_available_int([item.validate_current_error_ma for item in items]),
        validate_current_error_ratio=last_available_float([item.validate_current_error_ratio for item in items]),
        validate_current_tolerance_ma=last_available_int([item.validate_current_tolerance_ma for item in items]),
        validate_fail_reason_detail=last_available_text([item.validate_fail_reason_detail for item in items]),
        re_stage=last_available_text([item.re_stage for item in items]),
        re_reason=last_available_text([item.re_reason for item in items]),
        current_before_re=last_available_int([item.current_before_re for item in items]),
        position_before_re=last_available_float([item.position_before_re for item in items]),
        limit_state_before_re=last_available_text([item.limit_state_before_re for item in items]),
        motion_state_before_re=last_available_text([item.motion_state_before_re for item in items]),
        diag_triggered=diag_triggered,
        sgthrs=last_available_int([item.sgthrs for item in items]),
        tcoolthrs=last_available_int([item.tcoolthrs for item in items]),
    )


def weighted_sg_avg(items: tuple[SgStats, ...]) -> float | None:
    weighted_sum = 0.0
    total_count = 0
    fallback: float | None = None
    for item in items:
        avg = item.sg_avg_valid if item.sg_avg_valid is not None else item.sg_avg
        if avg is None:
            continue
        fallback = avg
        count = item.sg_valid_count if item.sg_valid_count is not None else item.sg_count
        if count is not None and count > 0:
            weighted_sum += avg * count
            total_count += count
    if total_count > 0:
        return weighted_sum / total_count
    return fallback


def ratio_or_none(count: int | None, total: int | None) -> float | None:
    if count is None or total is None or total <= 0:
        return None
    return count / total


def weighted_motion_gap_avg(items: tuple[SgStats, ...]) -> float | None:
    weighted_sum = 0.0
    total_weight = 0
    fallback: float | None = None
    for item in items:
        if item.avg_update_gap_us is None:
            continue
        fallback = item.avg_update_gap_us
        if item.motion_update_count is not None and item.motion_update_count > 1:
            weight = item.motion_update_count - 1
            weighted_sum += item.avg_update_gap_us * weight
            total_weight += weight
    if total_weight > 0:
        return weighted_sum / total_weight
    return fallback


def last_available_int(values: list[int | None]) -> int | None:
    for value in reversed(values):
        if value is not None:
            return value
    return None


def last_available_float(values: list[float | None]) -> float | None:
    for value in reversed(values):
        if value is not None:
            return value
    return None


def first_available_text(values: list[str | None]) -> str | None:
    for value in values:
        if value is not None:
            return value
    return None


def last_available_text(values: list[str | None]) -> str | None:
    for value in reversed(values):
        if value is not None:
            return value
    return None


def first_non_none_trigger(values: list[str | None]) -> str | None:
    for value in values:
        if value not in (None, "NONE"):
            return value
    return "NONE" if any(value == "NONE" for value in values) else None


def first_failure_code(test1: TestOutcome, test2: TestOutcome) -> str:
    for test in (test1, test2):
        if test.result != "PASS":
            return test.failure_reason
    return "SL"


def first_failed_test(test1: TestOutcome, test2: TestOutcome) -> TestOutcome:
    if test1.result != "PASS":
        return test1
    if test2.result != "PASS":
        return test2
    return test1


def tag_lines(command: str, lines: list[str]) -> list[str]:
    return [f"$ {command}"] + lines


def trim_log(lines: list[str], keep: int = 20) -> list[str]:
    if len(lines) <= keep:
        return lines
    return lines[:5] + ["..."] + lines[-(keep - 6) :]


def last_status(lines: list[str]) -> str:
    for line in reversed(lines):
        if line.startswith("state="):
            return line
    return lines[-1] if lines else ""


def parse_limit_state(lines: list[str]) -> str:
    for line in reversed(lines):
        match = re.search(r"limitDebounced=(ON|OFF)", line)
        if match:
            return match.group(1)
    return "UNKNOWN"


def parse_limit_raw(lines: list[str]) -> str:
    for line in reversed(lines):
        match = re.search(r"limitRaw=(ON|OFF)", line)
        if match:
            return match.group(1)
    return "UNKNOWN"


def final_position_within_tolerance(status_line: str, log: list[str], tolerance_mm: float) -> bool:
    error = final_error_mm(status_line, log)
    if error is None:
        return False
    return error <= tolerance_mm


def final_error_mm(status_line: str, log: list[str]) -> float | None:
    detail = last_motion_error_detail(log)
    if detail is not None:
        return abs(detail["pos"])
    return position_error_mm(status_line)


def position_error_mm(status_line: str) -> float | None:
    match = re.search(r"pos=(-?[0-9]+(?:\.[0-9]+)?)mm", status_line)
    if not match:
        return None
    return abs(float(match.group(1)))


def final_remaining_steps(log: list[str]) -> int | None:
    detail = last_motion_error_detail(log)
    if detail is None:
        return None
    return int(detail["remainingSteps"])


def last_motion_error_detail(log: list[str]) -> dict[str, float] | None:
    pattern = re.compile(
        r"Motion error detail: pos=(-?[0-9]+(?:\.[0-9]+)?)mm steps=(-?\d+) "
        r"target=(-?[0-9]+(?:\.[0-9]+)?)mm targetSteps=(-?\d+) remainingSteps=(\d+)"
    )
    for line in reversed(log):
        match = pattern.search(line)
        if match:
            return {
                "pos": float(match.group(1)),
                "steps": float(match.group(2)),
                "target": float(match.group(3)),
                "targetSteps": float(match.group(4)),
                "remainingSteps": float(match.group(5)),
            }
    return None


def score_result(result: str, final_limit_state: str, error_mm: float | None, tolerance_mm: float) -> float:
    if result != "PASS" or final_limit_state != "ON" or error_mm is None:
        return 0.0
    if tolerance_mm <= 0.0:
        return 100.0 if error_mm == 0.0 else 0.0
    return max(0.0, min(100.0, 100.0 * (1.0 - error_mm / tolerance_mm)))


def worst_error_mm(test1: TestOutcome, test2: TestOutcome) -> float | None:
    values = [value for value in (test1.final_error_mm, test2.final_error_mm) if value is not None]
    return max(values) if values else None


def worst_remaining_steps(test1: TestOutcome, test2: TestOutcome) -> int | None:
    values = [value for value in (test1.final_remaining_steps, test2.final_remaining_steps) if value is not None]
    return max(values) if values else None


def format_optional_mm(value: float | None) -> str:
    return "-" if value is None else f"{value:.4f}mm"


def format_optional_int(value: int | None) -> str:
    return "-" if value is None else str(value)


def format_test_outcome(test: TestOutcome) -> str:
    text = (
        f"{test.result} score={test.score:.1f} reason={test.failure_reason} "
        f"limit={test.final_limit_state} timing={test.final_limit_timing} "
        f"error={format_optional_mm(test.final_error_mm)} "
        f"remainingSteps={format_optional_int(test.final_remaining_steps)}"
    )
    if is_timeout_reason(test.failure_reason) and test.timeout_diag is not None:
        diag = test.timeout_diag
        text += (
            f" phase={diag.phase} command={diag.command} "
            f"elapsed_ms={diag.elapsed_ms} "
            f"firmware={'yes' if diag.firmware_responsive else 'no'} "
            f"diag={diag.diag_code} status={diag.status_code} mt={diag.mt_code}"
        )
    return text


def is_timeout_reason(reason: str) -> bool:
    return reason in ("TO", "SETUP_TO")


def write_results_csv(path: Path, results: list[SweepResult]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=RESULT_COLUMNS)
        writer.writeheader()
        for result in results:
            writer.writerow(result_csv_row(result))


def run_plot_report(
    result_csv_path: Path,
    reports_dir: Path,
    timestamp: str,
    show_cell_labels: bool,
    error_scale: str,
    figure_format: str,
) -> tuple[Path, bool, str]:
    plot_dir = reports_dir / f"step_loss_sweep_{timestamp}"
    script_path = Path(__file__).resolve().parent / "plot_step_loss_sweep.py"
    command = [
        sys.executable,
        str(script_path),
        "--csv",
        str(result_csv_path),
        "--out",
        str(plot_dir),
        "--error-scale",
        error_scale,
        "--format",
        figure_format,
    ]
    if show_cell_labels:
        command.append("--show-cell-labels")

    completed = subprocess.run(command, text=True, capture_output=True)
    output = "\n".join(part for part in (completed.stdout.strip(), completed.stderr.strip()) if part)
    return plot_dir, completed.returncode == 0, output


def append_plot_report_section(report_path: Path, plot_dir: Path, plot_ok: bool, plot_output: str, figure_format: str) -> None:
    lines = ["", "## Plot Report", ""]
    if plot_ok:
        report_rel = relative_link(report_path.parent, plot_dir / "report.md")
        lines.append(f"- Graph report: [{report_rel}]({report_rel})")
        for filename, label in [
            (f"01_overview_scatter.{figure_format}", "Overview Scatter"),
            (f"02_error_heatmap_by_current.{figure_format}", "Error Heatmap by Current"),
            (f"03_result_heatmap_by_current.{figure_format}", "Result Heatmap by Current"),
            (f"04_max_stable_speed_by_current.{figure_format}", "Max Stable Speed by Current"),
            (f"05_max_stable_speed_by_accel.{figure_format}", "Max Stable Speed by Accel"),
            (f"06_error_vs_speed_by_current.{figure_format}", "Error vs Speed by Current"),
        ]:
            image_path = plot_dir / "figures" / filename
            image_rel = relative_link(report_path.parent, image_path)
            lines.append("")
            lines.append(f"![{label}]({image_rel})")
    else:
        lines.append("Plot report was not generated.")
        lines.append("")
        lines.append("```text")
        lines.append(plot_output or "No output from plot_step_loss_sweep.py")
        lines.append("```")

    with report_path.open("a") as handle:
        handle.write("\n".join(lines) + "\n")


def relative_link(base_dir: Path, target: Path) -> str:
    return os.path.relpath(target, start=base_dir)


def result_csv_row(result: SweepResult) -> dict[str, str | int | float]:
    p = result.param
    sg = result.sg
    missing = "NA"
    sg_stats_disabled = p.sg_enabled == 0
    failure_detail, failure_stage, failure_evidence = classify_failure(result)
    diagnostic = sg_diagnostic(result)
    motion_detail = result_motion_error_detail(result)
    timeout_diag = first_failed_test(result.test1, result.test2).timeout_diag if is_timeout_reason(result.failure_reason) else None
    timeout_stage = timeout_diag.phase if timeout_diag is not None else (failure_stage if is_timeout_reason(result.failure_reason) else None)
    current_error = current_error_ma(result)
    current_ratio = current_error_ratio(result)
    current_tolerance = current_tolerance_ma(result)
    return {
        "timestamp": result.timestamp,
        "speed_mm_s": fmt_num(p.speed_mm_s),
        "accel_mm_s2": fmt_num(p.accel_mm_s2),
        "current_ma": p.current_ma,
        "chop_mode": p.chop_mode,
        "microsteps": p.microsteps,
        "test1_result": result.test1.result,
        "test2_result": result.test2.result,
        "final_result": result.final_result,
        "failure_reason": result.failure_reason,
        "final_limit_state": result.final_limit_state,
        "final_limit_timing": result.final_limit_timing,
        "final_score": f"{result.final_score:.1f}",
        "final_error_mm": "" if result.final_error_mm is None else f"{result.final_error_mm:.4f}",
        "final_remaining_steps": "" if result.final_remaining_steps is None else result.final_remaining_steps,
        "elapsed_sec": f"{result.elapsed_sec:.3f}",
        "sg_min": missing if sg_stats_disabled or sg.sg_min is None else sg.sg_min,
        "sg_max": missing if sg_stats_disabled or sg.sg_max is None else sg.sg_max,
        "sg_avg": missing if sg_stats_disabled or sg.sg_avg is None else f"{sg.sg_avg:.1f}",
        "sg_count": missing if sg_stats_disabled or sg.sg_count is None else sg.sg_count,
        "sg_min_valid": missing if sg_stats_disabled or sg.sg_min_valid is None else sg.sg_min_valid,
        "sg_max_valid": missing if sg_stats_disabled or sg.sg_max_valid is None else sg.sg_max_valid,
        "sg_avg_valid": missing if sg_stats_disabled or sg.sg_avg_valid is None else f"{sg.sg_avg_valid:.1f}",
        "sg_valid_count": missing if sg_stats_disabled or sg.sg_valid_count is None else sg.sg_valid_count,
        "sg_zero_count": missing if sg_stats_disabled or sg.sg_zero_count is None else sg.sg_zero_count,
        "sg_low_count_50": missing if sg_stats_disabled or sg.sg_low_count_50 is None else sg.sg_low_count_50,
        "sg_low_count_100": missing if sg_stats_disabled or sg.sg_low_count_100 is None else sg.sg_low_count_100,
        "sg_low_count_150": missing if sg_stats_disabled or sg.sg_low_count_150 is None else sg.sg_low_count_150,
        "sg_low_ratio_50": missing if sg_stats_disabled or sg.sg_low_ratio_50 is None else f"{sg.sg_low_ratio_50:.4f}",
        "sg_low_ratio_100": missing if sg_stats_disabled or sg.sg_low_ratio_100 is None else f"{sg.sg_low_ratio_100:.4f}",
        "sg_low_ratio_150": missing if sg_stats_disabled or sg.sg_low_ratio_150 is None else f"{sg.sg_low_ratio_150:.4f}",
        "motion_update_count": missing if sg.motion_update_count is None else sg.motion_update_count,
        "max_update_gap_us": missing if sg.max_update_gap_us is None else sg.max_update_gap_us,
        "avg_update_gap_us": missing if sg.avg_update_gap_us is None else f"{sg.avg_update_gap_us:.1f}",
        "diag_triggered": missing if sg.diag_triggered is None else sg.diag_triggered,
        "sgthrs": missing if sg.sgthrs is None else sg.sgthrs,
        "tcoolthrs": missing if sg.tcoolthrs is None else sg.tcoolthrs,
        "test_condition_id": p.test_condition_id,
        "repeat_index": missing if p.repeat_index is None else p.repeat_index,
        "sg_enabled": missing if p.sg_enabled is None else p.sg_enabled,
        "sg_interval_ms": missing if p.sg_interval_ms is None else p.sg_interval_ms,
        "sglog_enabled": missing if p.sglog_enabled is None else p.sglog_enabled,
        "move_start_limit_state": missing if sg.move_start_limit_state is None else sg.move_start_limit_state,
        "move_end_limit_state": missing if sg.move_end_limit_state is None else sg.move_end_limit_state,
        "home_end_limit_state": missing if sg.home_end_limit_state is None else sg.home_end_limit_state,
        "timeout_limit_state": missing if sg.timeout_limit_state is None else sg.timeout_limit_state,
        "limit_transition_count": missing if sg.limit_transition_count is None else sg.limit_transition_count,
        "limit_first_trigger_timing": missing if sg.limit_first_trigger_timing is None else sg.limit_first_trigger_timing,
        "timeout_current_position": missing if sg.timeout_current_position is None else f"{sg.timeout_current_position:.4f}",
        "timeout_target_position": missing if sg.timeout_target_position is None else f"{sg.timeout_target_position:.4f}",
        "timeout_remaining_steps": missing if sg.timeout_remaining_steps is None else sg.timeout_remaining_steps,
        "requested_current_ma": missing if sg.requested_current_ma is None else sg.requested_current_ma,
        "applied_current_ma": missing if sg.applied_current_ma is None else sg.applied_current_ma,
        "tmc_uart_ok": missing if sg.tmc_uart_ok is None else sg.tmc_uart_ok,
        "config_tmc_uart_ok": missing if sg.config_tmc_uart_ok is None else sg.config_tmc_uart_ok,
        "driver_status": missing if sg.driver_status is None else sg.driver_status,
        "validate_tmc_uart_ok": missing if sg.validate_tmc_uart_ok is None else sg.validate_tmc_uart_ok,
        "validate_driver_status": missing if sg.validate_driver_status is None else sg.validate_driver_status,
        "validate_ifcnt_before": missing if sg.validate_ifcnt_before is None else sg.validate_ifcnt_before,
        "validate_ifcnt_after": missing if sg.validate_ifcnt_after is None else sg.validate_ifcnt_after,
        "validate_ifcnt_delta": missing if sg.validate_ifcnt_delta is None else sg.validate_ifcnt_delta,
        "validate_gstat": missing if sg.validate_gstat is None else sg.validate_gstat,
        "validate_drv_status": missing if sg.validate_drv_status is None else sg.validate_drv_status,
        "validate_requested_current_ma": missing if sg.validate_requested_current_ma is None else sg.validate_requested_current_ma,
        "validate_applied_current_ma": missing if sg.validate_applied_current_ma is None else sg.validate_applied_current_ma,
        "validate_current_error_ma": missing if sg.validate_current_error_ma is None else sg.validate_current_error_ma,
        "validate_current_error_ratio": missing if sg.validate_current_error_ratio is None else f"{sg.validate_current_error_ratio:.4f}",
        "validate_current_tolerance_ma": missing if sg.validate_current_tolerance_ma is None else sg.validate_current_tolerance_ma,
        "validate_fail_reason_detail": missing if sg.validate_fail_reason_detail is None else sg.validate_fail_reason_detail,
        "re_stage": missing if sg.re_stage is None else sg.re_stage,
        "re_reason": missing if sg.re_reason is None else sg.re_reason,
        "current_before_re": missing if sg.current_before_re is None else sg.current_before_re,
        "position_before_re": missing if sg.position_before_re is None else f"{sg.position_before_re:.4f}",
        "limit_state_before_re": missing if sg.limit_state_before_re is None else sg.limit_state_before_re,
        "motion_state_before_re": missing if sg.motion_state_before_re is None else sg.motion_state_before_re,
        "failure_detail": failure_detail,
        "failure_stage": failure_stage,
        "failure_evidence": failure_evidence,
        "timeout_stage": missing if timeout_stage is None else timeout_stage,
        "timeout_elapsed_ms": missing if not is_timeout_reason(result.failure_reason) else (str(timeout_diag.elapsed_ms) if timeout_diag is not None else f"{result.elapsed_sec * 1000.0:.0f}"),
        "timeout_motion_state": missing if not is_timeout_reason(result.failure_reason) else (sg.motion_state_before_re or "NA"),
        "timeout_last_step_time_ms": missing,
        "timeout_expected_duration_ms": timeout_expected_duration_ms_text(result),
        "timeout_phase": missing if timeout_diag is None else timeout_diag.phase,
        "timeout_command": missing if timeout_diag is None else timeout_diag.command,
        "timeout_firmware_responsive": missing if timeout_diag is None else ("yes" if timeout_diag.firmware_responsive else "no"),
        "timeout_diag_probe": missing if timeout_diag is None else timeout_diag.diag_code,
        "timeout_status_probe": missing if timeout_diag is None else timeout_diag.status_code,
        "timeout_mt_probe": missing if timeout_diag is None else timeout_diag.mt_code,
        "expected_position": missing if motion_detail is None else f"{motion_detail['target']:.4f}",
        "actual_position": missing if motion_detail is None else f"{motion_detail['pos']:.4f}",
        "abs_error_mm": missing if result.final_error_mm is None else f"{abs(result.final_error_mm):.4f}",
        "error_threshold_mm": "0.5000",
        "remaining_steps_threshold": "0",
        "limit_expected_state": limit_expected_state(result),
        "limit_actual_state": result.final_limit_state,
        "limit_failure_detail": limit_failure_detail(result),
        "current_error_ma": missing if current_error is None else current_error,
        "current_error_ratio": missing if current_ratio is None else f"{current_ratio:.4f}",
        "current_tolerance_ma": missing if current_tolerance is None else current_tolerance,
        "sg_health_state": diagnostic["health"],
        "diagnostic_tags": diagnostic["tags"],
        "diagnostic_comment": diagnostic["comment"],
    }


def classify_failure(result: SweepResult) -> tuple[str, str, str]:
    if result.final_result == "PASS":
        return "OK", "complete", "existing_logic_pass"

    failed = first_failed_test(result.test1, result.test2)
    stage = failed.timeout_diag.phase if is_timeout_reason(result.failure_reason) and failed.timeout_diag is not None else failed.name
    evidence_parts = [
        f"reason={result.failure_reason}",
        f"limit={result.final_limit_state}",
        f"timing={result.final_limit_timing}",
    ]
    if failed.timeout_diag is not None:
        evidence_parts.extend(
            [
                f"phase={failed.timeout_diag.phase}",
                f"command={failed.timeout_diag.command}",
                f"firmware_responsive={'yes' if failed.timeout_diag.firmware_responsive else 'no'}",
                f"diag_probe={failed.timeout_diag.diag_code}",
                f"status_probe={failed.timeout_diag.status_code}",
                f"mt_probe={failed.timeout_diag.mt_code}",
            ]
        )
    if result.final_error_mm is not None:
        evidence_parts.append(f"error_mm={result.final_error_mm:.4f}")
    if result.final_remaining_steps is not None:
        evidence_parts.append(f"remaining_steps={result.final_remaining_steps}")

    if result.failure_reason == "SETUP_TO":
        detail = "SETUP_EXPECT_MISMATCH_OR_LOG_LOSS"
    elif result.failure_reason == "TO":
        detail = timeout_failure_detail(result)
    elif result.failure_reason == "ME":
        detail = "ME_POSITION_ERROR"
    elif result.failure_reason == "LOFF":
        detail = "LIMIT_NOT_REACHED"
    elif result.failure_reason == "LPOS":
        detail = "ME_POSITION_ERROR"
    elif result.failure_reason == "RE":
        detail = re_failure_detail(result)
    elif result.failure_reason == "HE":
        detail = "TO_HOME"
        stage = "home" if failed.timeout_diag is None else failed.timeout_diag.phase
    else:
        detail = "UNKNOWN"
    return detail, stage, ";".join(evidence_parts)


def timeout_failure_detail(result: SweepResult) -> str:
    sg = result.sg
    failed = first_failed_test(result.test1, result.test2)
    if failed.timeout_diag is not None and ":homing" in failed.timeout_diag.phase:
        return "TO_HOME"
    if sg.timeout_remaining_steps == 0:
        return "TO_COMPLETION_DETECTION_MISMATCH"
    if sg.timeout_current_position is not None and sg.timeout_target_position is not None:
        if abs(sg.timeout_current_position - sg.timeout_target_position) <= 0.5:
            return "TO_COMPLETION_DETECTION_MISMATCH"
    if failed.timeout_diag is not None and ":return:" in failed.timeout_diag.phase:
        return "TO_RETURN"
    if failed.name == "test1":
        return "TO_FORWARD"
    if failed.name == "test2":
        return "TO_RETURN"
    return "TO_FORWARD"


def re_failure_detail(result: SweepResult) -> str:
    sg = result.sg
    reason = (sg.re_reason or "").lower()
    stage = (sg.re_stage or "").lower()
    if "uart" in reason or "test_connection" in reason or "uart" in (sg.validate_fail_reason_detail or "").lower():
        return "RE_UART_VALIDATE"
    if "current" in stage or "current" in reason:
        return "RE_CURRENT_APPLY"
    if "state" in reason:
        return "RE_STATE_INVALID"
    if "driver" in reason or "drv" in reason:
        return "RE_DRIVER_STATUS"
    return "RE_SERIAL_SYNC"


def limit_expected_state(result: SweepResult) -> str:
    if result.failure_reason in ("LOFF", "LPOS") or result.final_result == "PASS":
        return "ON"
    return "NA"


def limit_failure_detail(result: SweepResult) -> str:
    if result.failure_reason == "SETUP_TO":
        return "NA"
    sg = result.sg
    if sg.limit_transition_count is not None and sg.limit_transition_count > 3:
        return "LIMIT_BOUNCE"
    if result.failure_reason == "LOFF":
        return "LIMIT_NOT_REACHED"
    if result.final_limit_timing == "EARLY_LIMIT":
        return "LIMIT_EARLY"
    return "NA"


def current_error_ma(result: SweepResult) -> int | None:
    if result.sg.validate_current_error_ma is not None:
        return result.sg.validate_current_error_ma
    if result.sg.applied_current_ma is None or result.sg.requested_current_ma is None:
        return None
    return result.sg.applied_current_ma - result.sg.requested_current_ma


def current_error_ratio(result: SweepResult) -> float | None:
    if result.sg.validate_current_error_ratio is not None:
        return result.sg.validate_current_error_ratio
    error = current_error_ma(result)
    requested = result.sg.requested_current_ma
    if error is None or requested is None or requested == 0:
        return None
    return error / requested


def current_tolerance_ma(result: SweepResult) -> int | None:
    if result.sg.validate_current_tolerance_ma is not None:
        return result.sg.validate_current_tolerance_ma
    requested = result.sg.requested_current_ma
    if requested is None:
        return None
    return max(50, int(requested * 0.10))


def result_motion_error_detail(result: SweepResult) -> dict[str, float] | None:
    failed = first_failed_test(result.test1, result.test2)
    return last_motion_error_detail(failed.log_excerpt)


def timeout_expected_duration_ms_text(result: SweepResult) -> str:
    failed = first_failed_test(result.test1, result.test2)
    if failed.timeout_diag is not None and ":setup:" in failed.timeout_diag.phase:
        return "NA"
    p = result.param
    distance = 50.0 if failed.name == "test2" else 10.0
    if p.speed_mm_s <= 0:
        return "NA"
    return f"{distance / p.speed_mm_s * 1000.0:.0f}"


def sg_diagnostic(result: SweepResult) -> dict[str, str]:
    sg = result.sg
    tags: list[str] = []
    if sg.sg_valid_count is None or sg.sg_valid_count < 10:
        tags.append("SG_INSUFFICIENT_SAMPLES")
    if sg.sg_zero_count is not None and sg.sg_count is not None and sg.sg_count > 0 and sg.sg_zero_count / sg.sg_count > 0.05:
        tags.append("SG_ZERO_SAMPLES")
    if sg.sg_avg_valid is not None and sg.sg_avg_valid < 120.0:
        tags.append("SG_LOW_AVG")
    if sg.sg_low_ratio_150 is not None and sg.sg_low_ratio_150 > 0.5:
        tags.append("SG_LOW")

    if not tags:
        health = "SG_NORMAL"
    elif "SG_INSUFFICIENT_SAMPLES" in tags and len(tags) == 1:
        health = "SG_UNKNOWN"
    else:
        health = "SG_WARN"
    if result.failure_reason == "SETUP_TO":
        diag = first_failed_test(result.test1, result.test2).timeout_diag
        if diag is not None and diag.firmware_responsive:
            comment = "Setup timeout with firmware responsive; likely expected response mismatch or serial log loss, not firmware halt"
        else:
            comment = "Setup timeout before motion; inspect command response and serial log tail"
    else:
        comment = "SG is diagnostic only; it does not affect PASS/FAIL"
    return {"health": health, "tags": ";".join(tags) if tags else "NONE", "comment": comment}


def write_markdown_report(
    report_path: Path,
    result_csv_path: Path,
    input_csv: Path,
    timestamp: str,
    port: str,
    baudrate: int,
    return_speed: float,
    limit_position_tolerance: float,
    results: list[SweepResult],
    heatmap_paths: list[Path],
    heatmap_warning: str,
) -> None:
    lines: list[str] = []
    lines.append("# Step Loss Sweep Report")
    lines.append("")
    lines.append("## Run")
    lines.append("")
    lines.extend(markdown_table(
        ["Item", "Value"],
        [
            ["timestamp", timestamp],
            ["serial port", port],
            ["baudrate", str(baudrate)],
            ["return speed", f"{fmt_num(return_speed)} mm/s"],
            ["limit position tolerance", f"{fmt_num(limit_position_tolerance)} mm"],
            ["input CSV", str(input_csv)],
            ["output CSV", str(result_csv_path)],
        ],
    ))

    lines.append("")
    lines.append("## Best Safe Settings")
    lines.append("")
    lines.extend(best_safe_settings_table(results))

    lines.append("")
    lines.append("## Condition Summary")
    lines.append("")
    lines.extend(condition_summary_table(results))

    lines.append("")
    lines.append("## Failure Diagnostics")
    lines.append("")
    lines.extend(failure_diagnostics_report(results))

    if has_sg_overhead_metadata(results):
        lines.append("")
        lines.append("## SG Overhead Validation")
        lines.append("")
        lines.extend(sg_overhead_validation_table(results))

    if any(item.param.test_condition_id == "B_REPRO" for item in results):
        lines.append("")
        lines.append("## B_REPRO Results")
        lines.append("")
        lines.extend(b_repro_report_section(results))

    if any(item.param.test_condition_id.startswith("CUR_") for item in results):
        lines.append("")
        lines.append("## Current Sweep Analysis")
        lines.append("")
        lines.extend(current_sweep_report_section(results))

    lines.append("")
    lines.append("## OK/NG Graph")
    for key, group in grouped_by_condition(results).items():
        lines.append("")
        lines.append(f"### {condition_title(key)}")
        lines.append("")
        lines.extend(grid_table(group, values="result"))

    lines.append("")
    lines.append("## Failure Reason Map")
    lines.append("")
    lines.extend(markdown_table(["Code", "Meaning"], [[code, meaning] for code, meaning in FAILURE_LEGEND.items()]))
    for key, group in grouped_by_condition(results).items():
        lines.append("")
        lines.append(f"### {condition_title(key)}")
        lines.append("")
        lines.extend(grid_table(group, values="failure"))

    lines.append("")
    lines.append("## Final Limit Timing Map")
    lines.append("")
    lines.extend(markdown_table(
        ["Code", "Meaning"],
        [
            ["DURING_MOVE", "5th b hit the limit before Move complete"],
            ["AFTER_COMPLETE", "5th b completed, then status reported limit ON"],
            ["AFTER_SETTLE", "5th b completed, raw limit was ON, and debounced limit became ON after the settle wait"],
            ["EARLY_LIMIT", "limit was hit before the 5th b"],
            ["NOT_REACHED", "5th b completed but final limit was OFF"],
            ["UNKNOWN", "timing could not be classified"],
        ],
    ))
    for key, group in grouped_by_condition(results).items():
        lines.append("")
        lines.append(f"### {condition_title(key)}")
        lines.append("")
        lines.extend(grid_table(group, values="timing"))

    lines.append("")
    lines.append("## Optional PNG Heatmaps")
    lines.append("")
    if heatmap_warning:
        lines.append(f"- {heatmap_warning}")
    elif heatmap_paths:
        for path in heatmap_paths:
            lines.append(f"- `{path}`")
    else:
        lines.append("- No heatmaps were generated.")

    lines.append("")
    lines.append("## Detail Results")
    lines.append("")
    lines.extend(detail_results_table(results))

    failed = [item for item in results if item.final_result != "PASS"]
    if failed:
        lines.append("")
        lines.append("## Failure Log Excerpts")
        for result in failed:
            p = result.param
            lines.append("")
            lines.append(f"### speed={fmt_num(p.speed_mm_s)} accel={fmt_num(p.accel_mm_s2)} current={p.current_ma} mode={p.chop_mode} microsteps={p.microsteps}")
            for test in (result.test1, result.test2):
                if test.result == "PASS":
                    continue
                lines.append("")
                lines.append(f"#### {test.name}: {test.failure_reason}")
                lines.append("")
                lines.append("```text")
                lines.extend(test.log_excerpt)
                lines.append("```")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n")


def best_safe_settings_table(results: list[SweepResult]) -> list[str]:
    groups: dict[tuple[int, str, int, float], list[SweepResult]] = defaultdict(list)
    for result in results:
        p = result.param
        groups[(p.current_ma, p.chop_mode, p.microsteps, p.accel_mm_s2)].append(result)

    rows: list[list[str]] = []
    for key in sorted(groups):
        current, chop, microsteps, accel = key
        group = groups[key]
        pass_speeds = sorted({item.param.speed_mm_s for item in group if item.final_result == "PASS"})
        all_speeds = sorted({item.param.speed_mm_s for item in group})
        if pass_speeds:
            max_pass = pass_speeds[-1]
            lower_speeds = [speed for speed in all_speeds if speed < max_pass]
            recommended = lower_speeds[-1] if lower_speeds else max_pass * 0.8
            max_pass_text = fmt_num(max_pass)
            recommended_text = fmt_num(recommended)
        else:
            max_pass_text = "-"
            recommended_text = "-"
        rows.append([str(current), chop, str(microsteps), fmt_num(accel), max_pass_text, recommended_text])

    return markdown_table(
        ["current_ma", "chop_mode", "microsteps", "accel_mm_s2", "max_pass_speed_mm_s", "recommended_speed_mm_s"],
        rows,
    )


def condition_summary_table(results: list[SweepResult]) -> list[str]:
    rows: list[list[str]] = []
    for key, group in grouped_by_condition(results).items():
        pass_count = sum(1 for item in group if item.final_result == "PASS")
        fail_count = len(group) - pass_count
        pass_rate = pass_count / len(group) * 100.0 if group else 0.0
        pass_speeds = [item.param.speed_mm_s for item in group if item.final_result == "PASS"]
        max_pass = fmt_num(max(pass_speeds)) if pass_speeds else "-"
        current, chop, microsteps = key
        rows.append([str(current), chop, str(microsteps), str(pass_count), str(fail_count), f"{pass_rate:.1f}%", max_pass])
    return markdown_table(
        ["current_ma", "chop_mode", "microsteps", "pass_count", "fail_count", "pass_rate", "max_pass_speed_mm_s"],
        rows,
    )


def failure_diagnostics_report(results: list[SweepResult]) -> list[str]:
    lines: list[str] = []
    lines.append("### Result Counts")
    lines.append("")
    lines.extend(count_table(results, lambda item: "OK" if item.final_result == "PASS" else "NG", "result"))

    lines.append("")
    lines.append("### Failure Reason Counts")
    lines.append("")
    lines.extend(count_table(results, lambda item: item.failure_reason, "failure_reason"))

    lines.append("")
    lines.append("### Failure Detail Counts")
    lines.append("")
    lines.extend(count_table(results, lambda item: classify_failure(item)[0], "failure_detail"))

    lines.append("")
    lines.append("### Timeout Rows")
    lines.append("")
    lines.extend(markdown_table(
        ["condition", "phase", "command", "detail", "elapsed_ms", "firmware", "diag", "status", "mt", "timeout_pos", "target", "remaining", "limit"],
        [
            [
                item.param.test_condition_id,
                timeout_diag_text(item, "phase"),
                timeout_diag_text(item, "command"),
                classify_failure(item)[0],
                timeout_diag_text(item, "elapsed_ms") if timeout_diag_text(item, "elapsed_ms") != "NA" else f"{item.elapsed_sec * 1000.0:.0f}",
                timeout_diag_text(item, "firmware"),
                timeout_diag_text(item, "diag"),
                timeout_diag_text(item, "status"),
                timeout_diag_text(item, "mt"),
                "NA" if item.sg.timeout_current_position is None else f"{item.sg.timeout_current_position:.4f}",
                "NA" if item.sg.timeout_target_position is None else f"{item.sg.timeout_target_position:.4f}",
                value_or_na(item.sg.timeout_remaining_steps),
                value_or_na(item.sg.timeout_limit_state),
            ]
            for item in results
            if is_timeout_reason(item.failure_reason)
        ],
    ))

    timeout_details = timeout_detail_report(results)
    if timeout_details:
        lines.append("")
        lines.extend(timeout_details)

    lines.append("")
    lines.append("### ME Rows")
    lines.append("")
    lines.extend(markdown_table(
        ["condition", "detail", "expected", "actual", "error_mm", "remaining_steps", "threshold_mm"],
        [
            [
                item.param.test_condition_id,
                classify_failure(item)[0],
                motion_value_text(result_motion_error_detail(item), "target"),
                motion_value_text(result_motion_error_detail(item), "pos"),
                "NA" if item.final_error_mm is None else f"{item.final_error_mm:.4f}",
                value_or_na(item.final_remaining_steps),
                "0.5000",
            ]
            for item in results
            if item.failure_reason in ("ME", "LPOS")
        ],
    ))

    lines.append("")
    lines.append("### Limit Rows")
    lines.append("")
    lines.extend(markdown_table(
        ["condition", "detail", "start", "end", "home_end", "transitions", "first_trigger", "expected", "actual"],
        [
            [
                item.param.test_condition_id,
                limit_failure_detail(item),
                value_or_na(item.sg.move_start_limit_state),
                value_or_na(item.sg.move_end_limit_state),
                value_or_na(item.sg.home_end_limit_state),
                value_or_na(item.sg.limit_transition_count),
                value_or_na(item.sg.limit_first_trigger_timing),
                limit_expected_state(item),
                item.final_limit_state,
            ]
            for item in results
            if limit_failure_detail(item) != "NA"
        ],
    ))

    lines.append("")
    lines.append("### RE Rows")
    lines.append("")
    lines.extend(markdown_table(
        ["condition", "detail", "stage", "reason", "requested", "applied", "config_uart", "validate_uart", "driver", "validate_detail"],
        [
            [
                item.param.test_condition_id,
                classify_failure(item)[0],
                value_or_na(item.sg.re_stage),
                value_or_na(item.sg.re_reason),
                value_or_na(item.sg.requested_current_ma),
                value_or_na(item.sg.applied_current_ma),
                value_or_na(item.sg.config_tmc_uart_ok),
                value_or_na(item.sg.validate_tmc_uart_ok),
                value_or_na(item.sg.driver_status),
                value_or_na(item.sg.validate_fail_reason_detail),
            ]
            for item in results
            if item.failure_reason == "RE"
        ],
    ))

    lines.append("")
    lines.append("### Diagnostic Tags")
    lines.append("")
    lines.extend(count_table(results, lambda item: sg_diagnostic(item)["tags"], "diagnostic_tags"))

    lines.append("")
    lines.append("### SG Diagnostic Cross Check")
    lines.append("")
    rows: list[list[str]] = []
    for item in results:
        diagnostic = sg_diagnostic(item)
        if ("SG_LOW" in diagnostic["tags"] or "SG_LOW_AVG" in diagnostic["tags"]) and item.final_result == "PASS":
            rows.append([item.param.test_condition_id, "SG_LOW_OK", diagnostic["tags"], item.failure_reason])
        if diagnostic["health"] == "SG_NORMAL" and item.final_result != "PASS":
            rows.append([item.param.test_condition_id, "SG_NORMAL_NG", diagnostic["tags"], item.failure_reason])
    lines.extend(markdown_table(["condition", "case", "diagnostic_tags", "failure_reason"], rows))
    lines.append("")
    lines.append("SG diagnostics are not used to determine PASS/FAIL.")
    return lines


def timeout_diag_for(result: SweepResult) -> TimeoutDiagnostic | None:
    if not is_timeout_reason(result.failure_reason):
        return None
    return first_failed_test(result.test1, result.test2).timeout_diag


def timeout_diag_text(result: SweepResult, field: str) -> str:
    diag = timeout_diag_for(result)
    if diag is None:
        return "NA"
    if field == "phase":
        return diag.phase
    if field == "command":
        return diag.command
    if field == "elapsed_ms":
        return str(diag.elapsed_ms)
    if field == "firmware":
        return "yes" if diag.firmware_responsive else "no"
    if field == "diag":
        return diag.diag_code
    if field == "status":
        return diag.status_code
    if field == "mt":
        return diag.mt_code
    return "NA"


def timeout_detail_report(results: list[SweepResult]) -> list[str]:
    timeout_results = [item for item in results if timeout_diag_for(item) is not None]
    if not timeout_results:
        return []
    lines = ["### TO Diagnostic Logs", ""]
    for item in timeout_results:
        diag = timeout_diag_for(item)
        if diag is None:
            continue
        lines.append(f"#### {item.param.test_condition_id} {diag.phase}")
        lines.append("")
        lines.extend(markdown_table(
            ["Item", "Value"],
            [
                ["command", diag.command],
                ["elapsed_ms", str(diag.elapsed_ms)],
                ["firmware_responsive", "yes" if diag.firmware_responsive else "no"],
                ["diag_probe", diag.diag_code],
                ["s_probe", diag.status_code],
                ["mt_probe", diag.mt_code],
            ],
        ))
        lines.append("")
        lines.append("Serial tail before TO:")
        lines.append("")
        lines.append("```text")
        lines.extend(diag.serial_tail or ["NA"])
        lines.append("```")
        lines.append("")
        lines.append("`diag` probe response:")
        lines.append("")
        lines.append("```text")
        lines.extend(diag.diag_lines or ["NA"])
        lines.append("```")
        lines.append("")
        lines.append("`s` probe response:")
        lines.append("")
        lines.append("```text")
        lines.extend(diag.status_lines or ["NA"])
        lines.append("```")
        lines.append("")
        lines.append("`mt` probe response:")
        lines.append("")
        lines.append("```text")
        lines.extend(diag.mt_lines or ["NA"])
        lines.append("```")
        lines.append("")
    return lines


def count_table(results: list[SweepResult], key_fn, key_name: str) -> list[str]:
    counts: dict[str, int] = defaultdict(int)
    for item in results:
        counts[key_fn(item)] += 1
    return markdown_table([key_name, "count"], [[key, str(counts[key])] for key in sorted(counts)])


def motion_value_text(detail: dict[str, float] | None, key: str) -> str:
    if detail is None or key not in detail:
        return "NA"
    return f"{detail[key]:.4f}"


def has_sg_overhead_metadata(results: list[SweepResult]) -> bool:
    for result in results:
        p = result.param
        if p.test_condition_id != "NA":
            return True
        if p.repeat_index is not None or p.sg_enabled is not None or p.sg_interval_ms is not None or p.sglog_enabled is not None:
            return True
    return False


def sg_overhead_validation_table(results: list[SweepResult]) -> list[str]:
    groups: dict[str, list[SweepResult]] = defaultdict(list)
    for result in results:
        groups[result.param.test_condition_id].append(result)

    rows: list[list[str]] = []
    for condition_id in sorted(groups):
        group = groups[condition_id]
        pass_count = sum(1 for item in group if item.final_result == "PASS")
        fail_count = len(group) - pass_count
        to_count = sum(1 for item in group if item.failure_reason == "TO")
        me_count = sum(1 for item in group if item.failure_reason == "ME")
        sample = group[0].param
        rows.append([
            condition_id,
            fmt_num(sample.speed_mm_s),
            fmt_num(sample.accel_mm_s2),
            str(sample.current_ma),
            sample.chop_mode,
            str(sample.microsteps),
            value_or_na(sample.sg_enabled),
            value_or_na(sample.sg_interval_ms),
            value_or_na(sample.sglog_enabled),
            str(len(group)),
            str(pass_count),
            str(fail_count),
            str(to_count),
            str(me_count),
            avg_float_text([item.final_error_mm for item in group], digits=4),
            max_float_text([item.final_error_mm for item in group], digits=4),
            avg_float_text([item.sg.max_update_gap_us for item in group], digits=1),
            max_int_text([item.sg.max_update_gap_us for item in group]),
            avg_float_text([item.sg.sg_avg_valid for item in group if item.param.sg_enabled != 0], digits=1),
            avg_float_text([item.sg.sg_low_ratio_150 for item in group if item.param.sg_enabled != 0], digits=4),
        ])

    return markdown_table(
        [
            "test_condition_id",
            "speed",
            "accel",
            "current",
            "chop",
            "microsteps",
            "sg_enabled",
            "sg_interval_ms",
            "sglog_enabled",
            "total",
            "OK",
            "NG",
            "TO",
            "ME",
            "avg_error_mm",
            "max_error_mm",
            "avg_max_gap_us",
            "max_gap_us",
            "avg_sg_avg_valid",
            "avg_sg_low_ratio_150",
        ],
        rows,
    )


def b_repro_report_section(results: list[SweepResult]) -> list[str]:
    group = [item for item in results if item.param.test_condition_id == "B_REPRO"]
    if not group:
        return ["No B_REPRO rows."]
    ok_count = sum(1 for item in group if item.final_result == "PASS")
    to_count = sum(1 for item in group if item.failure_reason == "TO")
    loff_count = sum(1 for item in group if item.failure_reason == "LOFF")
    me_count = sum(1 for item in group if item.failure_reason == "ME")

    lines: list[str] = []
    lines.extend(markdown_table(
        ["Metric", "Value"],
        [
            ["total", str(len(group))],
            ["OK", str(ok_count)],
            ["TO", str(to_count)],
            ["LOFF", str(loff_count)],
            ["ME", str(me_count)],
        ],
    ))
    lines.append("")
    lines.extend(markdown_table(
        [
            "repeat",
            "result",
            "reason",
            "error_mm",
            "elapsed_sec",
            "limit",
            "timing",
            "move_start_limit",
            "move_end_limit",
            "timeout_limit",
            "limit_transitions",
            "motion_updates",
            "max_gap_us",
            "avg_gap_us",
            "timeout_pos",
            "timeout_target",
            "timeout_remaining",
            "sg_avg_valid",
            "sg_low_ratio_150",
        ],
        [
            [
                value_or_na(item.param.repeat_index),
                "OK" if item.final_result == "PASS" else "NG",
                item.failure_reason,
                "NA" if item.final_error_mm is None else f"{item.final_error_mm:.4f}",
                f"{item.elapsed_sec:.3f}",
                item.final_limit_state,
                item.final_limit_timing,
                value_or_na(item.sg.move_start_limit_state),
                value_or_na(item.sg.move_end_limit_state),
                value_or_na(item.sg.timeout_limit_state),
                value_or_na(item.sg.limit_transition_count),
                value_or_na(item.sg.motion_update_count),
                value_or_na(item.sg.max_update_gap_us),
                "NA" if item.sg.avg_update_gap_us is None else f"{item.sg.avg_update_gap_us:.1f}",
                "NA" if item.sg.timeout_current_position is None else f"{item.sg.timeout_current_position:.4f}",
                "NA" if item.sg.timeout_target_position is None else f"{item.sg.timeout_target_position:.4f}",
                value_or_na(item.sg.timeout_remaining_steps),
                "NA" if item.sg.sg_avg_valid is None else f"{item.sg.sg_avg_valid:.1f}",
                "NA" if item.sg.sg_low_ratio_150 is None else f"{item.sg.sg_low_ratio_150:.4f}",
            ]
            for item in sorted(group, key=lambda result: result.param.repeat_index or 0)
        ],
    ))
    lines.append("")
    lines.append("Decision notes: mixed OK/TO/LOFF results point to the test sequence, limit detection, completion detection, or motion update cadence in addition to true step loss. Large `max_update_gap_us` or `timeout_remaining_steps` on TO rows suggests update jitter or an unfinished move. Good SG values with TO/LOFF should push the next investigation toward control logic and limit input behavior before StallGuard thresholds.")
    return lines


def current_sweep_report_section(results: list[SweepResult]) -> list[str]:
    group = [item for item in results if item.param.test_condition_id.startswith("CUR_")]
    if not group:
        return ["No current sweep rows."]

    lines: list[str] = []
    lines.append("### OK/NG By Current")
    lines.append("")
    lines.extend(ok_ng_by_key_table(group, lambda item: str(item.param.current_ma), "current_ma"))

    lines.append("")
    lines.append("### OK/NG By Speed")
    lines.append("")
    lines.extend(ok_ng_by_key_table(group, lambda item: fmt_num(item.param.speed_mm_s), "speed_mm_s"))

    lines.append("")
    lines.append("### OK/NG By Accel")
    lines.append("")
    lines.extend(ok_ng_by_key_table(group, lambda item: fmt_num(item.param.accel_mm_s2), "accel_mm_s2"))

    lines.append("")
    lines.append("### Error And SG Detail")
    lines.append("")
    lines.extend(markdown_table(
        [
            "condition",
            "result",
            "reason",
            "final_error_mm",
            "sg_avg_valid",
            "sg_low_ratio_150",
            "max_update_gap_us",
        ],
        [
            [
                item.param.test_condition_id,
                "OK" if item.final_result == "PASS" else "NG",
                item.failure_reason,
                "NA" if item.final_error_mm is None else f"{item.final_error_mm:.4f}",
                "NA" if item.sg.sg_avg_valid is None else f"{item.sg.sg_avg_valid:.1f}",
                "NA" if item.sg.sg_low_ratio_150 is None else f"{item.sg.sg_low_ratio_150:.4f}",
                value_or_na(item.sg.max_update_gap_us),
            ]
            for item in sorted(group, key=current_sweep_sort_key)
        ],
    ))

    lines.append("")
    lines.append("### Failure Details")
    lines.append("")
    failures = [item for item in group if item.failure_reason in ("TO", "ME", "LOFF")]
    lines.extend(markdown_table(
        [
            "condition",
            "reason",
            "limit",
            "timing",
            "move_start_limit",
            "move_end_limit",
            "timeout_limit",
            "timeout_remaining",
            "max_update_gap_us",
            "sg_avg_valid",
            "sg_low_ratio_150",
        ],
        [
            [
                item.param.test_condition_id,
                item.failure_reason,
                item.final_limit_state,
                item.final_limit_timing,
                value_or_na(item.sg.move_start_limit_state),
                value_or_na(item.sg.move_end_limit_state),
                value_or_na(item.sg.timeout_limit_state),
                value_or_na(item.sg.timeout_remaining_steps),
                value_or_na(item.sg.max_update_gap_us),
                "NA" if item.sg.sg_avg_valid is None else f"{item.sg.sg_avg_valid:.1f}",
                "NA" if item.sg.sg_low_ratio_150 is None else f"{item.sg.sg_low_ratio_150:.4f}",
            ]
            for item in sorted(failures, key=current_sweep_sort_key)
        ],
    ))

    safe = [item for item in group if item.final_result == "PASS"]
    lines.append("")
    lines.append("### Recommended Safe Conditions")
    lines.append("")
    lines.extend(markdown_table(
        ["condition", "current", "speed", "accel", "error_mm", "sg_avg_valid", "sg_low_ratio_150", "max_update_gap_us"],
        [
            [
                item.param.test_condition_id,
                str(item.param.current_ma),
                fmt_num(item.param.speed_mm_s),
                fmt_num(item.param.accel_mm_s2),
                "NA" if item.final_error_mm is None else f"{item.final_error_mm:.4f}",
                "NA" if item.sg.sg_avg_valid is None else f"{item.sg.sg_avg_valid:.1f}",
                "NA" if item.sg.sg_low_ratio_150 is None else f"{item.sg.sg_low_ratio_150:.4f}",
                value_or_na(item.sg.max_update_gap_us),
            ]
            for item in sorted(safe, key=current_sweep_sort_key)
        ],
    ))

    boundary = current_sweep_boundary_candidates(group)
    lines.append("")
    lines.append("### Boundary Candidates")
    lines.append("")
    lines.extend(markdown_table(
        ["condition", "current", "speed", "accel", "result", "reason", "neighbor_note"],
        boundary,
    ))
    return lines


def ok_ng_by_key_table(results: list[SweepResult], key_fn, key_name: str) -> list[str]:
    grouped: dict[str, list[SweepResult]] = defaultdict(list)
    for item in results:
        grouped[key_fn(item)].append(item)
    rows: list[list[str]] = []
    for key in sorted(grouped, key=numeric_sort_key):
        items = grouped[key]
        ok_count = sum(1 for item in items if item.final_result == "PASS")
        ng_count = len(items) - ok_count
        rows.append([key, str(ok_count), str(ng_count), f"{ok_count / len(items) * 100.0:.1f}%"])
    return markdown_table([key_name, "OK", "NG", "OK_rate"], rows)


def current_sweep_sort_key(item: SweepResult) -> tuple[int, float, float]:
    return (item.param.current_ma, item.param.speed_mm_s, item.param.accel_mm_s2)


def current_sweep_boundary_candidates(results: list[SweepResult]) -> list[list[str]]:
    by_point = {
        (item.param.current_ma, item.param.speed_mm_s, item.param.accel_mm_s2): item
        for item in results
    }
    rows: list[list[str]] = []
    for item in sorted(results, key=current_sweep_sort_key):
        p = item.param
        neighbors = [
            by_point.get((p.current_ma, p.speed_mm_s - 20, p.accel_mm_s2)),
            by_point.get((p.current_ma, p.speed_mm_s + 20, p.accel_mm_s2)),
            by_point.get((p.current_ma, p.speed_mm_s, 100.0 if p.accel_mm_s2 == 300.0 else 300.0)),
        ]
        has_mixed_neighbor = any(neighbor is not None and neighbor.final_result != item.final_result for neighbor in neighbors)
        if item.final_result != "PASS" or has_mixed_neighbor:
            note = "near mixed OK/NG boundary" if has_mixed_neighbor else "failed condition"
            rows.append([
                p.test_condition_id,
                str(p.current_ma),
                fmt_num(p.speed_mm_s),
                fmt_num(p.accel_mm_s2),
                "OK" if item.final_result == "PASS" else "NG",
                item.failure_reason,
                note,
            ])
    return rows


def numeric_sort_key(value: str) -> tuple[int, float | str]:
    try:
        return (0, float(value))
    except ValueError:
        return (1, value)


def value_or_na(value: object | None) -> str:
    return "NA" if value is None else str(value)


def avg_float_text(values: Iterable[float | int | None], digits: int) -> str:
    numeric = [float(value) for value in values if value is not None]
    if not numeric:
        return "NA"
    return f"{sum(numeric) / len(numeric):.{digits}f}"


def max_float_text(values: Iterable[float | int | None], digits: int) -> str:
    numeric = [float(value) for value in values if value is not None]
    if not numeric:
        return "NA"
    return f"{max(numeric):.{digits}f}"


def max_int_text(values: Iterable[int | None]) -> str:
    numeric = [value for value in values if value is not None]
    if not numeric:
        return "NA"
    return str(max(numeric))


def detail_results_table(results: list[SweepResult]) -> list[str]:
    rows: list[list[str]] = []
    for result in results:
        p = result.param
        rows.append([
            fmt_num(p.speed_mm_s),
            fmt_num(p.accel_mm_s2),
            str(p.current_ma),
            p.chop_mode,
            str(p.microsteps),
            result.test1.result,
            result.test2.result,
            result.final_result,
            result.failure_reason,
            result.final_limit_state,
            result.final_limit_timing,
            f"{result.final_score:.1f}",
            "-" if result.final_error_mm is None else f"{result.final_error_mm:.4f}",
            "-" if result.final_remaining_steps is None else str(result.final_remaining_steps),
            f"{result.elapsed_sec:.1f}",
        ])
    return markdown_table(
        [
            "speed",
            "accel",
            "current",
            "chop",
            "microsteps",
            "test1",
            "test2",
            "final",
            "reason",
            "limit",
            "limit_timing",
            "score",
            "error_mm",
            "remaining_steps",
            "elapsed_sec",
        ],
        rows,
    )


def grouped_by_condition(results: list[SweepResult]) -> dict[tuple[int, str, int], list[SweepResult]]:
    groups: dict[tuple[int, str, int], list[SweepResult]] = defaultdict(list)
    for result in results:
        p = result.param
        groups[(p.current_ma, p.chop_mode, p.microsteps)].append(result)
    return dict(sorted(groups.items()))


def condition_title(key: tuple[int, str, int]) -> str:
    current, chop, microsteps = key
    return f"current={current}mA, chop={chop}, microsteps=1/{microsteps}"


def grid_table(group: list[SweepResult], values: str) -> list[str]:
    speeds = sorted({item.param.speed_mm_s for item in group})
    accels = sorted({item.param.accel_mm_s2 for item in group}, reverse=True)
    by_point: dict[tuple[float, float], list[SweepResult]] = defaultdict(list)
    for item in group:
        by_point[(item.param.accel_mm_s2, item.param.speed_mm_s)].append(item)

    rows: list[list[str]] = []
    for accel in accels:
        row = [fmt_num(accel)]
        for speed in speeds:
            point_results = by_point.get((accel, speed), [])
            row.append(grid_cell(point_results, values))
        rows.append(row)
    return markdown_table(["accel \\ speed"] + [fmt_num(speed) for speed in speeds], rows)


def grid_cell(results: list[SweepResult], values: str) -> str:
    if not results:
        return "-"
    if values == "result":
        return "OK" if any(item.final_result == "PASS" for item in results) else "NG"
    if values == "timing":
        priority = ("DURING_MOVE", "AFTER_COMPLETE", "AFTER_SETTLE", "EARLY_LIMIT", "NOT_REACHED", "UNKNOWN")
        timings = {item.final_limit_timing for item in results}
        for timing in priority:
            if timing in timings:
                return timing
        return "UNKNOWN"
    if any(item.final_result == "PASS" for item in results):
        return "OK"
    return results[0].failure_reason


def markdown_table(headers: list[str], rows: list[list[str]]) -> list[str]:
    output = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    if rows:
        output.extend("| " + " | ".join(escape_cell(str(cell)) for cell in row) + " |" for row in rows)
    else:
        output.append("| " + " | ".join(["-"] * len(headers)) + " |")
    return output


def escape_cell(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", "<br>")


def write_optional_heatmaps(reports_dir: Path, timestamp: str, results: list[SweepResult]) -> tuple[list[Path], str]:
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.colors import ListedColormap  # type: ignore
    except ImportError:
        return [], "matplotlib is not available; PNG heatmaps were skipped."

    paths: list[Path] = []
    cmap = ListedColormap(["#d95f5f", "#6fbf73"])
    for key, group in grouped_by_condition(results).items():
        speeds = sorted({item.param.speed_mm_s for item in group})
        accels = sorted({item.param.accel_mm_s2 for item in group})
        matrix: list[list[int]] = []
        for accel in accels:
            row: list[int] = []
            for speed in speeds:
                matching = [item for item in group if item.param.accel_mm_s2 == accel and item.param.speed_mm_s == speed]
                row.append(1 if any(item.final_result == "PASS" for item in matching) else 0)
            matrix.append(row)

        current, chop, microsteps = key
        fig, ax = plt.subplots(figsize=(max(5, len(speeds) * 0.65), max(4, len(accels) * 0.45)))
        ax.imshow(matrix, cmap=cmap, vmin=0, vmax=1, origin="lower", aspect="auto")
        ax.set_xticks(range(len(speeds)), [fmt_num(speed) for speed in speeds])
        ax.set_yticks(range(len(accels)), [fmt_num(accel) for accel in accels])
        ax.set_xlabel("speed_mm_s")
        ax.set_ylabel("accel_mm_s2")
        ax.set_title(condition_title(key))
        for y, row in enumerate(matrix):
            for x, value in enumerate(row):
                ax.text(x, y, "OK" if value else "NG", ha="center", va="center", color="black")
        fig.tight_layout()
        path = reports_dir / f"step_loss_heatmap_{timestamp}_{current}mA_{chop}_micro{microsteps}.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(path)
    return paths, ""


def fmt_num(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return f"{value:.3f}".rstrip("0").rstrip(".")


if __name__ == "__main__":
    raise SystemExit(main())
