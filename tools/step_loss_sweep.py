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
)

FAILURE_LEGEND = {
    "OK": "pass",
    "SL": "suspected step loss",
    "TO": "timeout",
    "HE": "homing error",
    "ME": "motion error",
    "RE": "command rejected",
    "LOFF": "final limit OFF",
    "LPOS": "final limit ON too far from zero",
}


@dataclasses.dataclass(frozen=True)
class SweepParam:
    speed_mm_s: float
    accel_mm_s2: float
    current_ma: int
    chop_mode: str
    microsteps: int


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
    final_status: str
    log_excerpt: list[str]


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

    def wait_for_homing(self, timeout_sec: float) -> tuple[bool, str, list[str]]:
        return self.send_and_wait("h", ("Homing complete",), timeout_sec)

    def wait_for_move(self, command: str, timeout_sec: float) -> tuple[bool, str, list[str]]:
        return self.send_and_wait(command, ("Move complete",), timeout_sec)

    def status(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        ok, code, lines = self.send_and_wait("s", ("state=",), timeout_sec, reject_as_error=False)
        if not ok:
            return code, lines
        return "OK", lines


class SimulatedRunner:
    """Small deterministic runner for report-generation smoke tests."""

    def __init__(self) -> None:
        self.position_units = 0

    def close(self) -> None:
        return

    def send_and_wait(
        self,
        command: str,
        expect: Iterable[str],
        timeout_sec: float,
        reject_as_error: bool = True,
    ) -> tuple[bool, str, list[str]]:
        _ = expect, timeout_sec, reject_as_error
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

    def status(self, timeout_sec: float = 2.0) -> tuple[str, list[str]]:
        _ = timeout_sec
        return "OK", [self._status_line()]

    def _status_line(self) -> str:
        limit = "ON" if self.position_units <= 0 else "OFF"
        return f"state=Ready homing=Done motion=Idle pos={self.position_units * 10}.00mm limitDebounced={limit}"


def main() -> int:
    args = parse_args()
    params = load_params(args.csv)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    reports_dir = Path(args.reports_dir)
    reports_dir.mkdir(parents=True, exist_ok=True)
    report_path = Path(args.report) if args.report else reports_dir / f"step_loss_sweep_{timestamp}.md"
    result_csv_path = reports_dir / f"step_loss_sweep_{timestamp}.csv"

    selected_port = "simulate" if args.simulate else resolve_port(args.port)
    runner = SimulatedRunner() if args.simulate else SerialRunner(selected_port, args.baudrate)

    results: list[SweepResult] = []
    try:
        for index, param in enumerate(params, start=1):
            prefix = f"[{index}/{len(params)}] speed={fmt_num(param.speed_mm_s)} accel={fmt_num(param.accel_mm_s2)} current={param.current_ma} mode={param.chop_mode} microsteps={param.microsteps}"
            print(prefix, flush=True)
            result = run_param(runner, param, args.return_speed, args.command_timeout, args.homing_timeout, args.limit_position_tolerance, args.limit_settle_sec, timestamp)
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep stepper parameters and generate step-loss reports.")
    parser.add_argument("--csv", default="tools/step_loss_params.csv", help="input CSV path")
    parser.add_argument("--port", default="auto", help="serial port, or auto")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baudrate")
    parser.add_argument("--return-speed", type=float, default=30.0, help="slow return speed before b moves")
    parser.add_argument("--reports-dir", default="reports", help="report output directory")
    parser.add_argument("--report", default="", help="explicit Markdown report path")
    parser.add_argument("--command-timeout", type=float, default=30.0, help="timeout for one normal move")
    parser.add_argument("--homing-timeout", type=float, default=45.0, help="timeout for homing")
    parser.add_argument("--limit-position-tolerance", type=float, default=0.5, help="max final position error in mm when the limit turns ON")
    parser.add_argument("--limit-settle-sec", type=float, default=0.2, help="extra wait after final b when raw limit is ON but debounced limit is still OFF")
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

    return SweepParam(speed, accel, current, chop_mode, microsteps)


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
    timestamp: str,
) -> SweepResult:
    start = time.monotonic()
    test1 = run_test(runner, param, "test1", ["1", "1", "1", "1", "1"], return_speed, command_timeout, homing_timeout, limit_position_tolerance, limit_settle_sec)
    test2 = run_test(runner, param, "test2", ["5"], return_speed, command_timeout, homing_timeout, limit_position_tolerance, limit_settle_sec)
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
        elapsed_sec=time.monotonic() - start,
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
) -> TestOutcome:
    log: list[str] = []

    setup_commands = [
        (f"microstep {param.microsteps}", ("microstep set", "sim:")),
        (f"i {param.current_ma}", ("current set", "sim:")),
        (f"mode {param.chop_mode}", ("chop mode set", "sim:")),
        ("profile trap", ("Motion profile set",)),
        (f"a {fmt_num(param.accel_mm_s2)}", ("Acceleration set",)),
    ]
    for command, expect in setup_commands:
        ok, code, lines = runner.send_and_wait(command, expect, 5.0)
        log.extend(tag_lines(command, lines))
        if not ok:
            return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", "", log, limit_position_tolerance)

    ok, code, lines = runner.wait_for_homing(homing_timeout)
    log.extend(tag_lines("h", lines))
    if not ok:
        return outcome(name, "FAIL", "HE" if code in ("TO", "HE") else code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    ok, code, lines = runner.send_and_wait(f"v {fmt_num(param.speed_mm_s)}", ("Default move speed set",), 5.0)
    log.extend(tag_lines(f"v {fmt_num(param.speed_mm_s)}", lines))
    if not ok:
        return outcome(name, "FAIL", code, "UNKNOWN", "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    for command in forward_commands:
        ok, code, lines = runner.wait_for_move(command, command_timeout)
        log.extend(tag_lines(command, lines))
        if not ok:
            return outcome(name, "FAIL", code, parse_limit_state(lines), "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    ok, code, lines = runner.send_and_wait(f"v {fmt_num(return_speed)}", ("Default move speed set",), 5.0)
    log.extend(tag_lines(f"v {fmt_num(return_speed)}", lines))
    if not ok:
        return outcome(name, "FAIL", code, parse_limit_state(lines), "UNKNOWN", last_status(lines), log, limit_position_tolerance)

    final_lines: list[str] = []
    for index in range(5):
        ok, code, lines = runner.wait_for_move("b", command_timeout)
        log.extend(tag_lines("b", lines))
        final_lines = lines
        if not ok:
            if index == 4 and code == "ME":
                status_code, status_lines = runner.status()
                log.extend(tag_lines("s", status_lines))
                final_lines = status_lines or final_lines
                final_status = last_status(final_lines)
                final_limit = parse_limit_state(final_lines)
                if status_code == "TO":
                    return outcome(name, "FAIL", "TO", final_limit, "UNKNOWN", final_status, log, limit_position_tolerance)
                if final_limit == "ON":
                    if not final_position_within_tolerance(final_status, log, limit_position_tolerance):
                        return outcome(name, "FAIL", "LPOS", final_limit, "DURING_MOVE", final_status, log, limit_position_tolerance)
                    return outcome(name, "PASS", "OK", final_limit, "DURING_MOVE", final_status, log, limit_position_tolerance)
            early_timing = "EARLY_LIMIT" if code == "ME" and parse_limit_state(lines) == "ON" else "UNKNOWN"
            return outcome(name, "FAIL", code, parse_limit_state(lines), early_timing, last_status(lines), log, limit_position_tolerance)

    status_code, status_lines = runner.status()
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
        return outcome(name, "FAIL", "TO", final_limit, "UNKNOWN", final_status, log, limit_position_tolerance)
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
        final_status=final_status,
        log_excerpt=trim_log(log),
    )


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
    return (
        f"{test.result} score={test.score:.1f} reason={test.failure_reason} "
        f"limit={test.final_limit_state} timing={test.final_limit_timing} "
        f"error={format_optional_mm(test.final_error_mm)} "
        f"remainingSteps={format_optional_int(test.final_remaining_steps)}"
    )


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
    }


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
