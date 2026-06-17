#!/usr/bin/env python3
"""Sweep stepper speed/acceleration and judge step loss with an AS5600 shaft encoder."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ANGLE_RE = re.compile(
    r"ANGLE,.*?ok=(?P<ok>[01]),.*?angle_deg=(?P<angle>-?\d+(?:\.\d+)?),"
    r".*?raw_deg=(?P<raw>-?\d+(?:\.\d+)?)"
)


@dataclass(frozen=True)
class Param:
    speed_mm_s: float
    accel_mm_s2: float


@dataclass
class LegResult:
    command_mm: float
    before_deg: float
    after_deg: float
    measured_delta_deg: float
    expected_delta_deg: float
    error_deg: float
    error_mm: float
    move_elapsed_ms: int


@dataclass
class SweepResult:
    timestamp: str
    speed_mm_s: float
    accel_mm_s2: float
    current_ma: int
    chop_mode: str
    microsteps: int
    result: str
    reason: str
    max_abs_error_deg: float | None
    max_abs_error_mm: float | None
    final_drift_deg: float | None
    final_drift_mm: float | None
    elapsed_sec: float
    log_excerpt: str
    legs: list[LegResult]


class SerialRunner:
    def __init__(self, port: str, baudrate: int, line_timeout_sec: float = 0.1) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

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
        self.drain(0.25)
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
                return False, "REJECTED", lines
            if "motion error" in lowered:
                return False, "MOTION_ERROR", lines
            if "homing error" in lowered:
                return False, "HOMING_ERROR", lines
            if any(marker in line for marker in expected):
                return True, "OK", lines
        return False, "TIMEOUT", lines

    def angle_deg(
        self,
        samples: int,
        sample_gap_sec: float,
        settle_sec: float,
        field: str,
        discard_first: int,
        read_retries: int,
        stable_window: int,
        stable_tolerance_deg: float,
        max_reads: int,
    ) -> tuple[bool, float | None, list[str]]:
        values: list[float] = []
        lines: list[str] = []
        if settle_sec > 0.0:
            time.sleep(settle_sec)
        self.drain(0.05)
        required_window = max(1, stable_window)
        total_reads = max(max_reads, samples + discard_first, required_window)
        for index in range(total_reads):
            match = None
            for attempt in range(read_retries + 1):
                ok, code, got = self.send_and_wait("angle", ("ANGLE,",), 2.0, reject_as_error=False)
                lines.extend(got)
                if ok and got:
                    match = ANGLE_RE.search(got[-1])
                    if match is not None and match.group("ok") == "1":
                        break
                if attempt < read_retries:
                    time.sleep(sample_gap_sec)
            if match is None or match.group("ok") != "1":
                if not lines:
                    lines.append("angle read failed")
                return False, None, lines
            values.append(float(match.group(field)))
            if index + 1 > discard_first and len(values) >= required_window:
                recent = values[-required_window:]
                if circular_spread_deg(recent) <= stable_tolerance_deg:
                    return True, circular_mean_deg(recent[-samples:]), lines
            time.sleep(sample_gap_sec)
        if len(values) >= samples:
            return True, circular_mean_deg(values[-samples:]), lines
        return False, None, lines


def main() -> int:
    args = parse_args()
    port = resolve_port(args.port)
    params = build_params(args)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    reports_dir = Path(args.reports_dir)
    reports_dir.mkdir(parents=True, exist_ok=True)
    result_csv = reports_dir / f"as5600_step_loss_sweep_{timestamp}.csv"
    report_md = reports_dir / f"as5600_step_loss_sweep_{timestamp}.md"

    runner = SerialRunner(port, args.baudrate)
    results: list[SweepResult] = []
    try:
        for index, param in enumerate(params, start=1):
            print(
                f"[{index}/{len(params)}] speed={param.speed_mm_s:g} accel={param.accel_mm_s2:g} "
                f"current={args.current_ma} mode={args.chop_mode} microsteps={args.microsteps}",
                flush=True,
            )
            result = run_param(runner, args, param, timestamp)
            results.append(result)
            print(
                f"  => {result.result} reason={result.reason} "
                f"max_error={fmt_optional(result.max_abs_error_mm, 'mm')} "
                f"drift={fmt_optional(result.final_drift_mm, 'mm')}",
                flush=True,
            )
            if result.result != "PASS":
                runner.drain(0.5)
    finally:
        runner.close()

    write_csv(result_csv, results)
    write_report(report_md, result_csv, port, args, results)
    print(f"Report: {report_md}")
    print(f"Results CSV: {result_csv}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto", help="serial port, or auto")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--reports-dir", default="reports")
    parser.add_argument("--speeds", default="100,200,300,400,500", help="comma-separated mm/s")
    parser.add_argument("--accels", default="500,1000,1500,2000,3000", help="comma-separated mm/s^2")
    parser.add_argument("--current-ma", type=int, default=1400)
    parser.add_argument("--chop-mode", choices=("stealth", "spread"), default="spread")
    parser.add_argument("--microsteps", type=int, choices=(8, 16), default=8)
    parser.add_argument("--center-mm", type=float, default=177.5)
    parser.add_argument("--distance-mm", type=float, default=95.0)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--return-speed", type=float, default=50.0)
    parser.add_argument("--command-timeout", type=float, default=20.0)
    parser.add_argument("--homing-timeout", type=float, default=45.0)
    parser.add_argument("--angle-samples", type=int, default=1)
    parser.add_argument("--angle-discard-first", type=int, default=2)
    parser.add_argument("--angle-read-retries", type=int, default=2)
    parser.add_argument("--angle-stable-window", type=int, default=2)
    parser.add_argument("--angle-stable-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--angle-max-reads", type=int, default=10)
    parser.add_argument("--angle-sample-gap", type=float, default=0.03)
    parser.add_argument("--angle-settle-sec", type=float, default=0.3)
    parser.add_argument("--angle-field", choices=("angle", "raw"), default="raw")
    parser.add_argument("--angle-direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--max-error-deg", type=float, default=8.0)
    parser.add_argument("--max-drift-deg", type=float, default=8.0)
    parser.add_argument("--pulley-travel-mm-per-rev", type=float, default=40.0)
    return parser.parse_args()


def build_params(args: argparse.Namespace) -> list[Param]:
    speeds = parse_float_list(args.speeds, "--speeds")
    accels = parse_float_list(args.accels, "--accels")
    return [Param(speed, accel) for speed in speeds for accel in accels]


def parse_float_list(text: str, label: str) -> list[float]:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if not values:
        raise SystemExit(f"{label} must contain at least one value")
    if any(value <= 0.0 for value in values):
        raise SystemExit(f"{label} values must be positive")
    return values


def resolve_port(port: str) -> str:
    if port != "auto":
        return port
    ports: list[str] = []
    try:
        from serial.tools import list_ports  # type: ignore

        ports = [item.device for item in list_ports.comports()]
    except ImportError:
        for pattern in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*"):
            ports.extend(glob.glob(pattern))
    ports = sorted(set(ports))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise SystemExit("No serial ports found. Pass --port /dev/cu... explicitly.")
    raise SystemExit("Multiple serial ports found; pass --port explicitly:\n" + "\n".join(f"  {p}" for p in ports))


def run_param(runner: SerialRunner, args: argparse.Namespace, param: Param, timestamp: str) -> SweepResult:
    started = time.monotonic()
    log: list[str] = []
    legs: list[LegResult] = []

    setup = [
        (f"microstep {args.microsteps}", ("TMC2209 microstep set:", "microstep command ignored:")),
        (f"i {args.current_ma}", ("TMC2209 current set:", "Current command ignored:")),
        (f"mode {args.chop_mode}", ("TMC2209 chop mode set:", "Chop mode command ignored:")),
        ("profile trap", ("Motion profile set:",)),
        (f"a {param.accel_mm_s2:g}", ("Acceleration set:",)),
    ]
    for command, expect in setup:
        ok, code, lines = runner.send_and_wait(command, expect, 5.0)
        log.extend(tag(command, lines))
        if not ok:
            return failed_result(timestamp, args, param, code, started, log, legs)

    ok, code, lines = runner.send_and_wait("h", ("Homing complete",), args.homing_timeout)
    log.extend(tag("h", lines))
    if not ok:
        return failed_result(timestamp, args, param, code, started, log, legs)

    ok, code, lines = runner.send_and_wait(f"m {args.center_mm:g} {args.return_speed:g}", ("Move complete",), args.command_timeout)
    log.extend(tag("m center", lines))
    if not ok:
        return failed_result(timestamp, args, param, code, started, log, legs)

    ok, code, lines = runner.send_and_wait(f"v {param.speed_mm_s:g}", ("Default move speed set:",), 5.0)
    log.extend(tag("v", lines))
    if not ok:
        return failed_result(timestamp, args, param, code, started, log, legs)

    first_angle_ok, first_angle, angle_lines = runner.angle_deg(
        args.angle_samples,
        args.angle_sample_gap,
        args.angle_settle_sec,
        args.angle_field,
        args.angle_discard_first,
        args.angle_read_retries,
        args.angle_stable_window,
        args.angle_stable_tolerance_deg,
        args.angle_max_reads,
    )
    log.extend(tag("angle first", angle_lines))
    if not first_angle_ok or first_angle is None:
        return failed_result(timestamp, args, param, "ANGLE_READ_FAILED", started, log, legs)

    current_angle = first_angle
    for _ in range(args.cycles):
        for distance in (args.distance_mm, -args.distance_mm):
            leg = run_leg(runner, args, distance, current_angle, log)
            if leg is None:
                return failed_result(timestamp, args, param, "MOVE_OR_ANGLE_FAILED", started, log, legs)
            legs.append(leg)
            current_angle = leg.after_deg
            if abs(leg.error_deg) > args.max_error_deg:
                return finish_result(timestamp, args, param, "FAIL", "STEP_ERROR", started, log, legs, first_angle, current_angle)

    return finish_result(timestamp, args, param, "PASS", "OK", started, log, legs, first_angle, current_angle)


def run_leg(
    runner: SerialRunner,
    args: argparse.Namespace,
    distance_mm: float,
    before_deg: float,
    log: list[str],
) -> LegResult | None:
    started = time.monotonic()
    ok, _code, lines = runner.send_and_wait(f"m {distance_mm:g}", ("Move complete",), args.command_timeout)
    elapsed_ms = int(round((time.monotonic() - started) * 1000.0))
    log.extend(tag(f"m {distance_mm:g}", lines))
    if not ok:
        return None
    angle_ok, after_deg, angle_lines = runner.angle_deg(
        args.angle_samples,
        args.angle_sample_gap,
        args.angle_settle_sec,
        args.angle_field,
        args.angle_discard_first,
        args.angle_read_retries,
        args.angle_stable_window,
        args.angle_stable_tolerance_deg,
        args.angle_max_reads,
    )
    log.extend(tag("angle", angle_lines))
    if not angle_ok or after_deg is None:
        return None

    measured_delta = shortest_angle_delta_deg((after_deg - before_deg) * args.angle_direction)
    expected_delta = shortest_angle_delta_deg(distance_mm / args.pulley_travel_mm_per_rev * 360.0)
    error_deg = shortest_angle_delta_deg(measured_delta - expected_delta)
    return LegResult(
        command_mm=distance_mm,
        before_deg=before_deg,
        after_deg=after_deg,
        measured_delta_deg=measured_delta,
        expected_delta_deg=expected_delta,
        error_deg=error_deg,
        error_mm=error_deg / 360.0 * args.pulley_travel_mm_per_rev,
        move_elapsed_ms=elapsed_ms,
    )


def finish_result(
    timestamp: str,
    args: argparse.Namespace,
    param: Param,
    result: str,
    reason: str,
    started: float,
    log: list[str],
    legs: list[LegResult],
    first_angle: float | None,
    last_angle: float | None,
) -> SweepResult:
    max_error_deg = max((abs(leg.error_deg) for leg in legs), default=None)
    max_error_mm = max((abs(leg.error_mm) for leg in legs), default=None)
    drift_deg = None
    drift_mm = None
    if first_angle is not None and last_angle is not None:
        drift_deg = shortest_angle_delta_deg(last_angle - first_angle)
        drift_mm = drift_deg / 360.0 * args.pulley_travel_mm_per_rev
        if result == "PASS" and abs(drift_deg) > args.max_drift_deg:
            result = "FAIL"
            reason = "FINAL_DRIFT"
    return SweepResult(
        timestamp=timestamp,
        speed_mm_s=param.speed_mm_s,
        accel_mm_s2=param.accel_mm_s2,
        current_ma=args.current_ma,
        chop_mode=args.chop_mode,
        microsteps=args.microsteps,
        result=result,
        reason=reason,
        max_abs_error_deg=max_error_deg,
        max_abs_error_mm=max_error_mm,
        final_drift_deg=drift_deg,
        final_drift_mm=drift_mm,
        elapsed_sec=time.monotonic() - started,
        log_excerpt="\n".join(log[-30:]),
        legs=legs,
    )


def failed_result(
    timestamp: str,
    args: argparse.Namespace,
    param: Param,
    reason: str,
    started: float,
    log: list[str],
    legs: list[LegResult],
) -> SweepResult:
    return finish_result(timestamp, args, param, "FAIL", reason, started, log, legs, None, None)


def tag(command: str, lines: list[str]) -> list[str]:
    return [f"[{command}] {line}" for line in lines]


def circular_mean_deg(values: list[float]) -> float:
    sin_sum = sum(math.sin(math.radians(value)) for value in values)
    cos_sum = sum(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0


def circular_spread_deg(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = circular_mean_deg(values)
    return max(abs(shortest_angle_delta_deg(value - mean)) for value in values) * 2.0


def shortest_angle_delta_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def write_csv(path: Path, results: list[SweepResult]) -> None:
    fieldnames = [
        "timestamp",
        "speed_mm_s",
        "accel_mm_s2",
        "current_ma",
        "chop_mode",
        "microsteps",
        "result",
        "reason",
        "max_abs_error_deg",
        "max_abs_error_mm",
        "final_drift_deg",
        "final_drift_mm",
        "elapsed_sec",
        "leg_count",
        "log_excerpt",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for item in results:
            writer.writerow({name: getattr(item, name) for name in fieldnames if name != "leg_count"} | {"leg_count": len(item.legs)})


def write_report(path: Path, csv_path: Path, port: str, args: argparse.Namespace, results: list[SweepResult]) -> None:
    passed = [item for item in results if item.result == "PASS"]
    recommended = choose_recommended(passed)
    lines = [
        "# AS5600 Step Loss Sweep Report",
        "",
        f"- Port: `{port}`",
        f"- Results CSV: `{csv_path}`",
        f"- Conditions: {len(results)}",
        f"- PASS: {len(passed)}",
        f"- FAIL: {len(results) - len(passed)}",
        f"- Move distance: {args.distance_mm:g} mm x {args.cycles} round trips",
        f"- Thresholds: max leg error <= {args.max_error_deg:g} deg, final drift <= {args.max_drift_deg:g} deg",
        "",
        "## Recommended",
        "",
    ]
    if recommended is None:
        lines.append("No passing condition was found.")
    else:
        lines.extend(
            [
                "| speed_mm_s | accel_mm_s2 | current_ma | chop_mode | microsteps | max_abs_error_mm | final_drift_mm |",
                "| --- | --- | --- | --- | --- | --- | --- |",
                f"| {recommended.speed_mm_s:g} | {recommended.accel_mm_s2:g} | {recommended.current_ma} | {recommended.chop_mode} | {recommended.microsteps} | {fmt_cell(recommended.max_abs_error_mm)} | {fmt_cell(recommended.final_drift_mm)} |",
            ]
        )
    lines.extend(
        [
            "",
            "## Results",
            "",
            "| result | speed_mm_s | accel_mm_s2 | max_abs_error_mm | final_drift_mm | reason |",
            "| --- | --- | --- | --- | --- | --- |",
        ]
    )
    for item in sorted(results, key=lambda x: (x.result != "PASS", -x.speed_mm_s, -x.accel_mm_s2)):
        lines.append(
            f"| {item.result} | {item.speed_mm_s:g} | {item.accel_mm_s2:g} | "
            f"{fmt_cell(item.max_abs_error_mm)} | {fmt_cell(item.final_drift_mm)} | {item.reason} |"
        )
    path.write_text("\n".join(lines) + "\n")


def choose_recommended(results: list[SweepResult]) -> SweepResult | None:
    if not results:
        return None
    return sorted(
        results,
        key=lambda item: (
            item.speed_mm_s,
            item.accel_mm_s2,
            -(item.max_abs_error_mm or 0.0),
            -abs(item.final_drift_mm or 0.0),
        ),
        reverse=True,
    )[0]


def fmt_optional(value: float | None, unit: str) -> str:
    if value is None:
        return "NA"
    return f"{value:.4g}{unit}"


def fmt_cell(value: float | None) -> str:
    if value is None:
        return "NA"
    return f"{value:.4g}"


if __name__ == "__main__":
    raise SystemExit(main())
