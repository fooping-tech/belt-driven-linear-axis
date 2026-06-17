#!/usr/bin/env python3
"""Collect real-hardware balance logs from the firmware serial interface."""

from __future__ import annotations

import argparse
import csv
import glob
import re
import sys
import time
from pathlib import Path


BALANCE_FIELDS = [
    "mode",
    "x_m",
    "xdot_mps",
    "phi_rad",
    "phidot_rad_s",
    "cmd_x_m",
    "cmd_v_mps",
    "accel_mps2",
    "cmd_lag_m",
]


def parse_key_values(line: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for item in line.split(",")[1:]:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def auto_port() -> str:
    ports: list[str] = []
    for pattern in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        ports.extend(glob.glob(pattern))
    ports = sorted(set(ports))
    if not ports:
        raise SystemExit("No serial ports found. Pass --port explicitly.")
    if len(ports) > 1:
        raise SystemExit("Multiple serial ports found. Pass --port explicitly:\n" + "\n".join(ports))
    return ports[0]


class SerialClient:
    def __init__(self, port: str, baudrate: int):
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise SystemExit("pyserial is required. Use PlatformIO's Python or install pyserial.") from exc
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def send(self, command: str) -> None:
        print(f">>> {command}", flush=True)
        self.ser.write((command + "\n").encode("utf-8"))
        self.ser.flush()

    def read_line(self) -> str | None:
        raw = self.ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def command_wait(self, command: str, timeout_s: float, stop_patterns: list[str]) -> tuple[list[str], str | None]:
        self.ser.reset_input_buffer()
        self.send(command)
        patterns = [re.compile(pattern) for pattern in stop_patterns]
        deadline = time.time() + timeout_s
        lines: list[str] = []
        stop_line: str | None = None
        while time.time() < deadline:
            line = self.read_line()
            if not line:
                continue
            print(line, flush=True)
            lines.append(line)
            if any(pattern.search(line) for pattern in patterns):
                stop_line = line
                break
        return lines, stop_line


def write_csv(path: Path, rows: list[dict[str, str | float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["sample", "t_s", *BALANCE_FIELDS]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def analyze_rows(rows: list[dict[str, str | float | int]]) -> dict[str, float | int | str]:
    if not rows:
        return {
            "samples": 0,
            "lqr_count": 0,
            "min_abs_phi_rad": float("nan"),
            "max_abs_phi_rad": float("nan"),
            "max_abs_x_m": float("nan"),
            "max_abs_cmd_lag_m": float("nan"),
            "mean_abs_cmd_lag_m": float("nan"),
            "duration_s": 0.0,
        }
    phis = [abs(float(row["phi_rad"])) for row in rows]
    xs = [abs(float(row["x_m"])) for row in rows]
    lags = [abs(float(row["cmd_lag_m"])) for row in rows]
    return {
        "samples": len(rows),
        "lqr_count": sum(1 for row in rows if row["mode"] == "lqr"),
        "min_abs_phi_rad": min(phis),
        "max_abs_phi_rad": max(phis),
        "max_abs_x_m": max(xs),
        "max_abs_cmd_lag_m": max(lags),
        "mean_abs_cmd_lag_m": sum(lags) / len(lags),
        "duration_s": float(rows[-1]["t_s"]) if rows else 0.0,
    }


def write_report(
    path: Path,
    csv_path: Path,
    port: str,
    args: argparse.Namespace,
    summary: dict[str, float | int | str],
    setup_lines: list[str],
    stop_line: str | None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Real Balance Log",
        "",
        f"- CSV: `{csv_path}`",
        f"- Serial port: `{port}`",
        f"- Duration request: `{args.duration_s:.1f} s`",
        f"- Center command: `{args.center_mm:.3f} mm`",
        f"- Center move speed: `{args.center_speed_mm_s:.1f} mm/s`",
        f"- Stop line: `{stop_line or 'manual balstop/user_stop or none'}`",
        "",
        "## Summary",
        "",
        "| item | value |",
        "| --- | --- |",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            lines.append(f"| {key} | {value:.6g} |")
        else:
            lines.append(f"| {key} | {value} |")
    lines.extend([
        "",
        "## Setup Serial Output",
        "",
        "```text",
        *setup_lines[-80:],
        "```",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def collect(args: argparse.Namespace) -> tuple[Path, Path, dict[str, float | int | str]]:
    port = auto_port() if args.port == "auto" else args.port
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.reports_dir)
    csv_path = out_dir / f"real_balance_log_{timestamp}.csv"
    report_path = out_dir / f"real_balance_log_{timestamp}.md"

    client = SerialClient(port, args.baudrate)
    setup_lines: list[str] = []
    rows: list[dict[str, str | float | int]] = []
    stop_line: str | None = None
    try:
        for command, timeout, stops in [
            ("tmcv", 3.0, [r"TMC validate (OK|FAIL)"]),
            ("s", 2.0, [r"tmc2209_uart="]),
            ("h", args.homing_timeout_s, [r"Homing complete", r"Homing rejected", r"Homing failed", r"Error"]),
            (f"m {args.center_mm:g} {args.center_speed_mm_s:g}", args.move_timeout_s, [r"Move complete", r"Move rejected", r"Motion error", r"Error"]),
        ]:
            lines, stop = client.command_wait(command, timeout, stops)
            setup_lines.extend(lines)
            if command == "h" and (not stop or "Homing complete" not in stop):
                raise SystemExit("Homing failed or timed out")
            if command.startswith("m ") and (not stop or "Move complete" not in stop):
                raise SystemExit("Center move failed or timed out")

        time.sleep(args.settle_s)
        for _ in range(args.angle_checks):
            lines, _ = client.command_wait("angle", 1.5, [r"ANGLE,"])
            setup_lines.extend(lines)
            time.sleep(0.2)
        lines, _ = client.command_wait("as5600", 2.0, [r"AS5600,"])
        setup_lines.extend(lines)
        lines, stop = client.command_wait("balzero", 3.0, [r"BALANCE_ZERO,samples=", r"BALANCE_ZERO,rejected", r"BALANCE,rejected"])
        setup_lines.extend(lines)
        if not stop or "BALANCE_ZERO,samples=" not in stop:
            raise SystemExit("balzero failed")
        lines, _ = client.command_wait("bal", 2.0, [r"BALANCE_STATUS"])
        setup_lines.extend(lines)

        client.ser.reset_input_buffer()
        client.send("balstart")
        start = time.time()
        sample = 0
        while time.time() - start < args.duration_s:
            line = client.read_line()
            if not line:
                continue
            print(line, flush=True)
            if line.startswith("BALANCE_STATUS,mode="):
                values = parse_key_values(line)
                if all(field in values for field in BALANCE_FIELDS):
                    row: dict[str, str | float | int] = {"sample": sample, "t_s": time.time() - start}
                    for field in BALANCE_FIELDS:
                        row[field] = values[field] if field == "mode" else float(values[field])
                    rows.append(row)
                    sample += 1
            if "BALANCE,rejected" in line or "BALANCE,stop" in line:
                stop_line = line
                break

        if stop_line is None:
            client.send("balstop")
            deadline = time.time() + 2.0
            while time.time() < deadline:
                line = client.read_line()
                if not line:
                    continue
                print(line, flush=True)
                if "BALANCE,stop" in line:
                    stop_line = line
                    break
    finally:
        client.close()

    write_csv(csv_path, rows)
    summary = analyze_rows(rows)
    write_report(report_path, csv_path, port, args, summary, setup_lines, stop_line)
    return csv_path, report_path, summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--reports-dir", default="reports")
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--center-mm", type=float, default=177.5)
    parser.add_argument("--center-speed-mm-s", type=float, default=50.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--angle-checks", type=int, default=3)
    parser.add_argument("--homing-timeout-s", type=float, default=30.0)
    parser.add_argument("--move-timeout-s", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path, report_path, summary = collect(args)
    print(f"CSV: {csv_path}")
    print(f"Report: {report_path}")
    print("Summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
