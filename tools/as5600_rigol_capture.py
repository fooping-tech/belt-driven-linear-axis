#!/usr/bin/env python3
"""Capture AS5600 VDD/SCL/SDA waveforms with a RIGOL DS1104Z."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import time
from pathlib import Path
from typing import Any

import pyvisa
import serial


SCREENSHOT_COMMAND = ":DISP:DATA? ON,OFF,PNG"
CHANNELS = ("CHAN1", "CHAN2", "CHAN3")
PREAMBLE_KEYS = (
    "format",
    "type",
    "points",
    "count",
    "xincrement",
    "xorigin",
    "xreference",
    "yincrement",
    "yorigin",
    "yreference",
)


def timestamp() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def open_scope(ip: str, timeout_ms: int) -> tuple[Any, Any, str]:
    resource = f"TCPIP0::{ip}::INSTR"
    rm = pyvisa.ResourceManager("@py")
    scope = rm.open_resource(resource)
    scope.timeout = timeout_ms
    return rm, scope, resource


def write(scope: Any, command: str) -> None:
    print(command)
    scope.write(command)


def read_ieee_block(scope: Any, command: str) -> bytes:
    scope.write(command)
    raw = bytes(scope.read_raw())
    if not raw.startswith(b"#"):
        raise RuntimeError(f"{command} did not return an IEEE binary block: {raw[:16]!r}")
    digits = int(chr(raw[1]))
    header_len = 2 + digits
    while len(raw) < header_len:
        raw += bytes(scope.read_raw())
    payload_len = int(raw[2:header_len].decode("ascii"))
    total_len = header_len + payload_len
    while len(raw) < total_len:
        raw += bytes(scope.read_raw())
    return raw[header_len:total_len]


def parse_preamble(text: str) -> dict[str, float | int]:
    parts = [part.strip() for part in text.strip().split(",")]
    if len(parts) < len(PREAMBLE_KEYS):
        raise ValueError(f"Unexpected :WAV:PRE? format: {text!r}")
    parsed: dict[str, float | int] = {}
    for key, value in zip(PREAMBLE_KEYS, parts):
        if key in {"format", "type", "points", "count"}:
            parsed[key] = int(float(value))
        else:
            parsed[key] = float(value)
    return parsed


def convert_sample(index: int, adc: int, preamble: dict[str, float | int]) -> tuple[float, float]:
    time_s = (index - float(preamble["xreference"])) * float(preamble["xincrement"]) + float(preamble["xorigin"])
    voltage_v = (adc - float(preamble["yorigin"]) - float(preamble["yreference"])) * float(preamble["yincrement"])
    return time_s, voltage_v


def configure_scope(scope: Any, time_scale: float) -> list[str]:
    commands = [
        ":STOP",
        ":CHAN1:DISP ON",
        ":CHAN1:COUP DC",
        ":CHAN1:PROB 10",
        ":CHAN1:SCAL 1",
        ":CHAN1:OFFS -1.5",
        ":CHAN2:DISP ON",
        ":CHAN2:COUP DC",
        ":CHAN2:PROB 10",
        ":CHAN2:SCAL 1",
        ":CHAN2:OFFS -1.5",
        ":CHAN3:DISP ON",
        ":CHAN3:COUP DC",
        ":CHAN3:PROB 10",
        ":CHAN3:SCAL 1",
        ":CHAN3:OFFS -1.5",
        ":CHAN4:DISP OFF",
        f":TIM:SCAL {time_scale:g}",
        ":TIM:OFFS 0",
        ":TRIG:MODE EDGE",
        ":TRIG:EDGE:SOUR CHAN2",
        ":TRIG:EDGE:SLOP NEG",
        ":TRIG:EDGE:LEV 1.5",
        ":TRIG:SWE SING",
    ]
    for command in commands:
        write(scope, command)
    return commands


def save_waveform(scope: Any, channel: str, outdir: Path, tag: str) -> tuple[Path, Path]:
    write(scope, ":STOP")
    write(scope, f":WAV:SOUR {channel}")
    write(scope, ":WAV:MODE NORM")
    write(scope, ":WAV:FORM BYTE")
    preamble_text = str(scope.query(":WAV:PRE?")).strip()
    preamble = parse_preamble(preamble_text)
    payload = read_ieee_block(scope, ":WAV:DATA?")

    csv_path = outdir / f"{tag}_{channel}.csv"
    json_path = outdir / f"{tag}_{channel}.json"
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["time_s", "voltage_v", "adc"])
        for index, adc in enumerate(payload):
            time_s, voltage_v = convert_sample(index, adc, preamble)
            writer.writerow([f"{time_s:.12g}", f"{voltage_v:.12g}", adc])

    json_path.write_text(
        json.dumps(
            {
                "channel": channel,
                "preamble_raw": preamble_text,
                "preamble": preamble,
                "sample_count": len(payload),
                "csv": str(csv_path),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return csv_path, json_path


def send_as5600_command(port: str, baud: int, wait_s: float) -> list[str]:
    lines: list[str] = []
    with serial.Serial(port=port, baudrate=baud, timeout=0.2, dsrdtr=False, rtscts=False) as ser:
        ser.dtr = False
        ser.rts = False
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b"as5600\n")
        ser.flush()
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            raw = ser.readline()
            if raw:
                text = raw.decode("utf-8", errors="replace").rstrip()
                print(f"SERIAL: {text}")
                lines.append(text)
    return lines


def run(args: argparse.Namespace) -> Path:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    tag = f"as5600_rigol_{timestamp()}"

    rm, scope, resource = open_scope(args.ip, args.timeout_ms)
    try:
        idn = str(scope.query("*IDN?")).strip()
        print(f"*IDN?: {idn}")
        commands = configure_scope(scope, args.time_scale)
        write(scope, ":SING")
        time.sleep(0.2)
        serial_lines = send_as5600_command(args.port, args.baud, args.serial_wait)
        time.sleep(args.post_wait)
        write(scope, ":STOP")

        png = read_ieee_block(scope, SCREENSHOT_COMMAND)
        png_path = outdir / f"{tag}.png"
        png_path.write_bytes(png)

        waveform_paths = []
        for channel in CHANNELS:
            csv_path, json_path = save_waveform(scope, channel, outdir, tag)
            waveform_paths.append({"channel": channel, "csv": str(csv_path), "json": str(json_path)})

        setup_path = outdir / f"{tag}_setup.json"
        setup_path.write_text(
            json.dumps(
                {
                    "purpose": "AS5600 VDD/SCL/SDA I2C failure capture",
                    "idn": idn,
                    "resource": resource,
                    "serial_port": args.port,
                    "serial_baud": args.baud,
                    "channels": {
                        "CHAN1": "AS5600 VDD",
                        "CHAN2": "AS5600 SCL",
                        "CHAN3": "AS5600 SDA",
                    },
                    "trigger": {"source": "CHAN2", "slope": "falling", "level_v": 1.5},
                    "probe": 10,
                    "coupling": "DC",
                    "time_scale_s_per_div": args.time_scale,
                    "commands": commands + [":SING", ":STOP", SCREENSHOT_COMMAND],
                    "serial_lines": serial_lines,
                    "screenshot_png": str(png_path),
                    "waveforms": waveform_paths,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"Saved setup: {setup_path}")
        return setup_path
    finally:
        try:
            scope.close()
        finally:
            rm.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--outdir", default="captures/as5600")
    parser.add_argument("--time-scale", type=float, default=0.0001)
    parser.add_argument("--serial-wait", type=float, default=1.5)
    parser.add_argument("--post-wait", type=float, default=0.5)
    parser.add_argument("--timeout-ms", type=int, default=20000)
    return parser.parse_args()


def main() -> int:
    setup_path = run(parse_args())
    print(setup_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
