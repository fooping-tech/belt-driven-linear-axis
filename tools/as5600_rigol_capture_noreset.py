#!/usr/bin/env python3
"""Capture AS5600 I2C after serial boot has settled."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import time
from pathlib import Path
from typing import Any

import pyvisa
import serial


SCREENSHOT_COMMAND = ":DISP:DATA? ON,OFF,PNG"


def timestamp() -> str:
  return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def read_ieee_block(scope: Any, command: str) -> bytes:
  scope.write(command)
  raw = bytes(scope.read_raw())
  if not raw.startswith(b"#"):
    raise RuntimeError(f"{command} did not return an IEEE block: {raw[:16]!r}")
  digits = int(chr(raw[1]))
  header_len = 2 + digits
  while len(raw) < header_len:
    raw += bytes(scope.read_raw())
  payload_len = int(raw[2:header_len].decode("ascii"))
  total_len = header_len + payload_len
  while len(raw) < total_len:
    raw += bytes(scope.read_raw())
  return raw[header_len:total_len]


def configure_scope(scope: Any, trigger_channel: str, time_scale: float) -> list[str]:
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
      f":TRIG:EDGE:SOUR {trigger_channel}",
      ":TRIG:EDGE:SLOP NEG",
      ":TRIG:EDGE:LEV 1.5",
      ":TRIG:SWE SING",
  ]
  for command in commands:
    print(command)
    scope.write(command)
  return commands


def drain_serial(ser: serial.Serial, wait_s: float) -> list[str]:
  lines: list[str] = []
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
  tag = f"as5600_noreset_{timestamp()}"

  rm = pyvisa.ResourceManager("@py")
  scope = rm.open_resource(f"TCPIP0::{args.ip}::INSTR")
  scope.timeout = args.timeout_ms
  try:
    idn = str(scope.query("*IDN?")).strip()
    print(f"*IDN?: {idn}")
    with serial.Serial(args.port, args.baud, timeout=0.2, dsrdtr=False, rtscts=False) as ser:
      ser.dtr = False
      ser.rts = False
      boot_lines = drain_serial(ser, args.boot_wait)
      commands = configure_scope(scope, args.trigger_channel, args.time_scale)
      scope.write(":SING")
      time.sleep(0.2)
      ser.reset_input_buffer()
      ser.write((args.command + "\n").encode("ascii"))
      ser.flush()
      serial_lines = drain_serial(ser, args.serial_wait)
      time.sleep(args.post_wait)
      scope.write(":STOP")

    png = read_ieee_block(scope, SCREENSHOT_COMMAND)
    png_path = outdir / f"{tag}.png"
    png_path.write_bytes(png)
    setup_path = outdir / f"{tag}_setup.json"
    setup_path.write_text(
        json.dumps(
            {
                "idn": idn,
                "channels": {"CHAN1": "AS5600 VDD", "CHAN2": "AS5600 SCL", "CHAN3": "AS5600 SDA"},
                "trigger_channel": args.trigger_channel,
                "trigger_level_v": 1.5,
                "time_scale_s_per_div": args.time_scale,
                "boot_lines": boot_lines,
                "serial_lines": serial_lines,
                "serial_command": args.command,
                "commands": commands + [":SING", ":STOP", SCREENSHOT_COMMAND],
                "screenshot_png": str(png_path),
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
  parser.add_argument("--trigger-channel", default="CHAN2")
  parser.add_argument("--command", default="as5600")
  parser.add_argument("--time-scale", type=float, default=0.0001)
  parser.add_argument("--boot-wait", type=float, default=2.0)
  parser.add_argument("--serial-wait", type=float, default=1.5)
  parser.add_argument("--post-wait", type=float, default=0.5)
  parser.add_argument("--timeout-ms", type=int, default=20000)
  return parser.parse_args()


def main() -> int:
  path = run(parse_args())
  print(path)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
