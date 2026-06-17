# AGENTS.md

## Sample Program Policy

This repository will keep adding small PlatformIO sample programs over time.
Add each sample as an independent PlatformIO project under:

```text
sample_programs/<sample_name>/
```

Use lowercase snake_case for `<sample_name>`.
For the current one-stepper example, use:

```text
sample_programs/1stepper_simple/
```

## Required Sample Structure

Each sample program must include at least:

```text
sample_programs/<sample_name>/
  platformio.ini
  src/
    main.cpp
  README.md
```

The sample must be buildable from its own directory with PlatformIO.
Do not make a sample depend on files from the repository root unless that
dependency is intentionally documented in the sample README.

## Root Project Handling

Keep the root PlatformIO project as the working and current verification
project. When creating a sample from the current root program, copy the files
into the sample directory by default.

Do not remove or break the root `src/main.cpp`, `platformio.ini`, or
`README.md` when adding a new sample unless the user explicitly requests it.

## Sample README Requirements

Every sample `README.md` must describe:

- The purpose of the sample.
- Supported board or boards.
- Required wiring.
- How to operate the sample.
- How to build, upload, and open the serial monitor.

For PlatformIO commands, prefer:

```sh
pio run
pio run -t upload
pio device monitor
```

If the sample uses a non-default environment, show the exact `-e <env_name>`
commands needed.

## Verification

After adding or changing a sample, run the build from that sample directory:

```sh
pio run
```

When hardware is connected and an upload port is available, also upload the
firmware and test the running device:

```sh
pio run -t upload
pio device monitor
```

Use the serial monitor, debugger, or both to verify the behavior that changed.
For UART, SPI, I2C, STEP/DIR, PWM, or other signal-level issues, capture the
relevant startup logs and runtime status before finishing. If software logs are
not enough and an oscilloscope capability is available, use it to measure the
waveform and troubleshoot the physical signal path.

When running real-hardware experiments, leave a Markdown record under
`reports/` before finishing. Prefer appending to the existing report for the
same investigation; otherwise create a new dated report. Record at least:

- The hardware setup and any mechanical/electrical changes.
- Firmware/config parameters used for the run.
- Commands executed and the important serial output.
- Pass/fail result, observed behavior, and stop/fault reason.
- Diagnosis, parameter changes made, and the next planned experiment.

If the root project was also changed, run `pio run` from the repository root as
well.

Before finishing, check that the sample directory contains the required files
and that the README gives enough information to wire, operate, build, upload,
and monitor the example without reading the root project.
