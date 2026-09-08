# notebooks

This folder is where I look at real data from the servo rig. The firmware
streams raw ADC samples at the full 20 kHz control rate, the captures get
checked in under `telemetry/`, and the notebooks load them, plot them, and
check the control math against what the hardware actually did.

These notebooks are for understanding and for writeups, not for CI. They are
committed with their outputs baked in, so they read as documentation straight
from GitHub.

## Setup

You will need [uv](https://docs.astral.sh/uv/). Then:

```
cd notebooks
uv sync
uv run jupyter lab
```

`uv sync` creates a `.venv` and installs numpy, scipy, pandas, matplotlib, and
jupyterlab. Python is pinned to 3.12 in `.python-version`, and uv will fetch it
for you if you don't have it.

## Telemetry

The captures live under `telemetry/`, one folder per servo, then one folder
per campaign, then one folder per recording:

```
telemetry/
  sg90/
    usb/
      capture-1/
        meta.json
        sweep.csv.gz
    2s/
      capture-1/
        ...
```

Every recording is raw ADC counts straight off the wire, with a per-sample
tick counter, so missing samples are provable from the data itself. The
constants needed to turn counts into volts and amps are in each notebook.

## The notebooks

- `00-raw-duty-sweeps` characterizes the measurement chain and the open-loop
  response from the duty-sweep campaigns. It is the foundation the rest of
  the series builds on.
