# How to run

```bash
git clone https://github.com/GHOSTALERT/adaptive-speed-control.git
cd adaptive-speed-control
```

## Python re-run (no MATLAB needed)

Needs Python 3.10+.

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

.venv/bin/python python/speed_control.py      # results/*.csv, results/comparison.md, results/figures/*.png
.venv/bin/python -m pytest -q tests           # 13 checks against the report's values
```

If your Python has no `pip` inside new virtual environments (Debian/Ubuntu without `python3-venv`), use:

```bash
python3 -m venv --without-pip .venv
python3 -m pip --python .venv/bin/python install -r requirements.txt
```

## MATLAB (Control System Toolbox, Simulink)

```matlab
cd matlab
speed_control            % prints metrics and margins, draws Figures 1-10, saves PNGs to results/matlab/
build_simulink_model     % builds adaptive_speed_control.slx
sim('adaptive_speed_control')
```

The Simulink model has three parallel PID loops (Smooth, Normal, Aggressive) on one scope. A fourth loop takes its gains from a Multiport Switch driven by a `Profile` constant (1/2/3).

These two MATLAB scripts mirror the code in the report. They have not been run on the machine this repo was rebuilt on, because MATLAB is not installed there. The Python re-run reproduces every number.
