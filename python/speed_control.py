"""Adaptive driver-behaviour-aware vehicle speed control: Python re-run of the MATLAB study.

Plant m*dv/dt = u - b*v with m = 1200 kg, b = 50 N*s/m, so G(s) = 1/(1200 s + 50).
Fixed P / PI / PID, gain-scheduled PID for three driver profiles, state feedback
by pole placement, margins, root locus, Nyquist and disturbance rejection.

    .venv/bin/python python/speed_control.py            # figures + CSV + comparison
    .venv/bin/python python/speed_control.py --no-plots # numbers only

Outputs go to results/: metrics.csv, margins.csv, disturbance.csv, comparison.md, figures/.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import control as ct
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"

# Plant
M, B = 1200.0, 50.0
V_REF = 30.0                      # m/s (108 km/h)
G = ct.tf([1.0], [M, B])

# Fixed controllers (report section "Controller Design")
KP_P = 100.0
KP_PI, KI_PI = 120.0, 10.0
KP_PID, KI_PID, KD_PID = 150.0, 20.0, 5.0

# Gain schedule per driver profile
PROFILES = {
    "Smooth":     {"Kp": 80.0,  "Ki": 5.0,  "Kd": 2.0},
    "Normal":     {"Kp": 150.0, "Ki": 20.0, "Kd": 5.0},
    "Aggressive": {"Kp": 200.0, "Ki": 30.0, "Kd": 10.0},
}

DESIRED_POLE = -5.0               # state-feedback pole
D_AMP = 5.0                       # disturbance amplitude at the plant input

T_END = 200.0
T = np.linspace(0.0, T_END, 200_001)

# Values printed in the submitted report, for the comparison table
REPORT_METRICS = {
    "Fixed P":          (17.58, 31.30, 0.00, 58.58),
    "Fixed PI":         (13.67, 55.03, 7.87, 31.21),
    "Fixed PID":        (9.00, 38.87, 14.76, 20.53),
    "Adapt-Smooth":     (23.09, 72.85, 3.57, 51.10),
    "Adapt-Normal":     (9.00, 38.87, 14.76, 20.53),
    "Adapt-Aggressive": (6.97, 31.55, 15.51, 16.49),
}
REPORT_MARGINS = {"P": (120.00, 0.0722), "PI": (74.06, 0.1159), "PID": (64.77, 0.1578)}


def pid(kp: float, ki: float, kd: float) -> ct.TransferFunction:
    """Parallel PID C(s) = (Kd s^2 + Kp s + Ki) / s."""
    return ct.tf([kd, kp, ki], [1.0, 0.0])


def controllers() -> dict[str, ct.TransferFunction]:
    c = {
        "Fixed P": ct.tf([KP_P], [1.0]),
        "Fixed PI": ct.tf([KP_PI, KI_PI], [1.0, 0.0]),
        "Fixed PID": pid(KP_PID, KI_PID, KD_PID),
    }
    for name, g in PROFILES.items():
        c[f"Adapt-{name}"] = pid(g["Kp"], g["Ki"], g["Kd"])
    return c


def closed_loop(c: ct.TransferFunction) -> ct.TransferFunction:
    return ct.feedback(c * G, 1)


def step_metrics(sys: ct.TransferFunction) -> dict[str, float]:
    """MATLAB stepinfo equivalents: 10-90 % rise, 2 % settling, overshoot, peak time."""
    t, y = ct.step_response(V_REF * sys, T)
    info = ct.step_info(y, t)
    return {
        "rise": info["RiseTime"],
        "settling": info["SettlingTime"],
        "overshoot": info["Overshoot"],
        "peak_time": info["PeakTime"],
        "final": float(y[-1]),
        "ess": V_REF - float(y[-1]),
    }


def state_feedback() -> dict[str, float]:
    """Single-state model x = v: A = -b/m, B = 1/m, C = 1, D = 0; place the pole at DESIRED_POLE."""
    a, bb, c = -B / M, 1.0 / M, 1.0
    k = (a - DESIRED_POLE) / bb           # place() for a scalar system
    a_cl = a - bb * k
    nbar = -1.0 / (c * (1.0 / a_cl) * bb)
    return {"A": a, "B": bb, "K": k, "A_cl": a_cl, "Nbar": nbar,
            "controllable": bb != 0, "observable": c != 0}


def state_feedback_tf() -> ct.TransferFunction:
    sf = state_feedback()
    return ct.ss(sf["A_cl"], sf["B"] * sf["Nbar"], 1.0, 0.0)


def margins(c: ct.TransferFunction) -> dict[str, float]:
    gm, pm, wpc, wgc = ct.margin(c * G)
    return {"gm_db": np.inf if not np.isfinite(gm) else 20 * np.log10(gm),
            "pm_deg": pm, "w_gc": wgc, "w_pc": wpc}


def disturbance(c: ct.TransferFunction, kind: str) -> dict[str, float]:
    """Speed deviation for a disturbance D_AMP entering at the plant input.

    S(s) = G / (1 + C G). kind = "impulse" is what the report's code runs
    (impulse(d_amp * S, t)); kind = "step" is a sustained headwind.
    """
    s = ct.feedback(G, c)
    t = np.linspace(0.0, 120.0, 120_001)
    if kind == "impulse":
        t, y = ct.impulse_response(D_AMP * s, t)
    else:
        t, y = ct.step_response(D_AMP * s, t)
    peak = float(np.max(np.abs(y)))
    final = float(y[-1])
    band = 0.02 * peak
    outside = np.nonzero(np.abs(y - final) > band)[0]
    recover = float(t[outside[-1]]) if outside.size else 0.0
    return {"peak": peak, "final": final, "recovery_2pct": recover, "t": t, "y": y}


def driver_reference(profile: str, t: np.ndarray) -> np.ndarray:
    """Reference speed seen by each profile in Figure 2.

    Smooth ramps to 30 m/s over 20 s, Normal is a step, Aggressive is a step
    with a +25 % burst from t = 20 s to t = 22 s.
    """
    if profile == "Smooth":
        return np.minimum(t / 20.0, 1.0) * V_REF
    r = np.full_like(t, V_REF)
    if profile == "Aggressive":
        r[(t >= 20.0) & (t < 22.0)] *= 1.25
    return r


# ---------------------------------------------------------------- outputs

def write_csv(path: Path, header: list[str], rows: list[list]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def fmt(x: float, nd: int = 2) -> str:
    return "Inf" if not np.isfinite(x) else f"{x:.{nd}f}"


def run(plots: bool = True) -> dict:
    ctrl = controllers()
    metrics = {name: step_metrics(closed_loop(c)) for name, c in ctrl.items()}
    sf = state_feedback()
    metrics["State feedback"] = step_metrics(state_feedback_tf())

    marg = {k: margins(ctrl[f"Fixed {k}"]) for k in ("P", "PI", "PID")}
    for name in PROFILES:
        marg[f"Adapt-{name}"] = margins(ctrl[f"Adapt-{name}"])

    dist = {(k, kind): disturbance(ctrl[f"Fixed {k}"], kind)
            for k in ("P", "PI", "PID") for kind in ("impulse", "step")}

    write_csv(RESULTS / "metrics.csv",
              ["controller", "rise_s", "settling_s", "overshoot_pct", "peak_time_s", "final_mps", "ess_mps"],
              [[n, fmt(m["rise"]), fmt(m["settling"]), fmt(m["overshoot"]), fmt(m["peak_time"]),
                fmt(m["final"]), fmt(m["ess"], 3)] for n, m in metrics.items()])
    write_csv(RESULTS / "margins.csv", ["controller", "gm_db", "pm_deg", "w_gc_rad_s", "w_pc_rad_s"],
              [[n, fmt(m["gm_db"]), fmt(m["pm_deg"]), fmt(m["w_gc"], 4), fmt(m["w_pc"], 4)]
               for n, m in marg.items()])
    write_csv(RESULTS / "disturbance.csv", ["controller", "kind", "peak_dev_mps", "final_dev_mps", "recovery_2pct_s"],
              [[k, kind, f"{d['peak']:.5f}", f"{d['final']:.5f}", fmt(d["recovery_2pct"])]
               for (k, kind), d in dist.items()])

    write_comparison(metrics, marg, sf, dist)
    if plots:
        make_figures(ctrl, metrics, dist)
    return {"metrics": metrics, "margins": marg, "state_feedback": sf, "disturbance": dist}


def write_comparison(metrics, marg, sf, dist) -> None:
    zeros = np.sort(np.roots([KD_PID, KP_PID, KI_PID]))
    lines = [
        "# Re-run vs. report",
        "",
        "Generated by `python/speed_control.py`. Step metrics follow MATLAB `stepinfo` "
        "(10-90 % rise, 2 % settling) on a 30 m/s step, simulated to 200 s.",
        "",
        "## Step response",
        "",
        "| Controller | Rise (s) report / re-run | Settling (s) | Overshoot (%) | Peak time (s) |",
        "|---|---|---|---|---|",
    ]
    for name, (tr, ts, os_, tp) in REPORT_METRICS.items():
        m = metrics[name]
        lines.append(f"| {name} | {tr:.2f} / {m['rise']:.2f} | {ts:.2f} / {m['settling']:.2f} | "
                     f"{os_:.2f} / {m['overshoot']:.2f} | {tp:.2f} / {m['peak_time']:.2f} |")
    m = metrics["State feedback"]
    lines += [
        f"| State feedback | - / {m['rise']:.2f} | - / {m['settling']:.2f} | - / {m['overshoot']:.2f} | - |",
        "",
        "Peak times differ slightly because MATLAB `step` picks its own time grid. The P loop never "
        "overshoots, so its \"peak\" is just the end of the simulated window (58.58 s in MATLAB, "
        f"{metrics['Fixed P']['peak_time']:.0f} s here). The P loop settles at "
        f"{metrics['Fixed P']['final']:.2f} m/s (e_ss = b/(Kp+b) = {B / (KP_P + B):.3f}).",
        "",
        "## Margins",
        "",
        "| Controller | PM (deg) report / re-run | Gain crossover (rad/s) report / re-run | GM |",
        "|---|---|---|---|",
    ]
    for k, (pm, wgc) in REPORT_MARGINS.items():
        mm = marg[k]
        lines.append(f"| {k} | {pm:.2f} / {mm['pm_deg']:.2f} | {wgc:.4f} / {mm['w_gc']:.4f} | {fmt(mm['gm_db'])} |")
    lines += [
        "",
        "## State feedback",
        "",
        f"A = {sf['A']:.4f}, B = {sf['B']:.3e}, K = {sf['K']:.0f} (report 5950), A_cl = {sf['A_cl']:.2f}, "
        f"Nbar = {sf['Nbar']:.0f} (report 6000). PID zeros: {zeros[0]:.2f} and {zeros[1]:.3f} "
        "(report -29.9 and -0.133).",
        "",
        "## Disturbance rejection",
        "",
        f"Disturbance of amplitude {D_AMP:g} at the plant input, S(s) = G/(1+CG).",
        "",
        "| Controller | Impulse: peak dev. (m/s) | Impulse: final | Step: peak dev. (m/s) | Step: final |",
        "|---|---|---|---|---|",
    ]
    for k in ("P", "PI", "PID"):
        i, s = dist[(k, "impulse")], dist[(k, "step")]
        lines.append(f"| {k} | {i['peak']:.5f} | {i['final']:.1e} | {s['peak']:.5f} | {s['final']:.5f} |")
    lines += [
        "",
        "The report's code applies `impulse(d_amp * S, t)`. Because d enters as a force, the speed "
        "deviation it produces is small (about 4 mm/s for P, not 4 m/s), and every stable loop, P "
        "included, returns to zero after an impulse. The behaviour described in the report's text "
        "(P keeps a steady offset, PI and PID return to zero) is what a sustained (step) "
        "disturbance produces, so both are tabulated and plotted in Figure 9.",
        "",
    ]
    (RESULTS / "comparison.md").write_text("\n".join(lines))


def make_figures(ctrl, metrics, dist) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig_dir = RESULTS / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    t = np.linspace(0.0, 150.0, 15_001)

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(fig_dir / name, dpi=130)
        plt.close(fig)

    def step(name):
        return ct.step_response(V_REF * closed_loop(ctrl[name]), t)

    # 1. P vs PI vs PID
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name, style, col in (("Fixed P", "--", "tab:red"), ("Fixed PI", "-.", "tab:blue"),
                             ("Fixed PID", "-", "tab:green")):
        ax.plot(*step(name), style, color=col, lw=2, label=name.split()[1])
    ax.axhline(V_REF, color="k", lw=0.8, ls=":")
    ax.set(title="Step response: P vs PI vs PID (fixed gains)", xlabel="Time (s)", ylabel="Speed (m/s)")
    ax.legend(); ax.grid(alpha=0.3)
    save(fig, "fig01_step_p_pi_pid.png")

    # 2. Adaptive profiles with their own reference signals
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name, col in (("Smooth", "tab:blue"), ("Normal", "tab:green"), ("Aggressive", "tab:red")):
        r = driver_reference(name, t)
        _, y = ct.forced_response(closed_loop(ctrl[f"Adapt-{name}"]), t, r)
        ax.plot(t, y, color=col, lw=2, label=f"{name} (Kp={PROFILES[name]['Kp']:g})")
        ax.plot(t, r, color=col, lw=0.8, ls=":")
    ax.set(title="Adaptive gain-scheduled PID: driver profiles", xlabel="Time (s)", ylabel="Speed (m/s)")
    ax.legend(); ax.grid(alpha=0.3)
    save(fig, "fig02_adaptive_profiles.png")

    # 3. Fixed PID vs adaptive (normal)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(*step("Fixed PID"), "--", color="tab:red", lw=2.5, label="Fixed PID")
    ax.plot(*step("Adapt-Normal"), "-", color="tab:blue", lw=1.2, label="Adaptive PID (Normal)")
    ax.set(title="Fixed PID vs adaptive PID, normal driver", xlabel="Time (s)", ylabel="Speed (m/s)")
    ax.legend(); ax.grid(alpha=0.3)
    save(fig, "fig03_fixed_vs_adaptive.png")

    # 4 / 5. Root locus
    for fname, sys, title in (("fig04_rlocus_plant.png", G, "Root locus: plant G(s)"),
                              ("fig05_rlocus_pid.png", ctrl["Fixed PID"] * G, "Root locus: C_PID(s) G(s)")):
        fig, ax = plt.subplots(figsize=(7, 5))
        gains = np.concatenate([[0.0], np.logspace(-3, 6, 3000)])
        roots = np.asarray(ct.root_locus_map(sys, gains).loci)
        for i in range(roots.shape[1]):
            ax.plot(roots[:, i].real, roots[:, i].imag, lw=1.8)
        ax.plot(np.real(ct.poles(sys)), np.imag(ct.poles(sys)), "kx", ms=9, label="open-loop poles")
        z = ct.zeros(sys)
        if len(z):
            ax.plot(np.real(z), np.imag(z), "ko", mfc="none", ms=8, label="zeros")
        ax.axhline(0, color="k", lw=0.5); ax.axvline(0, color="k", lw=0.5)
        ax.set(title=title, xlabel="Real axis (1/s)", ylabel="Imaginary axis (1/s)")
        ax.set_xlim(-35, 2)
        ax.legend(); ax.grid(alpha=0.3)
        save(fig, fname)

    # 6. Bode
    w = np.logspace(-4, 2, 2000)
    fig, (am, ap) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for name, col in (("Fixed P", "tab:blue"), ("Fixed PI", "tab:orange"), ("Fixed PID", "goldenrod")):
        resp = ct.frequency_response(ctrl[name] * G, w)
        am.semilogx(w, 20 * np.log10(np.abs(resp.complex)), color=col, lw=2, label=name.split()[1])
        ap.semilogx(w, np.degrees(np.unwrap(np.angle(resp.complex))), color=col, lw=2)
    am.axhline(0, color="k", lw=0.6); ap.axhline(-180, color="k", lw=0.6)
    am.set(ylabel="Magnitude (dB)", title="Bode: open-loop C(s) G(s)"); ap.set(ylabel="Phase (deg)", xlabel="Frequency (rad/s)")
    am.legend(); am.grid(alpha=0.3, which="both"); ap.grid(alpha=0.3, which="both")
    save(fig, "fig06_bode.png")

    # 7. Nyquist (positive and negative frequencies, integrator contour skipped)
    w = np.logspace(-2.5, 3, 4000)
    L = ct.frequency_response(ctrl["Fixed PID"] * G, w).complex
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(L.real, L.imag, color="tab:blue", lw=2, label="ω > 0")
    ax.plot(L.real, -L.imag, color="tab:blue", lw=1, ls="--", label="ω < 0")
    ax.plot(-1, 0, "r+", ms=14, mew=2, label="-1 + 0j")
    ax.set(title="Nyquist: C_PID(s) G(s)", xlabel="Real", ylabel="Imaginary", xlim=(-1.5, 0.5), ylim=(-3, 3))
    ax.legend(); ax.grid(alpha=0.3)
    save(fig, "fig07_nyquist_pid.png")

    # 8. State feedback vs PID
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(*step("Fixed PID"), "--", color="tab:red", lw=2, label="Fixed PID")
    ts, ys = ct.step_response(V_REF * state_feedback_tf(), t)
    ax.plot(ts, ys, color="m", lw=2, label="State feedback, pole at -5")
    ax.set(title="State feedback vs fixed PID", xlabel="Time (s)", ylabel="Speed (m/s)", xlim=(0, 80))
    ax.legend(); ax.grid(alpha=0.3)
    save(fig, "fig08_state_feedback.png")

    # 9. Disturbance rejection, impulse (report code) and step
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, kind in zip(axes, ("impulse", "step")):
        for k, col in (("P", "tab:red"), ("PI", "tab:blue"), ("PID", "tab:green")):
            d = dist[(k, kind)]
            ax.plot(d["t"], -d["y"] * (1000 if kind == "impulse" else 1), color=col, lw=2, label=k)
        unit = "mm/s" if kind == "impulse" else "m/s"
        ax.set(title=f"{kind.capitalize()} disturbance d = {D_AMP:g} at plant input",
               xlabel="Time (s)", ylabel=f"Speed deviation ({unit})", xlim=(0, 80))
        ax.legend(); ax.grid(alpha=0.3)
    save(fig, "fig09_disturbance.png")

    # 10. Dashboard
    fig, axs = plt.subplots(3, 3, figsize=(15, 11))
    for name, col in (("Fixed P", "tab:red"), ("Fixed PI", "tab:blue"), ("Fixed PID", "tab:green")):
        axs[0, 0].plot(*step(name), color=col, label=name.split()[1])
    for name, col in (("Smooth", "tab:blue"), ("Normal", "tab:green"), ("Aggressive", "tab:red")):
        axs[0, 1].plot(*step(f"Adapt-{name}"), color=col, label=name)
    axs[0, 2].plot(*step("Fixed PID"), "--", color="tab:red", label="Fixed PID")
    axs[0, 2].plot(*step("Adapt-Normal"), color="tab:blue", label="Adaptive (Normal)")
    rl = np.asarray(ct.root_locus_map(G, np.logspace(-3, 6, 500)).loci)
    axs[1, 0].plot(rl.real, rl.imag, lw=2); axs[1, 0].plot(-B / M, 0, "kx")
    w = np.logspace(-4, 2, 1000)
    resp = ct.frequency_response(ctrl["Fixed PID"] * G, w).complex
    axs[1, 1].semilogx(w, 20 * np.log10(np.abs(resp)))
    axs[1, 2].semilogx(w, np.degrees(np.unwrap(np.angle(resp))))
    for k, col in (("P", "tab:red"), ("PI", "tab:blue"), ("PID", "tab:green")):
        d = dist[(k, "step")]
        axs[2, 0].plot(d["t"], -d["y"], color=col, label=k)
    axs[2, 1].plot(*step("Fixed PID"), "--", color="tab:red", label="PID")
    axs[2, 1].plot(ts, ys, color="m", label="State feedback")
    names = ["Fixed P", "Fixed PI", "Fixed PID", "Adapt-Smooth", "Adapt-Aggressive"]
    x = np.arange(len(names))
    for i, (key, lab) in enumerate((("rise", "Rise (s)"), ("settling", "Settling (s)"), ("overshoot", "OS (%)"))):
        axs[2, 2].bar(x + (i - 1) * 0.27, [metrics[n][key] for n in names], 0.27, label=lab)
    axs[2, 2].set_xticks(x, [n.replace("Adapt-", "A-").replace("Fixed ", "") for n in names])
    titles = ["P vs PI vs PID", "Adaptive profiles", "Fixed vs adaptive", "Root locus G(s)",
              "Bode magnitude, PID OL (dB)", "Bode phase, PID OL (deg)", "Step disturbance",
              "PID vs state feedback", "Metrics"]
    for ax, title in zip(axs.flat, titles):
        ax.set_title(title); ax.grid(alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=8)
    fig.suptitle("Adaptive driver-behaviour-aware speed control: summary", fontsize=14)
    save(fig, "fig10_dashboard.png")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--no-plots", action="store_true")
    args = p.parse_args()
    out = run(plots=not args.no_plots)
    for name, m in out["metrics"].items():
        print(f"{name:17s} rise {m['rise']:6.2f} s  settling {m['settling']:6.2f} s  "
              f"OS {m['overshoot']:5.2f} %  final {m['final']:6.2f} m/s")
    print(f"Wrote {RESULTS.relative_to(ROOT)}/metrics.csv, margins.csv, disturbance.csv, comparison.md")


if __name__ == "__main__":
    main()
