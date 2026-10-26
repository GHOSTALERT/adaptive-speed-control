"""Check the re-run against the numbers printed in the submitted report."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import speed_control as sc  # noqa: E402


@pytest.fixture(scope="module")
def out():
    return sc.run(plots=False)


@pytest.mark.parametrize("name", list(sc.REPORT_METRICS))
def test_step_metrics_match_report(out, name):
    rise, settling, overshoot, _ = sc.REPORT_METRICS[name]
    m = out["metrics"][name]
    assert m["rise"] == pytest.approx(rise, abs=0.02)
    assert m["settling"] == pytest.approx(settling, abs=0.02)
    assert m["overshoot"] == pytest.approx(overshoot, abs=0.02)


def test_p_controller_steady_state_error(out):
    assert out["metrics"]["Fixed P"]["final"] == pytest.approx(20.0, abs=1e-3)
    for name in ("Fixed PI", "Fixed PID", "Adapt-Smooth", "Adapt-Aggressive"):
        assert out["metrics"][name]["final"] == pytest.approx(30.0, abs=1e-3)


@pytest.mark.parametrize("name", list(sc.REPORT_MARGINS))
def test_margins_match_report(out, name):
    pm, wgc = sc.REPORT_MARGINS[name]
    m = out["margins"][name]
    assert np.isinf(m["gm_db"])
    assert m["pm_deg"] == pytest.approx(pm, abs=0.01)
    assert m["w_gc"] == pytest.approx(wgc, abs=1e-4)


def test_state_feedback_gains(out):
    sf = out["state_feedback"]
    assert sf["K"] == pytest.approx(5950.0, abs=0.5)
    assert sf["Nbar"] == pytest.approx(6000.0, abs=0.5)
    assert sf["A_cl"] == pytest.approx(-5.0)
    assert out["metrics"]["State feedback"]["overshoot"] == pytest.approx(0.0, abs=1e-6)


def test_disturbance_hierarchy(out):
    d = out["disturbance"]
    # Sustained disturbance: P keeps an offset, PI and PID integrate it away
    assert d[("P", "step")]["final"] == pytest.approx(sc.D_AMP / (sc.KP_P + sc.B), rel=1e-3)
    assert abs(d[("PI", "step")]["final"]) < 1e-4
    assert abs(d[("PID", "step")]["final"]) < 1e-4
    assert d[("PID", "step")]["peak"] < d[("PI", "step")]["peak"] < d[("P", "step")]["peak"]


def test_closed_loops_stable():
    for c in sc.controllers().values():
        assert np.all(np.real(sc.ct.poles(sc.closed_loop(c))) < 0)
