"""Checks of the run-comparison logic on synthetic numbers (test fixtures only, never reported as results)."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from evaluation.compare_runs import fmt, num, verdict  # noqa: E402


def test_verdict_direction():
    assert verdict(30.0, 15.0, "lower") == "Better"
    assert verdict(30.0, 45.0, "lower") == "Worse"
    assert verdict(0.088, 0.30, "higher") == "Better"
    assert verdict(80.0, 60.0, "higher") == "Worse"


def test_verdict_tolerance_and_missing():
    assert verdict(100.0, 102.0, "lower") == "Same"        # within 3 %
    assert verdict(100.0, 105.0, "lower") == "Worse"
    assert verdict(None, 5.0, "lower") == "-"              # missing value: no verdict
    assert verdict(5.0, None, "higher") == "-"
    assert verdict(5.0, 9.0, None) == "-"                  # information only
    assert verdict(0.0, 0.0, "lower") == "Same"


def test_booleans_and_unavailable_values():
    assert verdict(False, True, "higher") == "Better"      # task completed
    assert num({"value": None, "reason": "x"}) is None     # unavailable stays unavailable, never zero
    assert num(float("nan")) is None
    assert num(True) == 1.0
    assert fmt(None, 2) == "n/a" and fmt(1.2345, 2) == "1.23"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(name, "ok")
