"""The simulation-clock gate requires fresh forward progress before activation."""
import importlib.util
from pathlib import Path


GATE = Path(__file__).resolve().parents[1] / "src/control/rlai_control/launch/wait_for_sim_clock.py"


def clock_progress():
    assert GATE.is_file(), "Guarded control has no clock progress gate"
    spec = importlib.util.spec_from_file_location("wait_for_sim_clock", GATE)
    gate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gate)
    return gate.ClockProgress()


def test_clock_requires_fresh_progress_beyond_single_or_repeated_sample():
    progress = clock_progress()
    for value in (0, 1_000_000, 1_000_000, 50_999_999):
        assert not progress.observe(value)
    assert progress.observe(51_000_000)


def test_clock_rewind_discards_previous_progress():
    progress = clock_progress()
    for value in (900_000_000, 940_000_000, 100_000_000, 140_000_000):
        assert not progress.observe(value)
    assert progress.observe(150_000_000)
