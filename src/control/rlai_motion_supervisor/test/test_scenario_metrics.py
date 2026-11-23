import math
import pytest
from rlai_motion_supervisor.scenario_metrics import footprint_clearance, settled_latency


def test_northbound_padded_footprint_in_warehouse_passage():
    assert footprint_clearance({'x':6.7,'y':-2.,'yaw':math.pi/2},axis='x',lower=6.1,upper=7.3,half_length=.31,half_width=.26)==pytest.approx(.34)


def test_eastbound_passage_has_same_side_margin():
    assert footprint_clearance({'x':0.,'y':-8.7,'yaw':0.},axis='y',lower=-9.3,upper=-8.1,half_length=.31,half_width=.26)==pytest.approx(.34)


def test_rotated_corners_reduce_clearance():
    result=footprint_clearance({'x':6.7,'y':0.,'yaw':math.pi/4},axis='x',lower=6.1,upper=7.3,half_length=.31,half_width=.26)
    assert result==pytest.approx(.6-(.31+.26)/math.sqrt(2))


def test_overlapping_footprint_has_negative_clearance():
    assert footprint_clearance({'x':6.3,'y':0.,'yaw':math.pi/2},axis='x',lower=6.1,upper=7.3,half_length=.31,half_width=.26)<0


@pytest.mark.parametrize('pose',[None,{}, {'x':6.7,'y':0.,'yaw':math.nan}])
def test_unknown_pose_cannot_establish_clearance(pose):
    assert footprint_clearance(pose,axis='x',lower=6.1,upper=7.3,half_length=.31,half_width=.26) is None


@pytest.mark.parametrize('bounds',[{'axis':'z','lower':0.,'upper':1.},{'axis':'x','lower':1.,'upper':0.}])
def test_invalid_geometry_is_rejected(bounds):
    with pytest.raises(ValueError):footprint_clearance({'x':0.,'y':0.,'yaw':0.},half_length=.31,half_width=.26,**bounds)


def stream(stop_at=.7):
    return [{'at':i/10,'x':.3 if i/10<stop_at else 0.,'z':0.} for i in range(21)]


def test_latency_uses_start_of_sustained_zero_output():
    assert settled_latency(stream(),onset=0.,until=2.)==pytest.approx(.7)


def test_motion_resurgence_does_not_count_the_earlier_zero():
    samples=stream();samples[12]['x']=.1
    assert settled_latency(samples,onset=0.,until=2.)==pytest.approx(1.3)


@pytest.mark.parametrize('samples',[[],stream()[:-5],stream()[4:],stream()[:8]+stream()[12:]])
def test_missing_or_gapped_outputs_cannot_prove_a_stop(samples):
    assert settled_latency(samples,onset=0.,until=2.) is None


def test_nonfinite_output_cannot_prove_a_stop():
    samples=stream();samples[10]['x']=math.nan
    assert settled_latency(samples,onset=0.,until=2.) is None


def test_wheel_settle_uses_explicit_small_velocity_tolerance():
    samples=stream()
    for p in samples:
        if p['x']==0.:p['x']=.003;p['z']=.01
    assert settled_latency(samples,onset=0.,until=2.,linear_tolerance=.005,angular_tolerance=.02)==pytest.approx(.7)


def test_never_stopped_output_is_unknown():
    assert settled_latency(stream(3.),onset=0.,until=2.) is None
