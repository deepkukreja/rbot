import importlib
import math
import pytest


def module():
    return importlib.import_module('rlai_motion_supervisor.navigation_metrics')


def test_ground_truth_pairing_interpolates_position_and_wraps_yaw():
    truth=[{'stamp':1.,'x':0.,'y':2.,'yaw':math.radians(170)},
        {'stamp':1.1,'x':2.,'y':4.,'yaw':math.radians(-170)}]
    actual=module().interpolate_truth(truth,1.05,max_gap=.2)
    assert actual['x']==pytest.approx(1.) and actual['y']==pytest.approx(3.)
    assert abs(actual['yaw'])==pytest.approx(math.pi)


def test_missing_or_distant_truth_is_unknown_and_never_extrapolated():
    truth=[{'stamp':1.,'x':0.,'y':0.,'yaw':0.},{'stamp':2.,'x':1.,'y':0.,'yaw':0.}]
    for stamp in [.9,1.5,2.1]:
        assert module().interpolate_truth(truth,stamp,max_gap=.1) is None
    assert module().interpolate_truth([],1.) is None


def test_exact_truth_sample_does_not_require_a_second_observation():
    actual=module().interpolate_truth([{'stamp':1.,'x':2.,'y':3.,'yaw':.2}],1.)
    assert actual['x']==2. and actual['y']==3.


def test_action_success_does_not_override_bad_ground_truth_goal_position():
    result=module().score_goal(4,(1.,0.,0.),{'x':0.,'y':0.,'yaw':0.})
    assert not result['passed'] and result['reason']=='TRUTH_GOAL_TOLERANCE'
    assert result['xy_error_m']==1.


def test_aborted_action_near_goal_is_still_a_failed_goal():
    result=module().score_goal(6,(1.,0.,0.),{'x':1.,'y':0.,'yaw':0.})
    assert not result['passed'] and result['reason']=='ACTION_NOT_SUCCEEDED'


def test_yaw_completion_is_compared_across_angle_wrap():
    result=module().score_goal(4,(0.,0.,math.radians(179)),{'x':.1,'y':0.,'yaw':math.radians(-179)})
    assert result['passed']
    assert result['yaw_error_rad']==pytest.approx(math.radians(2))


def test_missing_or_invalid_ground_truth_cannot_pass_a_successful_action():
    for truth in [None,{'x':float('nan'),'y':0.,'yaw':0.}]:
        result=module().score_goal(4,(0.,0.,0.),truth)
        assert not result['passed'] and result['reason']=='TRUTH_UNAVAILABLE'


def test_goal_heading_failure_is_scored_even_when_position_is_close():
    result=module().score_goal(4,(0.,0.,0.),{'x':0.,'y':0.,'yaw':.5})
    assert not result['passed'] and result['reason']=='TRUTH_GOAL_TOLERANCE'


@pytest.mark.parametrize('goal',[(float('nan'),0.,0.),(0.,float('inf'),0.)])
def test_invalid_benchmark_goal_is_rejected(goal):
    with pytest.raises(ValueError):
        module().score_goal(4,goal,{'x':0.,'y':0.,'yaw':0.})
