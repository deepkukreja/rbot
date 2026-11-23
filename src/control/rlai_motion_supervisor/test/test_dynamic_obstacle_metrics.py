"""Catch optimistic clearance when a padded robot approaches an unmapped box."""
import math
import pytest
from rlai_motion_supervisor import scenario_metrics as metrics


def test_front_clearance_subtracts_box_edge_and_enclosing_robot_radius():
    assert metrics.conservative_box_clearance({'x':0.,'y':0.},center=(2.,0.),size=(1.,1.),half_length=.3,half_width=.4)==pytest.approx(1.)


def test_corner_distance_is_euclidean_rather_than_axis_separation():
    assert metrics.conservative_box_clearance({'x':0.,'y':0.},center=(1.5,1.5),size=(1.,1.),half_length=.3,half_width=.4)==pytest.approx(math.sqrt(2)-.5)


def test_box_containing_robot_center_has_negative_clearance():
    assert metrics.conservative_box_clearance({'x':2.,'y':0.},center=(2.,0.),size=(1.,1.),half_length=.3,half_width=.4)==pytest.approx(-1.)


def test_enclosing_circle_touching_box_does_not_count_as_clearance():
    assert metrics.conservative_box_clearance({'x':1.,'y':0.},center=(2.,0.),size=(1.,1.),half_length=.3,half_width=.4)==pytest.approx(0.)


@pytest.mark.parametrize('pose',[None,{}, {'x':math.nan,'y':0.}, {'x':0.,'y':math.inf}])
def test_missing_or_nonfinite_position_cannot_prove_clearance(pose):
    assert metrics.conservative_box_clearance(pose,center=(2.,0.),size=(1.,1.),half_length=.3,half_width=.4) is None


@pytest.mark.parametrize('size',[(0.,1.),(-1.,1.),(math.inf,1.)])
def test_invalid_obstacle_dimensions_are_rejected(size):
    with pytest.raises(ValueError):
        metrics.conservative_box_clearance({'x':0.,'y':0.},center=(2.,0.),size=size,half_length=.3,half_width=.4)
