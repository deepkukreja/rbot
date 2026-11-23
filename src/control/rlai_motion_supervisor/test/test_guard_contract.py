"""Health and source-time admission contracts; no ROS transport or hardware."""
from importlib import import_module
import math
import pytest


def healthy(now=1.,stamp=10.):
    h=import_module('rlai_motion_supervisor.health').HealthMonitor()
    h.clock(stamp-.1,now-.1)
    h.clock(stamp,now)
    for stream in ['scan','odom','map_tf','odom_tf','amcl']:
        h.observe(stream,stamp,now,valid=True)
    return h


@pytest.mark.parametrize('stream',['scan','odom','map_tf','odom_tf','amcl'])
def test_missing_or_invalid_source_inhibits(stream):
    h=healthy()
    h.observe(stream,10.1,1.1,valid=False)
    assert stream.upper()+'_INVALID' in h.reason(1.1,10.1)


@pytest.mark.parametrize('stream',['scan','odom','map_tf','odom_tf'])
def test_cached_or_replayed_source_does_not_refresh_liveness(stream):
    h=healthy()
    for name in ['scan','odom','map_tf','odom_tf','amcl']:
        if name!=stream:
            h.observe(name,10.7,1.7,valid=True)
    h.clock(10.7,1.7)
    h.observe(stream,10.,1.7,valid=True)
    assert stream.upper()+'_STALE' in h.reason(1.7,10.7)


def test_pose_covariance_requires_real_refresh_within_its_longer_budget():
    h=healthy()
    for name in ['scan','odom','map_tf','odom_tf']:
        h.observe(name,16.,7.,valid=True)
    h.clock(16.,7.)
    assert 'AMCL_STALE' in h.reason(7.,16.)


def test_paused_clock_and_rewind_inhibit_even_with_repeated_frames():
    h=healthy()
    for name in ['scan','odom','map_tf','odom_tf','amcl']:
        h.observe(name,10.7,1.7,valid=True)
    h.clock(10.,1.7)
    assert 'CLOCK_STALE' in h.reason(1.7,10.7)
    h.clock(2.,1.8)
    h.clock(11.,1.9)
    assert h.reason(1.9,11.)=='CLOCK_REWOUND'


def test_amcl_future_tf_tolerance_does_not_allow_unbounded_future_data():
    h=healthy()
    h.observe('map_tf',11.,1.1,valid=True)
    assert h.reason(1.1,10.1)==''
    h.observe('map_tf',13.,1.2,valid=True)
    assert 'MAP_TF_TIMESTAMP' in h.reason(1.2,10.2)


@pytest.mark.parametrize('values,expected',[
    ([1.,2.,3.,4.],True),([math.inf]*4,False),([math.nan]*4,False),
    ([1.,2.,math.nan,4.],False),([0.,-1.,1.,2.],False),
])
def test_scan_validity_never_treats_all_invalid_or_no_return_as_clear(values,expected):
    check=import_module('rlai_motion_supervisor.health').valid_scan
    assert check(values,.1,10.,.01)==expected


@pytest.mark.parametrize('xy,yaw,expected',[(.1,.1,True),(.3,.1,False),(.1,.3,False),(math.nan,.1,False),(-.1,.1,False)])
def test_localization_covariance_is_finite_bounded_and_nonnegative(xy,yaw,expected):
    check=import_module('rlai_motion_supervisor.health').valid_covariance
    cov=[0.]*36
    cov[0]=cov[7]=xy
    cov[35]=yaw
    assert check(cov)==expected


def test_inhibit_revokes_owner_and_restoration_does_not_resume():
    s=import_module('rlai_motion_supervisor.core').Supervisor()
    grant=s.acquire(owner='nav',source='AUTONOMY',duration=1.,now=0.)
    assert grant.ok
    s.set_inhibit('SCAN_STALE',.01)
    assert s.tick(.02).state=='INHIBITED'
    assert not s.acquire(owner='nav',source='AUTONOMY',duration=1.,now=.03).ok
    s.set_inhibit('',.04)
    assert s.tick(.05).state=='IDLE' and not s.tick(.05).owner
    assert not s.renew(grant.token,duration=1.,now=.06)


def gate():
    g=import_module('rlai_motion_supervisor.nav_gate').VelocityGate()
    g.begin(activation_stamp=10.)
    return g


@pytest.mark.parametrize('stamp,ros_now',[(9.9,10.01),(10.1,10.5),(11.,10.1)])
def test_nav_velocity_before_goal_or_expired_or_future_is_rejected(stamp,ros_now):
    g=gate()
    assert g.admit(stamp,ros_now,.01,(.1,0.,0.,0.,0.,0.)) is None


def test_nav_packets_are_consumed_once_and_retain_transit_age():
    g=gate()
    sample=g.admit(10.1,10.2,1.,(.1,0.,0.,0.,0.,0.))
    assert sample is not None and sample['age']==pytest.approx(.1)
    assert g.admit(10.1,10.21,1.01,(.4,0.,0.,0.,0.,0.)) is None
    g.end()
    assert g.admit(10.22,10.23,1.02,(.4,0.,0.,0.,0.,0.)) is None


def test_bad_owned_nav_geometry_fails_admission():
    g=gate()
    with pytest.raises(ValueError):
        g.admit(10.1,10.11,.1,(math.nan,0.,0.,0.,0.,0.))


@pytest.mark.parametrize('state',[0,1,2,4])
def test_inactive_nav2_component_prevents_admission(state):
    gate=import_module('rlai_motion_supervisor.nav_gate').LifecycleGate()
    for name in gate.required:
        gate.observe(name,3,1.)
    gate.observe('controller_server',state,1.1)
    assert 'CONTROLLER_SERVER' in gate.reason(1.1)


def test_active_lifecycle_evidence_ages_instead_of_remaining_ready():
    gate=import_module('rlai_motion_supervisor.nav_gate').LifecycleGate()
    for name in gate.required:
        gate.observe(name,3,1.)
    assert gate.reason(1.1)==''
    assert 'STALE' in gate.reason(3.)
