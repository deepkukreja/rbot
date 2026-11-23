import importlib
import math
import pytest
from builtin_interfaces.msg import Time
from rlai_motion_interfaces.msg import SupervisorStatus,NavigationStatus


def module():
    return importlib.import_module('rlai_motion_supervisor.localization_reset')


def inputs():
    supervisor=SupervisorStatus()
    supervisor.state='STOPPED'
    supervisor.health_required=True
    supervisor.inhibited=True
    supervisor.health_reason='AMCL_INVALID'
    navigation=NavigationStatus()
    navigation.phase='IDLE'
    navigation.reason='HEALTH_UNAVAILABLE'
    return (supervisor,99.9),(navigation,99.9),[(99.6,0.,0.),(99.7,0.,0.),(99.8,0.,0.),(99.98,0.,0.)]


def test_explicit_pose_encodes_nonzero_uncertainty_in_map_coordinates():
    pose=module().initial_pose_message(x=1.,y=2.,yaw=math.pi/2,xy_std=.2,yaw_std=.1,stamp=Time(sec=12))
    assert pose.header.frame_id=='map' and pose.header.stamp.sec==12
    assert pose.pose.pose.position.x==1. and pose.pose.pose.position.y==2.
    assert pose.pose.pose.orientation.z==pytest.approx(math.sqrt(.5))
    assert pose.pose.covariance[0]==pytest.approx(.04)
    assert pose.pose.covariance[7]==pytest.approx(.04)
    assert pose.pose.covariance[35]==pytest.approx(.01)
    assert all(v==0 for i,v in enumerate(pose.pose.covariance) if i not in {0,7,35})


@pytest.mark.parametrize('field,value',[
    ('x',float('nan')),('y',float('inf')),('yaw',float('nan')),
    ('xy_std',0.),('xy_std',-.1),('xy_std',1e200),('xy_std',1e-200),('yaw_std',1e-200),('yaw_std',0.),('yaw_std',float('inf')),
])
def test_invalid_or_zero_uncertainty_is_rejected_before_ros_request(field,value):
    values=dict(x=0.,y=0.,yaw=0.,xy_std=.1,yaw_std=.1,stamp=Time(sec=1))
    values[field]=value
    with pytest.raises(ValueError):
        module().initial_pose_message(**values)


def test_stopped_unowned_stationary_invalid_localization_allows_explicit_reset():
    assert module().reset_blocker(*inputs(),now=100.)==''


@pytest.mark.parametrize('component,field,value,reason',[
    (0,'state','IDLE','STOP_NOT_LATCHED'),(0,'owner','another_owner','OWNER_PRESENT'),
    (0,'health_required',False,'HEALTH_ADMISSION_DISABLED'),
    (1,'quarantined',True,'NAVIGATION_NOT_SETTLED'),(1,'phase','STOPPING','NAVIGATION_NOT_SETTLED'),
    (1,'goal_id','1234567890abcdef1234567890abcdef','NAVIGATION_NOT_SETTLED'),
    (1,'forwarding',True,'NAVIGATION_NOT_SETTLED'),
])
def test_control_state_changes_prevent_reset(component,field,value,reason):
    values=inputs()
    setattr(values[component][0],field,value)
    assert module().reset_blocker(*values,now=100.)==reason


def test_stale_status_and_future_receipts_do_not_authorize_reset():
    supervisor,navigation,speeds=inputs()
    assert module().reset_blocker((supervisor[0],99.),navigation,speeds,now=100.)=='STATUS_UNAVAILABLE'
    assert module().reset_blocker(supervisor,(navigation[0],101.),speeds,now=100.)=='STATUS_UNAVAILABLE'


def test_stationary_evidence_requires_multiple_finite_fresh_samples_over_time():
    supervisor,navigation,speeds=inputs()
    for samples in [speeds[:1],[(t+1.,v,w) for t,v,w in speeds],
        [(98.+i/10,0.,0.) for i in range(4)],[(99.7+i/20,0.,0.) for i in range(4)],
        [(t,.02,w) for t,v,w in speeds],[(t,v,.04) for t,v,w in speeds],[(t,float('nan'),w) for t,v,w in speeds]]:
        assert module().reset_blocker(supervisor,navigation,samples,now=100.)=='STATIONARITY_UNCONFIRMED'


def test_reset_entry_rejects_an_unvalidated_zero_covariance_pose_before_ros_io():
    from geometry_msgs.msg import PoseWithCovarianceStamped
    pose=PoseWithCovarianceStamped()
    pose.header.frame_id='map'
    pose.pose.pose.orientation.w=1.
    with pytest.raises(ValueError):
        module().reset_localization(None,pose)


def test_other_sensor_faults_do_not_authorize_a_localization_reset():
    supervisor,navigation,speeds=inputs()
    supervisor[0].health_reason='SCAN_STALE'
    assert module().reset_blocker(supervisor,navigation,speeds,now=100.)=='OTHER_SOURCE_UNHEALTHY'


def test_fresh_estimate_must_agree_with_supplied_pose_prior():
    values=dict(x=0.,y=0.,yaw=0.,xy_std=.1,yaw_std=.1,stamp=Time(sec=1))
    prior=module().initial_pose_message(**values)
    nearby=module().initial_pose_message(**{**values,'x':.02})
    distant=module().initial_pose_message(**{**values,'x':1000.})
    turned=module().initial_pose_message(**{**values,'yaw':math.pi})
    assert module().pose_consistent_with_prior(nearby,prior)
    assert not module().pose_consistent_with_prior(distant,prior)
    assert not module().pose_consistent_with_prior(turned,prior)
