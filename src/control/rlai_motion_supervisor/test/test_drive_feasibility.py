"""Known kinematic and malformed-input regressions for the read-only audit."""
import copy
import importlib
import importlib.util
import json
import math
import sys
import pytest

CONFIG={'diff_drive_controller':{'ros__parameters':{
    'left_wheel_names':['left'],'right_wheel_names':['right'],
    'wheel_separation':.35,'wheel_separation_multiplier':.886,'wheel_radius':.0625,
    'linear':{'x':{'min_velocity':-.35,'max_velocity':.5}},
    'angular':{'z':{'min_velocity':-1.9,'max_velocity':1.9}},
}}}
URDF='''<robot name="test">
<joint name="left" type="continuous"><limit velocity="5"/></joint>
<joint name="right" type="continuous"><limit velocity="5"/></joint>
<ros2_control name="test" type="system"><hardware><plugin>test/Simulation</plugin></hardware>
<joint name="left"><command_interface name="velocity"><param name="min">-5</param><param name="max">5</param></command_interface></joint>
<joint name="right"><command_interface name="velocity"><param name="min">-5</param><param name="max">5</param></command_interface></joint>
</ros2_control></robot>'''

def module():
    name='rlai_motion_supervisor.drive_feasibility'
    assert importlib.util.find_spec(name) is not None, 'Missing read-only coupled-wheel feasibility audit'
    return importlib.import_module(name)

def audit(linear=None,angular=None,config=None,urdf=URDF):
    return module().audit_configuration(config or copy.deepcopy(CONFIG),urdf,linear=linear,angular=angular)

def test_mixed_command_passes_axis_box_but_exceeds_right_wheel():
    report=audit(.3,1.)
    assert report['requested']['feasible'] is False
    assert report['requested']['wheels']['left']['demand_rad_s']==pytest.approx(2.3192)
    assert report['requested']['wheels']['right']['demand_rad_s']==pytest.approx(7.2808)
    assert report['requested']['wheels']['right']['within_limit'] is False
    assert report['effective_separation_m']==pytest.approx(.3101)

@pytest.mark.parametrize('linear,angular,expected',[(0,0,True),(.3,0,True),(.3125,0,True),(-.3125,0,True),
    (.312500001,0,False),(-.312500001,0,False),(0,1.9,True),(0,-1.9,True),(.5,0,False),(-.35,0,False)])
def test_axis_and_boundary_demands(linear,angular,expected):
    assert audit(linear,angular)['requested']['feasible'] is expected

def test_box_corner_audit_detects_every_infeasible_corner():
    report=audit()
    assert report['configuration_box_feasible'] is False
    assert len(report['box_corners'])==4
    assert all(not p['feasible'] for p in report['box_corners'])

def test_wheel_feasibility_does_not_override_stricter_chassis_admission():
    point=audit(0,2.)['requested']
    assert point['feasible'] is False
    assert point['wheel_limits_feasible'] is True
    assert point['within_chassis_limits'] is False
    assert point['common_scale_to_limits']<.95

def test_common_scale_is_strictly_feasible_and_preserves_requested_curve():
    requested=audit(.3,1.)['requested']
    scale=requested['common_scale_to_limits']
    assert 0<scale<1
    scaled=audit(.3*scale,scale)['requested']
    assert scaled['feasible'] is True
    assert scaled['wheels']['left']['demand_rad_s']/scaled['wheels']['right']['demand_rad_s']==pytest.approx(2.3192/7.2808)
    assert audit(0,0)['requested']['common_scale_to_limits']==1.

def test_asymmetric_interface_and_urdf_intersection_are_respected():
    xml=URDF.replace('<param name="min">-5</param>','<param name="min">-2</param>',1)
    assert audit(-.2,0,urdf=xml)['requested']['feasible'] is False
    assert audit(.2,0,urdf=xml)['requested']['feasible'] is True
    xml=URDF.replace('limit velocity="5"','limit velocity="2"',1)
    assert audit(.2,0,urdf=xml)['requested']['feasible'] is False

def test_radius_multiplier_changes_controller_joint_demand():
    config=copy.deepcopy(CONFIG)
    config['diff_drive_controller']['ros__parameters']['left_wheel_radius_multiplier']=.5
    report=audit(.2,0,config=config)
    assert report['requested']['wheels']['left']['demand_rad_s']==pytest.approx(6.4)
    assert report['requested']['feasible'] is False

@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_nonfinite_requests_are_errors(value):
    with pytest.raises(ValueError):audit(value,0)
    with pytest.raises(ValueError):audit(0,value)

@pytest.mark.parametrize('field,value',[('wheel_radius',0),('wheel_radius',-1),('wheel_separation',0),
    ('wheel_separation',math.nan),('wheel_separation_multiplier',math.inf),('left_wheel_radius_multiplier',-1)])
def test_unknown_or_invalid_geometry_never_reports_feasible(field,value):
    config=copy.deepcopy(CONFIG)
    config['diff_drive_controller']['ros__parameters'][field]=value
    with pytest.raises(ValueError):audit(config=config)

def test_missing_wheel_metadata_is_an_error_not_unlimited_speed():
    with pytest.raises(ValueError):audit(0,0,urdf=URDF.replace('<param name="max">5</param>','',1))
    with pytest.raises(ValueError):audit(0,0,urdf=URDF.replace('<limit velocity="5"/>','',1))

def test_audit_does_not_mutate_loaded_configuration():
    config=copy.deepcopy(CONFIG)
    before=copy.deepcopy(config)
    audit(.3,1,config=config)
    assert config==before

def test_explicit_file_cli_needs_no_ros_runtime_and_has_meaningful_exit(tmp_path,capsys,monkeypatch):
    import yaml
    config=tmp_path/'controllers.yaml';config.write_text(yaml.safe_dump(CONFIG))
    urdf=tmp_path/'robot.urdf';urdf.write_text(URDF)
    monkeypatch.setitem(sys.modules,'rclpy',None)
    args=['--controllers',str(config),'--urdf',str(urdf)]
    assert module().main(args)==2
    assert json.loads(capsys.readouterr().out)['configuration_box_feasible'] is False
    assert module().main(args+['--linear','.3','--angular','0'])==0
    assert json.loads(capsys.readouterr().out)['requested']['feasible'] is True
    assert module().main(args+['--linear','nan','--angular','0'])==2
    assert 'error' in json.loads(capsys.readouterr().out)
