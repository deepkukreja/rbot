import copy
import math
import sys
from types import SimpleNamespace

import pytest
import yaml

from profile_overlay import configure, prepare


def inputs():
    axis = {'has_velocity_limits': True, 'has_acceleration_limits': True,
            'has_jerk_limits': False}
    driver = {
        'use_sim_time': True,
        'left_wheel_names': ['left'], 'right_wheel_names': ['right'],
        'wheel_separation': .35, 'wheel_separation_multiplier': .886, 'wheel_radius': .0625,
        'linear': {'x': dict(axis, min_velocity=-.35, max_velocity=.5,
                            min_acceleration=-1., max_acceleration=1.)},
        'angular': {'z': dict(axis, min_velocity=-1.9, max_velocity=1.9,
                             min_acceleration=-3.2, max_acceleration=3.2)},
    }
    controller = {'diff_drive_controller': {'ros__parameters': driver},
                  'controller_manager': {'ros__parameters': {'update_rate': 100}}}
    nav = {'controller_server': {'ros__parameters': {'FollowPath': {
        'plugin': 'nav2_mppi_controller::MPPIController', 'motion_model': 'DiffDrive',
        'vx_min': -.35, 'vx_max': .5, 'wz_max': 1.9, 'vy_max': 0.,
    }}}, 'amcl': {'example': True}}
    wheel = """
      <joint name="{name}" type="continuous"><limit velocity="5"/></joint>
    """
    controlled = """
      <joint name="{name}"><command_interface name="velocity">
        <param name="min">-5</param><param name="max">5</param>
      </command_interface></joint>
    """
    robot = '<robot name="test">' + ''.join(wheel.format(name=n) for n in ('left', 'right'))
    robot += '<ros2_control name="Gazebo" type="system"><hardware>'
    robot += '<plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>'
    robot += ''.join(controlled.format(name=n) for n in ('left', 'right')) + '</ros2_control></robot>'
    return controller, nav, robot


def test_paired_profiles_share_cap_geometry_and_rates_without_mutating_base():
    control, nav, robot = inputs()
    original = copy.deepcopy((control, nav))
    candidate_control, candidate_nav = configure(control, nav, robot)
    c = candidate_control['diff_drive_controller']['ros__parameters']
    p = candidate_nav['controller_server']['ros__parameters']['FollowPath']
    assert c['coupled_wheel_velocity_limit'] == 5.0
    assert p['DiffDriveConstraints'] == {'max_wheel_velocity': 5., 'wheel_separation': .3101,
                                         'left_wheel_radius': .0625, 'right_wheel_radius': .0625}
    assert (p['ax_min'], p['ax_max'], p['az_max']) == (-1., 1., 3.2)
    assert (control, nav) == original
    assert candidate_nav['amcl'] == nav['amcl']
    assert candidate_control['controller_manager'] == control['controller_manager']


@pytest.mark.parametrize('old,new', [
    ('gz_ros2_control/GazeboSimSystem', 'hardware/Robot'),
    ('velocity="5"', 'velocity="4"'),
    ('name="min">-5', 'name="min">-3'),
    ('name="max">5', 'name="other">5'),
])
def test_rejects_unsupported_actuator_contract(old, new):
    control, nav, robot = inputs()
    with pytest.raises(ValueError):
        configure(control, nav, robot.replace(old, new, 1))


@pytest.mark.parametrize('field,value', [
    ('wheel_radius', math.nan), ('wheel_separation_multiplier', 0),
    ('right_wheel_names', ['left']), ('use_sim_time', False),
])
def test_rejects_invalid_controller_setup(field, value):
    control, nav, robot = inputs()
    control['diff_drive_controller']['ros__parameters'][field] = value
    with pytest.raises(ValueError):
        configure(control, nav, robot)


def test_rejects_jerk_or_model_mismatch():
    control, nav, robot = inputs()
    control['diff_drive_controller']['ros__parameters']['linear']['x']['has_jerk_limits'] = True
    with pytest.raises(ValueError):
        configure(control, nav, robot)


def test_generated_profile_resolves_navigation_executables(tmp_path, monkeypatch):
    control, nav, robot = inputs()
    base = tmp_path / 'base'
    packages = ['rlai_control', 'rlai_navigation', 'rlai_description', 'rlai_motion_supervisor']
    for name in packages:
        (base / 'share' / name).mkdir(parents=True)
    for package, filename, config in [('rlai_control', 'controllers.yaml', control),
                                       ('rlai_navigation', 'nav2_params.yaml', nav)]:
        target = base / 'share' / package / 'config'
        target.mkdir()
        (target / filename).write_text(yaml.safe_dump(config))
    launch = base / 'share/rlai_navigation/launch'
    launch.mkdir()
    (launch / 'supervised_navigation.launch.py').touch()
    (base / 'share/rlai_motion_supervisor/package.xml').touch()
    nav_libexec = base / 'lib/rlai_navigation'
    nav_libexec.mkdir(parents=True)
    executable = nav_libexec / 'scan_occlusion'
    executable.touch()
    executable.chmod(0o755)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', SimpleNamespace(
        get_package_share_directory=lambda p: str(base / 'share' / p),
        get_package_prefix=lambda _: str(base),
    ))
    monkeypatch.setitem(sys.modules, 'xacro', SimpleNamespace(
        process_file=lambda *args, **kwargs: SimpleNamespace(toxml=lambda: robot),
    ))
    output = tmp_path / 'profile'
    prepare(output)
    assert (output / 'lib/rlai_navigation/scan_occlusion').resolve(strict=True) == executable
    control, nav, robot = inputs()
    nav['controller_server']['ros__parameters']['FollowPath']['motion_model'] = 'Omni'
    with pytest.raises(ValueError):
        configure(control, nav, robot)
