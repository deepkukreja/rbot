#!/usr/bin/env python3
"""Generate paired options for the supervised rbot Gazebo workspace."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import yaml


def positive(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label} must be finite and positive')
    return float(value)


def single(elements, label):
    if len(elements) != 1:
        raise ValueError(f'Expected exactly one {label}')
    return elements[0]


def wheel_limit(root, system, name):
    joint = single([p for p in root.findall('joint') if p.get('name') == name], f'URDF joint {name}')
    velocity = positive(float(single(joint.findall('limit'), f'joint limit for {name}').attrib['velocity']),
                        f'{name} URDF velocity')
    configured = single([p for p in system.findall('joint') if p.get('name') == name], f'control joint {name}')
    interface = single([p for p in configured.findall('command_interface') if p.get('name') == 'velocity'],
                       f'velocity command interface for {name}')
    low = single([p for p in interface.findall('param') if p.get('name') == 'min'], f'{name} minimum')
    high = single([p for p in interface.findall('param') if p.get('name') == 'max'], f'{name} maximum')
    if float(low.text) != -velocity or float(high.text) != velocity:
        raise ValueError(f'{name} requires equal symmetric URDF and command limits')
    return velocity


def axis_limits(axis, label):
    if (axis.get('has_velocity_limits') is not True or axis.get('has_acceleration_limits') is not True
            or axis.get('has_jerk_limits') is not False):
        raise ValueError(f'{label} requires speed and acceleration limits with jerk disabled')
    if any(key in axis for key in ('max_deceleration', 'max_acceleration_reverse', 'max_deceleration_reverse')):
        raise ValueError(f'{label} directional rates fall outside the supported profile')
    rate = positive(axis['max_acceleration'], f'{label} acceleration')
    if axis['min_acceleration'] != -rate:
        raise ValueError(f'{label} requires symmetric rate limits')
    return rate


def configure(controller, nav, robot):
    controller, nav = deepcopy((controller, nav))
    try:
        driver = controller['diff_drive_controller']['ros__parameters']
        follow = nav['controller_server']['ros__parameters']['FollowPath']
        if driver['use_sim_time'] is not True:
            raise ValueError('The coupled profile requires simulation time')
        if follow['plugin'] != 'nav2_mppi_controller::MPPIController' or follow['motion_model'] != 'DiffDrive':
            raise ValueError('The planner must use the Jazzy MPPI DiffDrive motion model')
        left = single(driver['left_wheel_names'], 'left drive wheel')
        right = single(driver['right_wheel_names'], 'right drive wheel')
        if not isinstance(left, str) or not isinstance(right, str) or left == right:
            raise ValueError('Wheel names must be distinct strings')
        root = ET.fromstring(robot)
        system = single(root.findall('ros2_control'), 'ros2_control system')
        if system.findtext('hardware/plugin') != 'gz_ros2_control/GazeboSimSystem':
            raise ValueError('Only the Gazebo simulation actuator is supported')
        cap = wheel_limit(root, system, left)
        if wheel_limit(root, system, right) != cap:
            raise ValueError('Both wheels must share one symmetric speed limit')
        separation = positive(driver['wheel_separation'], 'wheel separation')
        separation *= positive(driver.get('wheel_separation_multiplier', 1.), 'separation multiplier')
        radius = positive(driver['wheel_radius'], 'wheel radius')
        left_radius = radius * positive(driver.get('left_wheel_radius_multiplier', 1.), 'left radius multiplier')
        right_radius = radius * positive(driver.get('right_wheel_radius_multiplier', 1.), 'right radius multiplier')
        for value in (separation, left_radius, right_radius):
            positive(value, 'effective wheel geometry')
        linear, angular = driver['linear']['x'], driver['angular']['z']
        ax, az = axis_limits(linear, 'linear'), axis_limits(angular, 'angular')
        for key, expected in (('vx_min', linear['min_velocity']), ('vx_max', linear['max_velocity']),
                              ('wz_max', angular['max_velocity'])):
            if not math.isfinite(expected) or follow[key] != expected:
                raise ValueError(f'Planner {key} must match the finite driver axis limit')
        if angular['min_velocity'] != -angular['max_velocity'] or follow['vy_max'] != 0.:
            raise ValueError('Angular axes must be symmetric and lateral velocity must be zero')
    except (KeyError, TypeError, ET.ParseError, AttributeError) as error:
        raise ValueError(f'Incomplete driver, planner, or robot configuration: {error}') from error
    driver['coupled_wheel_velocity_limit'] = cap
    driver['publish_limited_velocity'] = True
    follow.update(ax_min=-ax, ax_max=ax, az_max=az, DiffDriveConstraints={
        'max_wheel_velocity': cap, 'wheel_separation': separation,
        'left_wheel_radius': left_radius, 'right_wheel_radius': right_radius,
    })
    return controller, nav


def prepare(destination):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import xacro

    def share(package):
        return Path(get_package_share_directory(package))

    if destination.exists():
        raise ValueError(f'Choose a new profile directory: {destination}')
    if (not (share('rlai_navigation') / 'launch/supervised_navigation.launch.py').is_file()
            or not (share('rlai_motion_supervisor') / 'package.xml').is_file()):
        raise ValueError('Source the guarded rbot workspace before preparing the profile')
    control, nav = share('rlai_control'), share('rlai_navigation')
    nav_libexec = Path(get_package_prefix('rlai_navigation')) / 'lib/rlai_navigation'
    if not (nav_libexec / 'scan_occlusion').is_file():
        raise ValueError('The guarded navigation executables are missing')
    robot = xacro.process_file(str(share('rlai_description') / 'urdf/robot.urdf.xacro'),
                               mappings={'sim_mode': 'gazebo'}).toxml()
    filenames = ('controllers.yaml', 'nav2_params.yaml')
    configuration = [yaml.safe_load((path / 'config' / name).read_text())
                     for path, name in zip((control, nav), filenames)]
    paired = configure(*configuration, robot)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.coupled-profile-', dir=destination.parent) as staging:
        prefix = Path(staging) / 'prefix'
        index = prefix / 'share/ament_index/resource_index/packages'
        index.mkdir(parents=True)
        for path, name, config in zip((control, nav), filenames, paired):
            (index / path.name).touch()
            target = prefix / 'share' / path.name
            shutil.copytree(path, target)
            (target / 'config' / name).write_text(yaml.safe_dump(config, sort_keys=False))
        # An ament marker shadows the whole package. Retain the matched
        # underlay libexec so launch_ros can resolve supervised executables.
        (prefix / 'lib').mkdir()
        (prefix / 'lib/rlai_navigation').symlink_to(nav_libexec, target_is_directory=True)
        prefix.rename(destination)
    print(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path, help='New ament prefix for the paired profiles')
    args = parser.parse_args()
    prepare(args.destination.resolve())
