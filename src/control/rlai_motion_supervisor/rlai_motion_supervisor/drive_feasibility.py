"""Read-only differential-drive command feasibility from actual configuration.

This module creates no ROS node and publishes nothing. The suggested common
scale describes a single command endpoint; it is not a trajectory limiter.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _number(value, name, positive=False):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be a finite number') from error
    if isinstance(value, bool) or not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f'{name} must be finite' + (' and positive' if positive else ''))
    return result


def audit_configuration(configuration, urdf, *, linear=None, angular=None):
    """Audit one reversible differential-drive pair; unknown metadata is an error."""
    try:
        params = configuration['diff_drive_controller']['ros__parameters']
        separation = _number(params['wheel_separation'], 'wheel_separation', True)
        separation *= _number(params.get('wheel_separation_multiplier', 1.), 'wheel_separation_multiplier', True)
        radius = _number(params['wheel_radius'], 'wheel_radius', True)
        axes = params['linear']['x'], params['angular']['z']
        vmin = _number(axes[0]['min_velocity'], 'linear minimum')
        vmax = _number(axes[0]['max_velocity'], 'linear maximum')
        wmax = _number(axes[1]['max_velocity'], 'angular maximum')
        wmin = _number(axes[1].get('min_velocity', -wmax), 'angular minimum')
    except (KeyError, TypeError) as error:
        raise ValueError('Incomplete diff_drive_controller geometry or velocity limits') from error
    if not (vmin <= 0 <= vmax and vmin < vmax and wmin <= 0 <= wmax and wmin < wmax):
        raise ValueError('Axis limits must be ordered and contain zero')
    root = ET.fromstring(urdf)
    wheels = {}
    for side, direction in [('left', -1), ('right', 1)]:
        names = params.get(side + '_wheel_names')
        if not isinstance(names, list) or len(names) != 1 or not isinstance(names[0], str):
            raise ValueError('This audit requires exactly one named wheel per side')
        name = names[0]
        physical = [joint for joint in root.findall('./joint') if joint.get('name') == name]
        hardware = [joint for joint in root.findall('./ros2_control/joint') if joint.get('name') == name]
        if len(physical) != 1 or len(hardware) != 1:
            raise ValueError(f'Expected one physical and one control joint for {name}')
        joint_limit = physical[0].find('limit')
        if joint_limit is None:
            raise ValueError(f'Missing URDF velocity limit for {name}')
        velocity = _number(joint_limit.get('velocity'), name + ' URDF velocity', True)
        interfaces = [entry for entry in hardware[0].findall('command_interface') if entry.get('name') == 'velocity']
        if len(interfaces) != 1:
            raise ValueError(f'Missing or ambiguous velocity interface for {name}')
        bounds = {}
        for key in ['min', 'max']:
            values = [entry.text for entry in interfaces[0].findall('param') if entry.get('name') == key]
            if len(values) != 1:
                raise ValueError(f'Missing or ambiguous {key} velocity bound for {name}')
            bounds[key] = _number(values[0], name + ' ' + key)
        lower, upper = max(bounds['min'], -velocity), min(bounds['max'], velocity)
        if not lower <= 0 <= upper or lower >= upper:
            raise ValueError(f'Invalid wheel velocity bounds for {name}')
        effective_radius = radius * _number(params.get(side + '_wheel_radius_multiplier', 1.), side + ' radius multiplier', True)
        wheels[side] = dict(name=name, direction=direction, radius_m=effective_radius,
                            minimum_rad_s=lower, maximum_rad_s=upper)
    if wheels['left']['name'] == wheels['right']['name']:
        raise ValueError('Left and right wheels must be distinct')

    def point(v, w):
        v, w = _number(v, 'linear command'), _number(w, 'angular command')
        demands, scale = {}, 1.
        within_chassis = vmin <= v <= vmax and wmin <= w <= wmax
        for value, lower, upper in [(v, vmin, vmax), (w, wmin, wmax)]:
            if value > 0:
                scale = min(scale, upper / value)
            elif value < 0:
                scale = min(scale, lower / value)
        for side, wheel in wheels.items():
            demand = (v + wheel['direction'] * separation * w / 2) / wheel['radius_m']
            if not math.isfinite(demand):
                raise ValueError('Wheel demand cannot be represented as finite')
            lower, upper = wheel['minimum_rad_s'], wheel['maximum_rad_s']
            demands[side] = dict(demand_rad_s=demand, within_limit=lower <= demand <= upper,
                independent_clip_model_rad_s=min(upper, max(lower, demand)))
            if demand > 0:
                scale = min(scale, upper / demand)
            elif demand < 0:
                scale = min(scale, lower / demand)
        if scale < 1:
            scale = math.nextafter(scale, 0.)  # Stay inside the boundary after double rounding.
        within_wheels = all(p['within_limit'] for p in demands.values())
        return dict(linear_mps=v, angular_rps=w, wheels=demands,
                    feasible=within_wheels and within_chassis, wheel_limits_feasible=within_wheels,
                    within_chassis_limits=within_chassis,
                    common_scale_to_limits=scale)

    corners = [point(v, w) for v in [vmin, vmax] for w in [wmin, wmax]]
    report = dict(effective_separation_m=separation, wheel_limits=wheels,
                  chassis_axis_limits=dict(linear_min=vmin, linear_max=vmax, angular_min=wmin, angular_max=wmax),
                  configuration_box_feasible=all(p['feasible'] for p in corners), box_corners=corners,
                  scope='Read-only command endpoints; no motion, dynamics or physical qualification.')
    if (linear is None) != (angular is None):
        raise ValueError('Provide both linear and angular commands')
    if linear is not None:
        report['requested'] = point(linear, angular)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controllers', type=Path, help='Controller YAML; defaults to installed rlai_control')
    parser.add_argument('--urdf', type=Path, help='Expanded URDF or xacro; defaults to installed rlai_description')
    parser.add_argument('--linear', type=float, help='Optional endpoint in m/s; requires --angular')
    parser.add_argument('--angular', type=float, help='Optional endpoint in rad/s; requires --linear')
    args = parser.parse_args(argv)
    try:
        import yaml
        if args.controllers is None or args.urdf is None:
            from ament_index_python.packages import get_package_share_directory
            args.controllers = args.controllers or Path(get_package_share_directory('rlai_control')) / 'config/controllers.yaml'
            args.urdf = args.urdf or Path(get_package_share_directory('rlai_description')) / 'urdf/robot.urdf.xacro'
        description = args.urdf.read_text()
        if args.urdf.suffix == '.xacro':
            import xacro
            description = xacro.process_file(str(args.urdf), mappings={'sim_mode': 'gazebo'}).toxml()
        report = audit_configuration(yaml.safe_load(args.controllers.read_text()), description,
                                     linear=args.linear, angular=args.angular)
        report['inputs'] = dict(controllers=str(args.controllers.resolve()), urdf=str(args.urdf.resolve()),
            expanded_urdf_sha256=hashlib.sha256(description.encode()).hexdigest())
        print(json.dumps(report, indent=2, allow_nan=False))
        feasible = report['requested']['feasible'] if 'requested' in report else report['configuration_box_feasible']
        return 0 if feasible else 2
    except Exception as error:
        # The CLI may lack optional installed robot resources; never turn an
        # unknown configuration into a feasible result or create a ROS node.
        print(json.dumps({'error': str(error)}, allow_nan=False))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
