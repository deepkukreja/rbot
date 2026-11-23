"""Explicit process-scoped Fast DDS loopback transport for local development."""
import argparse
import os
from pathlib import Path


def bundled_profile():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('rlai_localization'))/'config/loopback_udp.xml'


def loopback_environment(profile, environment, *, allow_profile_override=False):
    """Copy the environment; apply XML without RMW's LOCALHOST SHM override."""
    path=Path(profile).resolve(strict=True)
    if not path.is_file():
        raise ValueError('DDS profile must be a file')
    changed=dict(environment)
    middleware=changed.get('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    if middleware not in ('','rmw_fastrtps_cpp','rmw_fastrtps_dynamic_cpp'):
        raise ValueError('Loopback UDP mode requires Fast DDS; existing middleware was preserved')
    inherited=changed.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if inherited and Path(inherited).resolve()!=path and not allow_profile_override:
        raise ValueError('An existing DDS profile would be replaced; merge its settings and select it explicitly with --profile')
    changed['RMW_IMPLEMENTATION']=middleware or 'rmw_fastrtps_cpp'
    changed['FASTRTPS_DEFAULT_PROFILES_FILE']=str(path)
    changed['ROS_AUTOMATIC_DISCOVERY_RANGE']='SYSTEM_DEFAULT'
    changed.pop('ROS_LOCALHOST_ONLY',None)
    changed.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
    return changed


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,help='Explicit merged custom loopback profile')
    parser.add_argument('command',nargs=argparse.REMAINDER)
    arguments=parser.parse_args(args)
    command=arguments.command
    if command[:1]==['--']:
        command=command[1:]
    if not command:
        parser.error('provide a command, for example: -- ros2 launch rlai_localization localization_amcl.launch.py ...')
    try:
        environment=loopback_environment(arguments.profile or bundled_profile(),os.environ,
                                         allow_profile_override=arguments.profile is not None)
    except (ValueError,OSError) as error:
        parser.error(str(error))
    try:
        os.execvpe(command[0],command,environment)
    except OSError as error:
        parser.exit(127,str(error)+'\n')


if __name__=='__main__':
    main()
