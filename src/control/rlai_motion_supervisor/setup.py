from setuptools import find_packages, setup
from glob import glob

setup(name='rlai_motion_supervisor',version='0.2.0',packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/rlai_motion_supervisor']),
                ('share/rlai_motion_supervisor',['package.xml','README.md','GUARDED_NAVIGATION.md']),
                ('share/rlai_motion_supervisor/scripts',glob('scripts/*.py'))],
    install_requires=['setuptools'],zip_safe=True,
    maintainer='Robot Buildworks',maintainer_email='engineering@robotbuildworks.local',
    description='Local exclusive motion ownership and monotonic watchdog',license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts':['motion_supervisor = rlai_motion_supervisor.node:main',
                                      'nav_adapter = rlai_motion_supervisor.nav_adapter:main',
                                      'reset_sim_localization = rlai_motion_supervisor.localization_reset:main',
                                      'audit_drive_limits = rlai_motion_supervisor.drive_feasibility:main']})
