#!/usr/bin/env python3
"""Run one HEGE field simulation process in an isolated simulation domain."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

REPO = Path(__file__).resolve().parents[2]


def check_ros_dependencies():
    missing = []
    for executable in ['ros2', 'MicroXRCEAgent']:
        if not shutil.which(executable):
            missing.append(executable)
    try:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    except ImportError:
        missing.append('ROS Python environment (source /opt/ros/humble/setup.bash)')
    else:
        for package in ['twist_mux', 'hege_bringup', 'hege_px4_bridge', 'hege_px4_sensors', 'px4_msgs']:
            try:
                get_package_prefix(package)
            except PackageNotFoundError:
                missing.append(package)
    if missing:
        raise ValueError('Missing: ' + ', '.join(missing) + '.\n'
                         'Pull this branch and use VS Code: Dev Containers: Rebuild Container.\n'
                         'Then source /opt/ros/humble/setup.bash and hege_ws/install/setup.bash.\n'
                         'See simulation/hege_field/README.md for the workspace build.')


def command_for(part, runtime, env):
    manifest = json.loads((runtime / 'manifest.json').read_text())
    px4 = Path(manifest['px4'])
    env.update(ROS_DOMAIN_ID='74', ROS_LOCALHOST_ONLY='0', GZ_PARTITION='hege-field-sim')
    if part == 'world':
        plugins = REPO / '.hege_sim/plugin_build'
        if not (plugins / 'libHegeWheelDrive.so').is_file():
            raise ValueError('Build the simulation plugin first; see simulation/hege_field/README.md')
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(plugins) + os.pathsep + env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        return ['gz', 'sim', '-r', str(runtime / 'hege_field.sdf')]
    if part == 'ros':
        check_ros_dependencies()
        return ['ros2', 'launch', 'hege_bringup', 'hege_field.launch.py']
    binary = px4 / 'build/px4_sitl_default/bin/px4'
    if not binary.is_file():
        raise ValueError(f'Build first: make -C {px4} px4_sitl')
    version = subprocess.check_output(['git', '-C', str(px4), 'describe', '--tags', '--exact-match'], text=True).strip()
    if version != 'v1.16.1':
        raise ValueError('PX4 checkout changed; v1.16.1 is required.')
    # Remove inherited simulator overrides, then set this simulation profile.
    for key in list(env):
        if key.startswith(('PX4_PARAM_', 'PX4_GZ_', 'PX4_SIM_')):
            del env[key]
    env.update(PX4_SYS_AUTOSTART='4012', PX4_SIMULATOR='gz',
               PX4_SIM_MODEL='gz_rover_ackermann', PX4_GZ_STANDALONE='1',
               PX4_GZ_MODEL_NAME='hege_px4', PX4_GZ_WORLD='hege_field',
               PX4_UXRCE_DDS_PORT='8889')
    params = {
        'UXRCE_DDS_DOM_ID': 74, 'UXRCE_DDS_PRT': 8889,
        'RA_WHEEL_BASE': 1.90, 'RA_MAX_STR_ANG': 0.5759,
        'RA_STR_RATE_LIM': 34.3775,
        'RO_SPEED_LIM': 1.0, 'RO_YAW_RATE_LIM': 45,
        'RO_YAW_P': 3.0, 'RO_SPEED_P': 1.0, 'RO_SPEED_I': 0.1,
        'RO_ACCEL_LIM': 0.5, 'RO_DECEL_LIM': 1.0, 'RO_JERK_LIM': 1.0,
        # GZ wheel output = uint16 output - 100, in rad/s.
        # Range [0,200] -> [-100,+100] shaft rad/s, 10:1 reduction;
        # r=0.4 -> +/-4.0 m/s ideal, in 0.04 m/s increments.
        # This is a simulator model mapping, NOT a measured real HEGE value.
        'SIM_GZ_WH_MIN1': 0, 'SIM_GZ_WH_MAX1': 200, 'SIM_GZ_WH_DIS1': 100,
        'RO_MAX_THR_SPEED': 4.0,
        'SIM_GZ_SV_MINA1': -32.99664, 'SIM_GZ_SV_MAXA1': 32.99664,
        'SIM_GZ_SV_REV': 1,
    }
    env.update({f'PX4_PARAM_{key}': str(value) for key, value in params.items()})
    rootfs = runtime / 'px4_rootfs'
    rootfs.mkdir(exist_ok=True)
    return [str(binary), str(px4 / 'build/px4_sitl_default/etc'),
            '-w', str(rootfs), '-s', 'etc/init.d-posix/rcS']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('part', choices=['world', 'px4', 'ros'])
    parser.add_argument('--runtime', type=Path, default=REPO / '.hege_sim')
    args = parser.parse_args()
    try:
        env = os.environ.copy()
        command = command_for(args.part, args.runtime.resolve(), env)
        if not shutil.which(command[0]):
            raise ValueError(f'{command[0]} unavailable; source the Humble workspace or install the executable.')
        print('Starting:', ' '.join(command), flush=True)
        os.execvpe(command[0], command, env)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'{exc}\nSee simulation/hege_field/README.md for setup.\n')
