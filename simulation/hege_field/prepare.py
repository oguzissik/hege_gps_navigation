#!/usr/bin/env python3
"""Build HEGE/PX4 simulation assets without modifying either source checkout."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

# Scene assets (terrain, textures, world) are vendored in ./assets. They were
# copied once from the teammate's repository at this commit; kept for provenance.
FRIEND = '4170c7bea836b90559b304b609252dedfc303ffe'
GZ_MODELS = 'e05f4312d3f28aa621157610584a4870406cb6d3'
REPO = Path(__file__).resolve().parents[2]
ASSETS = Path(__file__).resolve().parent / 'assets'
LENGTH, TRACK, REAR_RADIUS, FRONT_RADIUS = 1.90, 1.55, 0.40, 0.275
STEERING = 0.5759  # virtual bicycle angle, matching last recorded RA_MAX_STR_ANG


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def git_blob(repo, revision, path):
    return subprocess.check_output(['git', '-C', str(repo), 'show', f'{revision}:{path}'])


def write_xml(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def inertia(link, mass, xx, yy, zz):
    node = element(link, 'inertial')
    element(node, 'mass', mass)
    tensor = element(node, 'inertia')
    for name, value in [('ixx', xx), ('iyy', yy), ('izz', zz),
                        ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        element(tensor, name, value)


def shape(link, name, pose, kind, dimensions, colour, collision=True):
    for tag in (['visual', 'collision'] if collision else ['visual']):
        node = element(link, tag, name=f'{name}_{tag}')
        element(node, 'pose', pose)
        geom = element(element(node, 'geometry'), kind)
        for key, value in dimensions.items():
            element(geom, key, value)
        if tag == 'visual':
            material = element(node, 'material')
            element(material, 'ambient', colour)
            element(material, 'diffuse', colour)
        else:
            friction = element(element(element(node, 'surface'), 'friction'), 'ode')
            element(friction, 'mu', 0.8)
            element(friction, 'mu2', 0.8)


def joint(model, name, parent, child, axis, steering=False):
    node = element(model, 'joint', name=name, type='revolute')
    element(node, 'parent', parent)
    element(node, 'child', child)
    element(node, 'pose', '0 0 0 0 0 0', relative_to=child)
    axis_node = element(node, 'axis')
    element(axis_node, 'xyz', axis)
    limit = element(axis_node, 'limit')
    # Inner wheel angle exceeds virtual bicycle angle in an Ackermann linkage.
    element(limit, 'lower', -0.9 if steering else -1e16)
    element(limit, 'upper', 0.9 if steering else 1e16)
    element(limit, 'velocity', 0.60 if steering else 15)
    element(limit, 'effort', 5000 if steering else 3000)


def model_xml(px4):
    root = ET.Element('sdf', version='1.9')
    model = element(root, 'model', name='hege_px4', canonical_link='base_link')
    element(model, 'self_collide', 'false')
    base = element(model, 'link', name='base_link')
    element(base, 'pose', '0 0 1.10 0 0 0')
    mass = 1174.0  # total 1300 minus 2*35 rear, 2*20 front, 2*8 knuckles
    inertia(base, mass, mass*(1.35**2+0.5**2)/12,
            mass*(2.2**2+0.5**2)/12, mass*(2.2**2+1.35**2)/12)
    shape(base, 'body', '0 0 -0.15 0 0 0', 'box', {'size': '2.20 1.35 0.50'}, '1 0.45 0 1')
    shape(base, 'imu', '0 0 0.10 0 0 0', 'box', {'size': '0.12 0.08 0.04'}, '0.05 0.25 0.8 1', False)
    shape(base, 'gps', '0 0 0.20 0 0 0', 'cylinder', {'radius': 0.09, 'length': 0.04}, '0.9 0.9 0.9 1', False)
    # Preserve the exact v1.16.1 GZBridge sensor names and scoped topics.
    sensors = ET.fromstring(git_blob(px4 / 'Tools/simulation/gz', GZ_MODELS,
                                    'models/rover_ackermann/model.sdf'))
    for sensor in sensors.findall('./model/link[@name="base_link"]/sensor'):
        base.append(copy.deepcopy(sensor))
    for axle, x, radius, wheel_mass in [('rear', -LENGTH/2, REAR_RADIUS, 35),
                                       ('front', LENGTH/2, FRONT_RADIUS, 20)]:
        for side, sign in [('left', 1), ('right', -1)]:
            name = f'{side}_{axle}_wheel'
            parent = 'base_link'
            if axle == 'front':
                parent = f'{side}_steer_link'
                knuckle = element(model, 'link', name=parent)
                element(knuckle, 'pose', f'{x} {sign*TRACK/2} {radius} 0 0 0')
                inertia(knuckle, 8, 0.1, 0.1, 0.1)
                joint(model, f'{side}_steer_joint', 'base_link', parent, '0 0 1', True)
            wheel = element(model, 'link', name=name)
            element(wheel, 'pose', f'{x} {sign*TRACK/2} {radius} 0 0 0')
            radial = wheel_mass*(3*radius**2+0.2**2)/12
            inertia(wheel, wheel_mass, radial, wheel_mass*radius**2/2, radial)
            shape(wheel, name, '0 0 0 -1.5707963267948966 0 0', 'cylinder',
                  {'radius': radius, 'length': 0.20}, '0.1 0.1 0.1 1')
            joint(model, f'{name}_joint', parent, name, '0 1 0')
    drive = element(model, 'plugin', filename='HegeWheelDrive', name='hege::WheelDrive')
    element(drive, 'gear_ratio', 10.0)
    element(drive, 'joint_name', 'left_rear_wheel_joint')
    element(drive, 'joint_name', 'right_rear_wheel_joint')
    steering = element(model, 'plugin', filename='gz-sim-ackermann-steering-system',
                       name='gz::sim::systems::AckermannSteering')
    for key, value in [('steering_only', 'true'), ('sub_topic', 'servo_0'),
                       ('left_steering_joint', 'left_steer_joint'),
                       ('right_steering_joint', 'right_steer_joint'),
                       ('wheel_separation', TRACK), ('wheel_base', LENGTH),
                       ('steering_limit', STEERING), ('steer_p_gain', 5.0)]:
        element(steering, key, value)
    # Deliberately no Gazebo cmd_vel input and no ground truth estimator input.
    return root


def prepare(px4, output):
    version = subprocess.check_output(['git', '-C', str(px4), 'describe', '--tags', '--exact-match'], text=True).strip()
    if version != 'v1.16.1':
        raise ValueError(f'Expected PX4 v1.16.1, got {version}')
    output.mkdir(parents=True, exist_ok=True)
    assets = output / 'assets'
    assets.mkdir(exist_ok=True)
    for relative in ['field_heightmap.pgm', 'textures/dirt_diffusespecular.png', 'textures/flat_normal.png']:
        source = ASSETS / relative
        if not source.is_file():
            raise ValueError(f'Missing vendored asset {source}')
        target = assets / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    world_root = ET.fromstring((ASSETS / 'hege_field.world').read_bytes())
    world = world_root.find('world')
    world.set('name', 'hege_field')
    # Remove duplicated physics/scene declarations from upstream.
    for tag in ['physics', 'scene']:
        for duplicate in world.findall(tag)[1:]:
            world.remove(duplicate)
    world.find('physics/max_step_size').text = '0.001'
    for uri in world.findall('.//uri') + world.findall('.//diffuse') + world.findall('.//normal'):
        prefix = 'file:///workspace/src/hege_description/worlds/'
        if uri.text and uri.text.startswith(prefix):
            uri.text = (assets / uri.text[len(prefix):]).as_uri()
    for system in ['AirPressure', 'Magnetometer']:
        filename = 'air-pressure' if system == 'AirPressure' else 'magnetometer'
        element(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{system}')
    # Keep the imported terrain elevations; spawn 0.10 m above its highest point.
    model = model_xml(px4).find('model')
    element(model, 'pose', '0 0 -0.24 0 0 0')
    world.append(model)
    write_xml(world_root, output / 'hege_field.sdf')
    (output / 'manifest.json').write_text(json.dumps({
        'px4': str(px4), 'px4_version': version, 'friend_commit': FRIEND,
        'gz_models_commit': GZ_MODELS, 'world': 'hege_field', 'model': 'hege_px4',
    }, indent=2) + '\n')
    print(f'Prepared {output / "hege_field.sdf"}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4', required=True, type=Path)
    parser.add_argument('--output', type=Path, default=REPO / '.hege_sim')
    args = parser.parse_args()
    try:
        prepare(args.px4.expanduser().resolve(), args.output.expanduser().resolve())
    except (subprocess.CalledProcessError, ValueError) as exc:
        parser.exit(1, f'{exc}\nInitialize the PX4 v1.16.1 Gazebo submodule first (Tools/simulation/gz).\n')
