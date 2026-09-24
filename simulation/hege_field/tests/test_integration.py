import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f'{name}.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


prepare = module('prepare')
runner = module('run')
SENSORS = b'''<sdf><model><link name="base_link">
<sensor name="imu_sensor" type="imu"/><sensor name="navsat_sensor" type="navsat"/>
<sensor name="magnetometer_sensor" type="magnetometer"/>
<sensor name="air_pressure_sensor" type="air_pressure"/>
</link></model></sdf>'''


class FieldIntegration(unittest.TestCase):
    def setUp(self):
        with patch.object(prepare, 'git_blob', return_value=SENSORS):
            self.model = prepare.model_xml(Path('/px4')).find('model')

    def test_geometry_and_mass(self):
        links = {link.get('name'): link for link in self.model.findall('link')}
        self.assertAlmostEqual(sum(float(link.findtext('inertial/mass')) for link in links.values()), 1300)
        for side in ['left', 'right']:
            front = list(map(float, links[f'{side}_front_wheel'].findtext('pose').split()))
            rear = list(map(float, links[f'{side}_rear_wheel'].findtext('pose').split()))
            self.assertAlmostEqual(front[0] - rear[0], 1.90)
            for axle, pose in [('front', front), ('rear', rear)]:
                radius = float(links[f'{side}_{axle}_wheel'].findtext('collision/geometry/cylinder/radius'))
                self.assertAlmostEqual(pose[2] - radius, 0)

    def test_only_px4_actuator_interfaces(self):
        self.assertNotIn('cmd_vel', ET.tostring(self.model, encoding='unicode'))
        steer = self.model.find("plugin[@name='gz::sim::systems::AckermannSteering']")
        self.assertEqual(steer.findtext('steering_only'), 'true')
        self.assertEqual(steer.findtext('sub_topic'), 'servo_0')
        self.assertAlmostEqual(float(steer.findtext('wheel_separation')), 1.55)
        drive = self.model.find("plugin[@name='hege::WheelDrive']")
        self.assertEqual([j.text for j in drive.findall('joint_name')],
                         ['left_rear_wheel_joint', 'right_rear_wheel_joint'])
        self.assertEqual(float(drive.findtext('gear_ratio')), 10)

    def test_ackermann_inner_angle_fits_joint_limit(self):
        angle = math.atan(prepare.LENGTH * math.tan(prepare.STEERING) /
                          (prepare.LENGTH - prepare.TRACK/2 * math.tan(prepare.STEERING)))
        for name in ['left_steer_joint', 'right_steer_joint']:
            limit = float(self.model.findtext(f"joint[@name='{name}']/axis/limit/upper"))
            self.assertLess(angle, limit)

    def test_sensor_contract_and_link_references(self):
        sensors = self.model.findall("link[@name='base_link']/sensor")
        self.assertEqual({s.get('name') for s in sensors},
                         {'imu_sensor', 'navsat_sensor', 'magnetometer_sensor', 'air_pressure_sensor'})
        links = {link.get('name') for link in self.model.findall('link')}
        for joint in self.model.findall('joint'):
            self.assertIn(joint.findtext('parent'), links)
            self.assertIn(joint.findtext('child'), links)

    def test_runtime_isolation_and_speed_mapping(self):
        with tempfile.TemporaryDirectory() as temp:
            runtime = Path(temp)
            px4 = runtime / 'px4'
            binary = px4 / 'build/px4_sitl_default/bin/px4'
            binary.parent.mkdir(parents=True)
            binary.touch()
            (runtime / 'manifest.json').write_text(json.dumps({'px4': str(px4)}))
            env = {'ROS_DOMAIN_ID': '73', 'PX4_PARAM_RO_SPEED_LIM': '99', 'PX4_GZ_MODEL_POSE': 'bad'}
            with patch.object(runner.subprocess, 'check_output', return_value='v1.16.1\n'):
                command = runner.command_for('px4', runtime, env)
            self.assertEqual(env['ROS_DOMAIN_ID'], '74')
            self.assertEqual(env['PX4_PARAM_UXRCE_DDS_PRT'], '8889')
            self.assertEqual(env['PX4_PARAM_RO_SPEED_LIM'], '1.0')
            self.assertNotIn('PX4_GZ_MODEL_POSE', env)
            self.assertIn(str(runtime / 'px4_rootfs'), command)
            maximum = (float(env['PX4_PARAM_SIM_GZ_WH_MAX1'])-100)/10*prepare.REAR_RADIUS
            self.assertAlmostEqual(maximum, float(env['PX4_PARAM_RO_MAX_THR_SPEED']))

    def test_world_assets_are_relocated_and_duplicates_removed(self):
        source = b'''<sdf version="1.9"><world name="old">
        <physics name="a"><max_step_size>0.004</max_step_size></physics><physics name="b"/>
        <scene/><scene/><model name="field"><link name="link"><visual name="v"><geometry>
        <heightmap><uri>file:///workspace/src/hege_description/worlds/field_heightmap.pgm</uri>
        </heightmap></geometry></visual></link></model></world></sdf>'''
        def blob(repo, revision, path):
            if path.endswith('model.sdf'):
                return SENSORS
            if path.endswith('.world'):
                return source
            return b'asset-fixture'
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)
            with patch.object(prepare, 'git_blob', side_effect=blob), \
                 patch.object(prepare.subprocess, 'check_output', return_value='v1.16.1\n'):
                prepare.prepare(Path('/px4'), output)
            world = ET.parse(output / 'hege_field.sdf').getroot().find('world')
            self.assertEqual(len(world.findall('physics')), 1)
            self.assertEqual(len(world.findall('scene')), 1)
            self.assertEqual(world.findtext('.//uri'), (output / 'assets/field_heightmap.pgm').as_uri())
            self.assertEqual(len(world.findall("model[@name='hege_px4']")), 1)


if __name__ == '__main__':
    unittest.main()
