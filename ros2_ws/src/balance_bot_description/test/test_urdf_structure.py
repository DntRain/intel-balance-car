import pathlib
import unittest
import xml.etree.ElementTree as ET


URDF_PATH = pathlib.Path(__file__).parents[1] / 'urdf' / 'balance_bot.urdf.xacro'


class UrdfStructureTest(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(URDF_PATH).getroot()

    def test_declares_navigation_sensor_links(self):
        links = {element.attrib['name'] for element in self.root.findall('link')}
        self.assertTrue({'base_link', 'laser_link', 'camera_link'} <= links)

    def test_declares_fixed_sensor_joints_from_base_link(self):
        joints = {
            element.attrib['name']: element
            for element in self.root.findall('joint')
        }
        for joint_name, child_name in [('laser_joint', 'laser_link'),
                                       ('camera_joint', 'camera_link')]:
            joint = joints[joint_name]
            self.assertEqual('fixed', joint.attrib['type'])
            self.assertEqual('base_link', joint.find('parent').attrib['link'])
            self.assertEqual(child_name, joint.find('child').attrib['link'])

    def test_uses_scalar_sensor_pose_arguments(self):
        argument_names = {element.attrib['name'] for element in self.root.findall(
            '{http://www.ros.org/wiki/xacro}arg')}
        self.assertTrue({'laser_x', 'laser_y', 'laser_z',
                         'camera_x', 'camera_y', 'camera_z'} <= argument_names)

    def test_launch_marks_robot_description_as_string(self):
        launch_path = URDF_PATH.parents[1] / 'launch' / 'description.launch.py'
        launch_source = launch_path.read_text(encoding='utf-8')
        self.assertIn('ParameterValue(robot_description, value_type=str)', launch_source)


if __name__ == '__main__':
    unittest.main()
