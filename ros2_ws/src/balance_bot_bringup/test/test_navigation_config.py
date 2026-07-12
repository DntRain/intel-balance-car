import pathlib
import unittest

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).parents[1]
CONFIG_DIR = PACKAGE_ROOT / 'config'


class NavigationConfigTest(unittest.TestCase):
    def load_yaml(self, filename):
        with (CONFIG_DIR / filename).open(encoding='utf-8') as config_file:
            return yaml.safe_load(config_file)

    def test_slam_uses_the_robot_sensor_frames(self):
        params = self.load_yaml('slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
        self.assertEqual('map', params['map_frame'])
        self.assertEqual('odom', params['odom_frame'])
        self.assertEqual('base_link', params['base_frame'])
        self.assertEqual('/scan', params['scan_topic'])

    def test_amcl_uses_map_odom_base_link_chain(self):
        params = self.load_yaml('nav2_params.yaml')['amcl']['ros__parameters']
        self.assertEqual('map', params['global_frame_id'])
        self.assertEqual('odom', params['odom_frame_id'])
        self.assertEqual('base_link', params['base_frame_id'])
        self.assertEqual('/scan', params['scan_topic'])

    def test_map_server_exposes_yaml_filename_for_nav2_map_argument(self):
        params = self.load_yaml('nav2_params.yaml')['map_server']['ros__parameters']
        self.assertIn('yaml_filename', params)

    def test_nav2_costmaps_are_configured_for_laser_obstacles(self):
        params = self.load_yaml('nav2_params.yaml')
        for costmap in ('local_costmap', 'global_costmap'):
            obstacle_layer = params[costmap][costmap]['ros__parameters']['obstacle_layer']
            self.assertEqual('scan', obstacle_layer['observation_sources'])
            self.assertEqual('LaserScan', obstacle_layer['scan']['data_type'])
            self.assertEqual('/scan', obstacle_layer['scan']['topic'])

    def test_navigation_launches_are_installed(self):
        launch_dir = PACKAGE_ROOT / 'launch'
        self.assertTrue((launch_dir / 'slam.launch.py').is_file())
        self.assertTrue((launch_dir / 'navigation.launch.py').is_file())

    def test_sensors_launch_includes_model_lidar_and_camera(self):
        launch_path = PACKAGE_ROOT / 'launch' / 'sensors.launch.py'
        launch_source = launch_path.read_text(encoding='utf-8')
        self.assertIn("'balance_bot_description'", launch_source)
        self.assertIn("'ld03_lidar'", launch_source)
        self.assertIn("'camera.launch.py'", launch_source)

    def test_handheld_mapping_uses_laser_odometry_before_slam(self):
        launch_path = PACKAGE_ROOT / 'launch' / 'handheld_mapping.launch.py'
        launch_source = launch_path.read_text(encoding='utf-8')
        self.assertIn("package='rf2o_laser_odometry'", launch_source)
        self.assertIn("'laser_scan_topic': '/scan'", launch_source)
        self.assertIn("'odom_topic': '/odom_rf2o'", launch_source)
        self.assertIn("'publish_tf': True", launch_source)
        self.assertIn("'slam.launch.py'", launch_source)

    def test_handheld_localization_uses_rf2o_and_nav2_localization_only(self):
        launch_path = PACKAGE_ROOT / 'launch' / 'handheld_localization.launch.py'
        launch_source = launch_path.read_text(encoding='utf-8')
        self.assertIn("package='rf2o_laser_odometry'", launch_source)
        self.assertIn("'localization_launch.py'", launch_source)
        self.assertIn("DeclareLaunchArgument('map'", launch_source)
        self.assertNotIn("'bringup_launch.py'", launch_source)

    def test_ld08_pwm_uses_documented_frequency_and_duty_cycle(self):
        script_path = PACKAGE_ROOT / 'scripts' / 'ld08_pwm.py'
        script_source = script_path.read_text(encoding='utf-8')
        self.assertIn('PWM_PIN = 33', script_source)
        self.assertIn('PWM_FREQUENCY_HZ = 16_000', script_source)
        self.assertIn('PWM_DUTY_CYCLE = 60.0', script_source)
        self.assertIn('GPIO.PWM(PWM_PIN, PWM_FREQUENCY_HZ)', script_source)
        self.assertIn('pwm.ChangeDutyCycle(PWM_DUTY_CYCLE)', script_source)

    def test_sensors_launch_starts_ld08_pwm_helper(self):
        launch_source = (PACKAGE_ROOT / 'launch' / 'sensors.launch.py').read_text(encoding='utf-8')
        self.assertIn("'scripts' / 'ld08_pwm.py'", launch_source)

    def test_camera_launch_sets_the_urdf_camera_frame(self):
        launch_path = PACKAGE_ROOT / 'launch' / 'camera.launch.py'
        launch_source = launch_path.read_text(encoding='utf-8')
        self.assertIn("{'frame_id': 'camera_link'}", launch_source)
        self.assertIn("'codec_sub_topic': '/image_raw'", launch_source)
        self.assertIn("'websocket_image_topic': '/image_jpeg'", launch_source)

    def test_camera_config_path_keeps_driver_required_trailing_slash(self):
        launch_path = PACKAGE_ROOT / 'launch' / 'camera.launch.py'
        launch_source = launch_path.read_text(encoding='utf-8')
        self.assertIn("str(config_path) + '/'", launch_source)


if __name__ == '__main__':
    unittest.main()
