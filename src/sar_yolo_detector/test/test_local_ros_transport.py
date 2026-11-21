import json
import unittest
from unittest.mock import patch
from sar_yolo_detector.local_ros_transport import local_ros_environment


class LocalROSTransportTest(unittest.TestCase):
    def test_mirrored_wsl_uses_existing_host_scope_linux_address(self):
        original={'ROS_MASTER_URI':'http://127.0.0.1:11461','NO_PROXY':'example.local'}
        replies=[json.dumps([dict(dev='loopback0')]),json.dumps([dict(addr_info=[
            dict(family='inet',scope='host',local='127.0.0.1'),
            dict(family='inet',scope='global',local='10.255.255.254')])])]
        with patch('subprocess.check_output',side_effect=replies):env=local_ros_environment(original)
        self.assertEqual(env['ROS_IP'],'10.255.255.254')
        self.assertEqual(env['ROS_MASTER_URI'],'http://10.255.255.254:11461')
        self.assertEqual(env['SAR_LOCAL_WEB_HOST'],'10.255.255.254')
        self.assertIn('10.255.255.254',env['NO_PROXY'])
        self.assertEqual(env['no_proxy'],env['NO_PROXY'])
        self.assertNotIn('ROS_IP',original)

    def test_explicit_network_and_remote_master_are_preserved(self):
        with patch('subprocess.check_output') as command:
            for env in ({'ROS_IP':'192.168.1.3'},{'ROS_HOSTNAME':'robot'},
                        {'ROS_MASTER_URI':'http://robot:11311'}):
                self.assertEqual(local_ros_environment(env),env)
            command.assert_not_called()

    def test_native_loopback_and_missing_tools_preserve_environment(self):
        with patch('subprocess.check_output',return_value='[{"dev":"lo"}]'):
            self.assertEqual(local_ros_environment({}),{})
        with patch('subprocess.check_output',side_effect=FileNotFoundError):
            self.assertEqual(local_ros_environment({}),{})


if __name__=='__main__':unittest.main()
