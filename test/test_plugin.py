#!/usr/bin/env python

import unittest

import rospy
import rostest

from gazebo_ee_monitor_plugin.srv import Attach, AttachRequest, AttachResponse
from gazebo_ee_monitor_plugin.srv import Detach, DetachRequest, DetachResponse


class TestWorldModelPlugin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.__attach_srv = rospy.ServiceProxy(
            name='/gazebo/attach',
            service_class=Attach
        )
        cls.__detach_srv = rospy.ServiceProxy(
            name='/gazebo/detach',
            service_class=Detach
        )
        cls.__attach_srv.wait_for_service(timeout=30)
        cls.__detach_srv.wait_for_service(timeout=30)

    def test_attach(self):
        response = self.__attach_srv.call(
            AttachRequest(
                model_name_1='box',
                link_name_1='attachment_link',
                model_name_2='sphere',
                link_name_2='attachment_link'
            )
        )
        assert isinstance(response, AttachResponse)

        response = self.__detach_srv.call(
            DetachRequest(
                model_name_1='box',
                link_name_1='attachment_link',
                model_name_2='sphere',
                link_name_2='attachment_link'
            )
        )
        assert isinstance(response, DetachResponse)


if __name__ == '__main__':
    rospy.init_node('test_plugin')

    rostest.rosrun('world_model_monitor', 'test_world_model_plugin', TestWorldModelPlugin)
