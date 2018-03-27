#!/usr/bin/env python

import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

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
        cls.__set_model_state_srv = rospy.ServiceProxy(
            name='/gazebo/set_model_state',
            service_class=SetModelState
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

        response = self.__set_model_state_srv.call(
            SetModelStateRequest(
                model_state=ModelState(
                    model_name='box',
                    pose=Pose(
                        position=Point(0, 0, 2),
                        orientation=Quaternion(x=0, y=0, z=0, w=1)
                    ),
                    twist=Twist(
                        linear=Vector3(0, 0, 0),
                        angular=Vector3(0, 0, 0)
                    ),
                    reference_frame=''
                )
            )
        )
        assert isinstance(response, SetModelStateResponse)

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
