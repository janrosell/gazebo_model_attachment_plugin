#!/usr/bin/env python

import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from gazebo_model_attachment_plugin.srv import Attach, AttachRequest, AttachResponse
from gazebo_model_attachment_plugin.srv import Detach, DetachRequest, DetachResponse


class TestModelAttachmentPlugin(unittest.TestCase):
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
        cls.__get_model_state_srv = rospy.ServiceProxy(
            name='/gazebo/get_model_state',
            service_class=GetModelState
        )
        cls.__attach_srv.wait_for_service(timeout=30)
        cls.__detach_srv.wait_for_service(timeout=30)
        cls.__get_model_state_srv.wait_for_service(timeout=30)

    def test_attach(self):
        # Test Model Attachment

        response = self.__attach_srv.call(
            AttachRequest(
                joint_name='test_attachment_box',
                model_name_1='box',
                link_name_1='attachment_link',
                model_name_2='sphere',
                link_name_2='attachment_link'
            )
        )
        self.assertIsInstance(response, AttachResponse)
        self.assertTrue(response.success)

        response = self.__attach_srv.call(
            AttachRequest(
                joint_name='test_attachment_cylinder',
                model_name_1='box',
                link_name_1='attachment_link',
                model_name_2='cylinder',
                link_name_2='attachment_link'
            )
        )
        self.assertIsInstance(response, AttachResponse)
        self.assertTrue(response.success)

        # Ensure the sphere is correctly set as a child of the box

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
        self.assertIsInstance(response, SetModelStateResponse)

        box_model_state = self.__get_model_state_srv.call(
            GetModelStateRequest(
                model_name='box'
            )
        )
        self.assertIsInstance(box_model_state, GetModelStateResponse)
        self.assertTrue(box_model_state.success)

        sphere_model_state = self.__get_model_state_srv.call(
            GetModelStateRequest(
                model_name='sphere'
            )
        )
        self.assertIsInstance(sphere_model_state, GetModelStateResponse)
        self.assertTrue(sphere_model_state.success)

        self.assertEqual(sphere_model_state.pose.position.y,
                         box_model_state.pose.position.y)
        self.assertEqual(sphere_model_state.pose.position.z,
                         box_model_state.pose.position.z)
        self.assertEqual(sphere_model_state.twist, box_model_state.twist)

        # Test Model Detachment

        response = self.__detach_srv.call(
            DetachRequest(
                joint_name='test_attachment_box',
                model_name_1='box',
                model_name_2='sphere'
            )
        )
        self.assertIsInstance(response, DetachResponse)
        self.assertTrue(response.success)

        # Ensure Cylinder isn't dropped as a child when the Sphere is

        response = self.__set_model_state_srv.call(
            SetModelStateRequest(
                model_state=ModelState(
                    model_name='box',
                    pose=Pose(
                        position=Point(2, 2, 2),
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
        self.assertIsInstance(response, SetModelStateResponse)

        box_model_state = self.__get_model_state_srv.call(
            GetModelStateRequest(
                model_name='box'
            )
        )
        self.assertIsInstance(box_model_state, GetModelStateResponse)
        self.assertTrue(box_model_state.success)

        cylinder_model_state = self.__get_model_state_srv.call(
            GetModelStateRequest(
                model_name='cylinder'
            )
        )
        self.assertIsInstance(cylinder_model_state, GetModelStateResponse)
        self.assertTrue(cylinder_model_state.success)

        self.assertEqual(cylinder_model_state.pose.position.y,
                         box_model_state.pose.position.y)
        self.assertEqual(cylinder_model_state.pose.position.z,
                         box_model_state.pose.position.z)
        self.assertEqual(cylinder_model_state.twist, box_model_state.twist)


if __name__ == '__main__':
    rospy.init_node('test_plugin')

    rostest.rosrun('world_model_monitor',
                   'test_model_attachment_plugin', TestModelAttachmentPlugin)
