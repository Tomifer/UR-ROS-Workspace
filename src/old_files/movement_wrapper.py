#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2021 FZI Forschungszentrum Informatik
# Created on behalf of Universal Robots A/S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Felix Exner mauch@fzi.de
# \date    2021-08-05
#
#
# ---------------------------------------------------------------------

import sys
import time

from numpy.core.fromnumeric import swapaxes

import rospy
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        # Start a subscriber to the /tf topic to get the current position of the robot
        self.cart_pos = {'x': 0, 'y': 0, 'z': 0}
        self.cart_orient = {'x': 0, 'y': 0, 'z': 0, 'w': 0}
        self.tf = rospy.Subscriber(
            "/tf", TFMessage, self.update_cartesian_position)
        self.pub = rospy.Publisher(
            'twist_controller/command', Twist, queue_size=10)
        rospy.init_node("movement_wrapper")
        rospy.loginfo("Initializing UR Driver...")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy(
            "controller_manager/load_controller", LoadController)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.prev_movement = [[], []]
        self.prev_controller = None

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        self.switch_controller(self.cartesian_trajectory_controller)
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(
                self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )
        time.sleep(0.5)
        rospy.loginfo("Complete!")

    def is_moving(self):
        """Checks if the robot is moving"""
        return self.trajectory_client.get_state() == actionlib.GoalStatus.ACTIVE

    def send_cartesian_trajectory(self, point_list, duration_list, blocking=False):
        """Creates a Cartesian trajectory and sends it using the selected action server"""

        if point_list == self.prev_movement[0] and duration_list == self.prev_movement[1]:
            return
        self.prev_movement = [point_list, duration_list]
        self.switch_controller(self.cartesian_trajectory_controller)
        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()

        # The following list are arbitrary positions
        # Change to your own needs if desired
        pose_list = []
        for point in point_list:
            pose_list.append(geometry_msgs.Pose(
                geometry_msgs.Vector3(point[0], point[1], point[2]), geometry_msgs.Quaternion(
                    point[3], point[4], point[5], point[6])
            ))

        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo(
            "Executing trajectory using the {}".format(
                self.cartesian_trajectory_controller)
        )

        self.trajectory_client.send_goal(goal)
        if blocking:
            self.trajectory_client.wait_for_result()
            result = self.trajectory_client.get_result()
            rospy.loginfo(
                "Trajectory execution finished in state {}".format(result.error_code))

    def twist_controller(self, linear_vel, angular_vel):
        # This method will create a twist message and publish it to the twist controller
        self.switch_controller(CONFLICTING_CONTROLLERS[1])
        twist = Twist()
        twist.linear.x = linear_vel['x']
        twist.linear.y = linear_vel['y']
        twist.linear.z = linear_vel['z']
        twist.angular.x = angular_vel['x']
        twist.angular.y = angular_vel['y']
        twist.angular.z = angular_vel['z']
        self.pub.publish(twist)

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        if self.prev_controller == target_controller:
            return
        self.prev_controller = target_controller

        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)
        time.sleep(1)

    def update_cartesian_position(self, data):
        # This function is called everytime a new position is published to the /tf topic
        # It updates the current position of the robot
        self.cart_pos['x'] = data.transforms[0].transform.translation.x
        self.cart_pos['y'] = data.transforms[0].transform.translation.y
        self.cart_pos['z'] = data.transforms[0].transform.translation.z
        self.cart_orient['x'] = data.transforms[0].transform.rotation.x
        self.cart_orient['y'] = data.transforms[0].transform.rotation.y
        self.cart_orient['z'] = data.transforms[0].transform.rotation.z
        self.cart_orient['w'] = data.transforms[0].transform.rotation.w

    def get_current_position(self):
        return (self.cart_pos, self.cart_orient)

if __name__ == "__main__":
    client = TrajectoryClient()
    client.twist_controller()
    """
    point_list = []
    point_list.append([0.4, -0.5, 0.0, 1, 0, 0, 1])
    point_list.append([0.4, -0.5, 0.0, 0.707, 0.707, 0, 1])
    point_list.append([0.4, -0.5, 0.0, 0.707, 0.707, 0, 0.5])
    point_list.append([0.4, -0.5, 0.0, 0, 0, 1, 1])
    point_list.append([0.4, -0.5, 0.0, 0, 0, 0, 1])
    point_list.append([0.4, -0.5, 0.6, 0, 0, 0, 1])
    point_list.append([0.4, 0.5, 0.6, 0, 0, 0, 1])
    point_list.append([0.4, 0.5, 0.0, 0, 0, 0, 1])
    point_list.append([0.6, -0.5, 0.0, 0, 0, 0, 1])
    duration_list = [3.0, 5.0, 7.0, 9.0, 11.0]
    duration_list = [3.0, 6.0, 9.0, 14.0, 15.0, 18.0, 21.0, 24.0, 27.0, 30.0]
    client.send_cartesian_trajectory(point_list, duration_list)
    """
