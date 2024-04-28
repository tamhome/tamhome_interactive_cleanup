#!/usr/bin/env python3
# coding:utf-8

import tf
import math
import numpy as np
import os
import ast
import time
import json
import rospy
import smach
import smach_ros
import roslib.packages
from typing import List
from tamlib.utils import Logger
from geometry_msgs.msg import Point, Pose2D, Pose, PoseStamped
from sigverse_hsrb_nav import HSRBNavigation
from sigverse_hsrlib import MoveJoints
from sigverse_hsrlib import HSRBMoveIt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from interactive_cleanup.msg import InteractiveCleanupMsg
from tamlib.tf import Transform, euler2quaternion

class Move(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(
            self, outcomes=outcomes,
            input_keys=["status"],
        )
        Logger.__init__(self, loglevel="INFO")
        self.hsrnav = HSRBNavigation()
        self.move_joint = MoveJoints()
        self.moveit = HSRBMoveIt()
        self.loop_counter = 0
        self.loop_mun_num = 1000

        self.move_point = {
            "cardboard_box": {
                "nav_pose": Pose2D(x=3.25, y=-2.25, theta=-1.57),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [3.25, -2.83, 0.7]
            },
            "square_low_table": {
                "nav_pose": Pose2D(x=0.45, y=1.87, theta=1.57),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [0.33, 2.53, 0.7]
            },
            "wagon#01": {
                "nav_pose": Pose2D(x=2.45, y=1.87, theta=1.57),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [2.33, 2.53, 0.7]
            },
            "wagon#02": {
                "nav_pose": Pose2D(x=4.15, y=2.58, theta=1.57),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [4.12, 3.03, 0.7]
            },
            "wooden_shelf": {
                "nav_pose": Pose2D(x=3.89, y=0.96, theta=0),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [4.45, 0.95, 0.7]
            },
            "white_side_table#2": {
                "nav_pose": Pose2D(x=3.89, y=-1.09, theta=0),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [4.45, -1.02, 0.7]
            },
            "white_side_table#1": {
                "nav_pose": Pose2D(x=0.98, y=-1.95, theta=-1.57),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [4.45, -1.02, 0.7]
            },
            "trash_box_for_bottle_can": {
                "nav_pose": Pose2D(x=-0.88, y=-1.05, theta=-3.14),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [-1.50, -0.93, 0.7]
            },
            "trash_box_for_bumper": {
                "nav_pose": Pose2D(x=-0.88, y=0.78, theta=-3.14),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [-1.50, 0.78, 0.7]
            },
            "trash_box_for_recycle": {
                "nav_pose": Pose2D(x=-0.88, y=2.78, theta=-3.14),
                "joint": [0.1, -0.9, 0.0, -0.65, 0.0],
                "tf": [-1.50, 2.78, 0.7]
            },
        }

        # self._pub_omni_base_controller = rospy.Publisher("/hsrb/omni_base_controller/command", JointTrajectory, queue_size=1)
        self._pub_base_rot = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
        self.pub_to_moderator = rospy.Publisher("/interactive_cleanup/message/to_moderator", InteractiveCleanupMsg, queue_size=5)

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """
        Calculate the Euclidean distance between two Pose objects.

        Parameters:
        pose1 (Pose): The first pose.
        pose2 (Pose): The second pose.

        Returns:
        float: The Euclidean distance between pose1 and pose2.
        """
        x1, y1, z1 = pose1.position.x, pose1.position.y, pose1.position.z
        x2, y2, z2 = pose2.position.x, pose2.position.y, pose2.position.z

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def calc_nearest_furniture(self, target_point: List) -> int:
        """指差しの座標と最も近いオブジェクトを検出
        """
        target_pose = Pose()
        target_pose.position.x = target_point[0]
        target_pose.position.y = target_point[1]
        target_pose.position.z = target_point[2]
        target_pose.orientation = euler2quaternion(0, 0, 0)

        min_distance = np.inf
        target_idx = 0

        for idx, detected_object in enumerate(detected_objects):
            detected_object_pose_map: Pose = self.tamtf.get_pose_with_offset(
                target_frame="map",
                source_frame=self.head_rgbd_frame,
                offset=detected_object,
            )

            distance = self.calculate_distance(target_pose, detected_object_pose_map)
            if distance < min_distance:
                min_distance = distance
                target_idx = idx

        return target_idx

    def execute(self, userdata):
        status = userdata.status
        self.loginfo(f"move to {status}")
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        for _ in range(3):
            self._pub_base_rot.publish(twist_msg)

        if status == "pick_up":
            try:
                self.loginfo("wait for human moving")
                rospy.sleep(2)
                twist_msg.angular.z = 0.0
                for _ in range(3):
                    self._pub_base_rot.publish(twist_msg)
                self.loginfo("go to pickup!")

                object_point = rospy.get_param("/interactive_cleanup/pickup/point", [0, 0, 0])
                navigation_goal = Pose2D()
                if object_point[0] > 0:
                    navigation_goal.x = object_point[0] - 1.0
                else:
                    navigation_goal.x = object_point[0] + 1.0
                    if navigation_goal.x > 3:
                        navigation_goal.x = 3
                if object_point[1] > 0:
                    navigation_goal.y = object_point[1] - 1.0
                else:
                    navigation_goal.y = object_point[1] + 1.0
                    if navigation_goal.y < -2:
                        navigation_goal.y = -2

                navigation_goal.theta = 0
                self.hsrnav.navigation(navigation_goal, "abs")

                # 物体の方を向く
                target_pose = Pose()
                target_pose.position.x = object_point[0]
                target_pose.position.y = object_point[1]
                target_pose.position.z = object_point[2]
                target_pose.orientation.x = 0
                target_pose.orientation.y = 0
                target_pose.orientation.z = 0
                target_pose.orientation.w = 1

                self.move_joint.gaze_point(target_pose, timeout=40)

            except Exception as e:
                self.logwarn(e)
                self.loop_counter += 1
                if self.loop_counter > self.loop_mun_num:
                    self.loop_counter = 0
                    return "except"
                else:
                    return "loop"

        elif status == "clean_up":
            try:
                # 対象とする家具を決定
                cleanup_target_furniture = rospy.get_param("/interactive_cleanup/cleanup/target_furniture", "floor")
                cleanup_point = rospy.get_param("/interactive_cleanup/cleanup/point", [3.25, -2.83, 0.7])

                self.loginf(f"recognized target furniture is {cleanup_target_furniture}")
                if cleanup_target_furniture == "floor":
                    try:
                        cleanup_target_furniture = "cardboard_box"
                        min_distance = 1000000
                        for key, furniture_info in self.move_point.items():
                            target_tf = furniture_info["tf"]
                            distance_x = abs(cleanup_point[0] - target_tf[0])
                            distance_y = abs(cleanup_point[1] - target_tf[1])
                            distance_z = abs(cleanup_point[2] - target_tf[2])
                            distance = distance_x + distance_y + distance_z
                            if distance < min_distance:
                                min_distance = distance
                                cleanup_target_furniture = key

                        joint = self.move_point[cleanup_target_furniture]["joint"]
                        target_tf = self.move_point[cleanup_target_furniture]["tf"]
                        self.loginfo(f"target_furniture is {cleanup_target_furniture}")
                    except Exception as e:
                        self.logwarn("片付け場所を認識できていません．デフォルトの場所に片付けます．")
                        cleanup_target_furniture = "cardboard_box"
                        navigation_goal = self.move_point[cleanup_target_furniture]["nav_pose"]
                        joint = self.move_point[cleanup_target_furniture]["joint"]
                        target_tf = self.move_point[cleanup_target_furniture]["tf"]
                else:
                    navigation_goal = self.move_point[cleanup_target_furniture]["nav_pose"]
                    joint = self.move_point[cleanup_target_furniture]["joint"]
                    target_tf = self.move_point[cleanup_target_furniture]["tf"]
            except Exception as e:
                self.logwarn("対象の家具は見つかりませんでした．デフォルトの場所に片付けます．")
                cleanup_target_furniture = "cardboard_box"
                navigation_goal = self.move_point[cleanup_target_furniture]["nav_pose"]
                joint = self.move_point[cleanup_target_furniture]["joint"]
                target_tf = self.move_point[cleanup_target_furniture]["tf"]

            try:
                self.hsrnav.navigation(navigation_goal, "abs")

                # 目標位置にTFを発行
                pose_stamped = Pose()
                pose_stamped.position.x = target_tf[0]
                pose_stamped.position.y = target_tf[1]
                pose_stamped.position.z = target_tf[2]
                pose_stamped.position.x = target_tf[0]
                orientation = tf.transformations.quaternion_from_euler(3.14, 0, 0)
                pose_stamped.orientation.x = orientation[0]
                pose_stamped.orientation.y = orientation[1]
                pose_stamped.orientation.z = orientation[2]
                pose_stamped.orientation.w = orientation[3]

                self.moveit.move_to_pose(pose_stamped)
                self.move_joint.gripper(3.14)
                rospy.sleep(5)

                # msg = rospy.wait_for_message("/interactive_cleanup/message/to_robot", InteractiveCleanupMsg, timeout=1)

                for _ in range(5):
                    msg = InteractiveCleanupMsg()
                    msg.message = "Task_finished"
                    msg.detail = "Task_finished"
                    self.pub_to_moderator.publish(msg)
                    rospy.sleep(0.2)

                    # self.move_joint.move_arm_by_pose(joint[0], joint[1], joint[2], joint[3], joint[4])

            except Exception as e:
                self.logwarn(e)
                self.loop_counter += 1
                if self.loop_counter > self.loop_mun_num:
                    self.loop_counter = 0
                    return "except"
                else:
                    return "loop"

        else:
            self.logwarn("無効なステータスが入力されています．ナビゲーションをスキップします")

        return "next"
