#!/usr/bin/env python3
# coding:utf-8

import tf
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
                "tf": [3.25, -2.95, 0.7]
                }
        }

    def execute(self, userdata):
        status = userdata.status
        self.loginfo(f"move to {status}")

        if status == "pick_up":
            try:
                rospy.sleep(0.5)
                object_point = rospy.get_param("/interactive_cleanup/pickup/point", [0, 0, 0])
                navigation_goal = Pose2D()
                navigation_goal.x = object_point[0] - 0.5
                navigation_goal.y = object_point[1] - 1
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

                self.move_joint.gaze_point(target_pose)

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
                cleanup_point = rospy.get_param("/interactive_cleanup/cleanup/point", [0, 0, 0])
                # 対象とする家具を決定
                target_furniture = "cardboard_box"
                try:
                    navigation_goal = self.move_point[target_furniture]["nav_pose"]
                    joint = self.move_point[target_furniture]["joint"]
                    target_tf = self.move_point[target_furniture]["tf"]
                except Exception as e:
                    self.logwarn(e)
                    navigation_goal = Pose2D()
                    navigation_goal.x = 1
                    navigation_goal.y = 0
                    navigation_goal.theta = 0

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
