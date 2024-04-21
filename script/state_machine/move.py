#!/usr/bin/env python3
# coding:utf-8

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
from geometry_msgs.msg import Point, Pose2D
from sigverse_hsrb_nav import HSRBNavigation

class Move(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(
            self, outcomes=outcomes,
            input_keys=["status"],
        )
        Logger.__init__(self, loglevel="INFO")
        self.hsrnav = HSRBNavigation()
        self.loop_counter = 0

    def execute(self, userdata):
        status = userdata.status
        self.loginfo(f"move to {status}")

        if status == "pick_up":
            try:
                object_point = rospy.get_param("/interactive_cleanup/pickup/point", [0, 0, 0])
                navigation_goal = Pose2D()
                navigation_goal.x = object_point[0] - 0.5
                navigation_goal.y = object_point[1] - 1
                navigation_goal.theta = 0

            except Exception as e:
                self.logwarn(e)
                self.loop_counter += 1
                if self.loop_counter > 100:
                    return "except"
                else:
                    return "loop"

        elif status == "clean_up":
            try:
                cleanup_point = rospy.get_param("/interactive_cleanup/cleanup/point", [0, 0, 0])
                navigation_goal = Pose2D()
                navigation_goal.x = cleanup_point[0] - 1
                navigation_goal.y = cleanup_point[1]
                navigation_goal.theta = 0

            except Exception as e:
                self.logwarn(e)
                self.loop_counter += 1
                if self.loop_counter > 100:
                    return "except"
                else:
                    return "loop"

        else:
            self.logwarn("無効なステータスが入力されています．ナビゲーションをスキップします")

        self.hsrnav.navigation(navigation_goal, "abs")

        return "next"
