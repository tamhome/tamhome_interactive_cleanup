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
from sigverse_hsrlib import MoveJoints

from human_navigation.msg import HumanNaviMsg
from human_navigation.msg import HumanNaviAvatarStatus
from human_navigation.msg import HumanNaviTaskInfo
from human_navigation.msg import HumanNaviDestination
from human_navigation.msg import HumanNaviObjectInfo
from human_navigation.msg import HumanNaviGuidanceMsg
from human_navigation.msg import HumanNaviObjectStatus

from geometry_msgs.msg import Point
from interactive_cleanup.msg import InteractiveCleanupMsg


class CleanUp(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(
            self, outcomes=outcomes,
            input_keys=["task_info"],
            output_keys=["task_info"]
        )
        Logger.__init__(self, loglevel="INFO")
        self.move_joint = MoveJoints()
        self.pub_to_moderator = rospy.Publisher("/interactive_cleanup/message/to_moderator", InteractiveCleanupMsg, queue_size=5)

    def execute(self, userdata):
        rospy.sleep(4)
        self.move_joint.gripper(3.14)
        for _ in range(5):
            msg = InteractiveCleanupMsg()
            msg.message = "Task_finished"
            msg.detail = "Task_finished"
            self.pub_to_moderator.publish(msg)
            rospy.sleep(0.2)

        self.loginfo("complete interactive cleanup task!")

        return "success"
