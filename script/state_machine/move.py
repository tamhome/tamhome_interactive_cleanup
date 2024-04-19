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

from human_navigation.msg import HumanNaviMsg
from human_navigation.msg import HumanNaviAvatarStatus
from human_navigation.msg import HumanNaviTaskInfo
from human_navigation.msg import HumanNaviDestination
from human_navigation.msg import HumanNaviObjectInfo
from human_navigation.msg import HumanNaviGuidanceMsg
from human_navigation.msg import HumanNaviObjectStatus

from geometry_msgs.msg import Point


class Move(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(
            self, outcomes=outcomes,
            input_keys=["task_info"],
            output_keys=["task_info"]
        )
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        return "next"
