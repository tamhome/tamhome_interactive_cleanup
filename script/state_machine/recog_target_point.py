#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from tamlib.utils import Logger
from std_srvs.srv import SetBool, SetBoolRequest

from interactive_cleanup.msg import InteractiveCleanupMsg


class RecogTargetPoint(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        """指差し場所を特定
        """
        return "move"
