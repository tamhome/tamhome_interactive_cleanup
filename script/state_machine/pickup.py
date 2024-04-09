#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from tamlib.utils import Logger
from std_srvs.srv import SetBool, SetBoolRequest


class Pickup(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def exec(self, userdata):
        """
        """
        pass
