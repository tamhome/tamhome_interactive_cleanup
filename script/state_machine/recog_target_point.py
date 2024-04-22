#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from tamlib.utils import Logger
from std_srvs.srv import SetBool, SetBoolRequest

from interactive_cleanup.msg import InteractiveCleanupMsg


class RecogTargetPoint(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=[],
            output_keys=["status"]
        )
        self.srv_focus_person_enable = rospy.ServiceProxy("/action_recognition_server/run_enable", SetBool)
        self.srv_pointing_estimation_enable = rospy.ServiceProxy("/pointing_estimation/run_enable", SetBool)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        """指差し場所を特定
        """
        self.loginfo("start focus node")

        req = SetBoolRequest(True)
        self.srv_focus_person_enable(req)
        self.srv_pointing_estimation_enable(req)

        while not rospy.is_shutdown():
            # req = SetBoolRequest(True)
            # self.srv_focus_person_enable(req)
            # self.srv_pointing_estimation_enable(req)
            start_flag = rospy.get_param("/interactive_cleanup/task/start", False)
            if start_flag:
                self.loginfo("stop focus node")
                req = SetBoolRequest(False)
                self.srv_focus_person_enable(req)
                self.srv_pointing_estimation_enable(req)
                break
            else:
                rospy.sleep(0.5)

        userdata.status = "pick_up"

        return "move"
