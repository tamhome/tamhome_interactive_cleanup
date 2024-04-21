#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from tamlib.utils import Logger
from std_srvs.srv import SetBool, SetBoolRequest

from interactive_cleanup.msg import InteractiveCleanupMsg


class Init(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        return "next"


class Wait4Start(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.pub_to_moderator = rospy.Publisher("/interactive_cleanup/message/to_moderator", InteractiveCleanupMsg, queue_size=5)

    def wait_for_ready_msg(self, ready_msg="Are_you_ready?") -> None:
        """ready_for_messageの送信を待つ関数
        Args:
            ready_msg(string): 開始の際に送信される特定のメッセージを指定
            defaults to Are_you_ready?
        """
        self.loginfo(f"wait for ready message: {ready_msg}")
        is_wait_ready = rospy.get_param("/interactive_cleanup/wait_to_ready", True)
        while not rospy.is_shutdown():
            if is_wait_ready is False:
                break
            msg = rospy.wait_for_message("/interactive_cleanup/message/to_robot", InteractiveCleanupMsg, timeout=None)
            self.loginfo(msg.message)
            if msg.message == ready_msg:
                self.logsuccess(f"Start task {ready_msg}")
                pub_msg = InteractiveCleanupMsg()
                pub_msg.message = "I_am_ready"
                self.pub_to_moderator.publish(pub_msg)
                break
        return

    def execute(self, userdata):
        self.loginfo("I'm preparing to start.")
        self.wait_for_ready_msg()
        return "next"


class Start(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        self.loginfo("Start")
        return "next"


class Finish(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        self.loginfo("Finished")
        return "finish"


class Except(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        self.logerr("Excepted")
        # return "except"
        return "recovery"
