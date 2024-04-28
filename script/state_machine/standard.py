#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from tamlib.utils import Logger
from std_srvs.srv import SetBool, SetBoolRequest

from interactive_cleanup.msg import InteractiveCleanupMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from sigverse_hsrlib import HSRBMoveIt
from sigverse_hsrb_nav import HSRBNavigation


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
        self.inital_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        self.first_trial = True
        self.moveit = HSRBMoveIt()
        self.hsrb_nav = HSRBNavigation()

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

    def set_inital_pose(self):
        self.loginfo("set inital pose.")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 1
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        for _ in range(5):
            self.inital_pose.publish(msg)
            rospy.sleep(0.5)

    def execute(self, userdata):
        self.loginfo("I'm preparing to start.")
        self.moveit.delete()
        self.hsrb_nav.cancel_goal()

        # 指差し位置の初期化
        rospy.set_param("/interactive_cleanup/task/start", False)
        rospy.set_param("/interactive_cleanup/pickup/point", 0)
        rospy.set_param("/interactive_cleanup/cleanup/point", 0)

        if True:
            self.set_inital_pose()
        else:
            self.first_trial = False

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
        self.pub_to_moderator = rospy.Publisher("/interactive_cleanup/message/to_moderator", InteractiveCleanupMsg, queue_size=5)

    def execute(self, userdata):
        self.loginfo("Finished")
        msg = InteractiveCleanupMsg()
        msg.message = "Task_finished"
        msg.detail = "Task_finished"
        self.pub_to_moderator.publish(msg)

        msg = InteractiveCleanupMsg()
        msg.message = "Give_up"
        msg.detail = "Give_up"
        self.pub_to_moderator.publish(msg)

        return "finish"


class Except(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        self.logerr("Excepted")
        # return "except"
        return "recovery"
