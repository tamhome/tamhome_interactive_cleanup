#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import smach
import smach_ros
from tamlib.utils import Logger
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from interactive_cleanup.msg import InteractiveCleanupMsg


class TimeServer(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")

        self.srv_change_init_state = rospy.ServiceProxy("/set/init_state", Trigger)
        self.sub_moderator = rospy.Subscriber("/interactive_cleanup/message/to_robot", InteractiveCleanupMsg, callback=self.cb_timeup)

    def cb_timeup(self, handyman_msg: InteractiveCleanupMsg) -> None:
        self.loginfo(handyman_msg)
        message = handyman_msg.message
        detail = handyman_msg.detail

        if message == "Task_failed":
            if detail == "Time_is_up":
                self.loginfo("send restart message")
                req = TriggerRequest()
                self.srv_change_init_state(req)


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])
    p_loop_rate = rospy.get_param("~loop_rate", 30)
    loop_wait = rospy.Rate(p_loop_rate)
    cls = TimeServer()

    while not rospy.is_shutdown():
        try:
            loop_wait.sleep()
        except rospy.exceptions.ROSException as e:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
            rospy.logerr("[" + rospy.get_name() + "]: " + str(e))
