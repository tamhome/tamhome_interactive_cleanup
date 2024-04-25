#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import smach
import smach_ros
from tamlib.utils import Logger
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from handyman_msgs.msg import HandymanMsg


class TimeServer(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")

        self.srv_change_init_state = rospy.ServiceProxy("/set/init_state", Trigger)
        self.sub_moderator = rospy.Subscriber("/handyman/message/to_robot", HandymanMsg, callback=self.cb_timeup)

    def cb_timeup(self, handyman_msg: HandymanMsg) -> None:
        message = handyman_msg.message
        if message == "Time_up":
            req = TriggerRequest()
            self.srv_change_init_state(req)


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])
    loop_rate = rospy.get_param("~loop_rate", 30)
    rate = rospy.loop_rate(loop_rate)
    cls = TimeServer()

    while not rospy.is_shutdown():
        try:
            rospy.sleep(rate)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
            rospy.logerr("[" + rospy.get_name() + "]: " + str(e))
