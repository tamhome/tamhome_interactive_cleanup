#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, TriggerResponse

# ステートの定義
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_proc1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START')
        return 'to_proc1'

class Proc1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_proc2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PROC1')
        rospy.sleep(30)
        return 'to_proc2'

class Proc2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PROC2')
        return 'to_finish'

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH')
        return 'done'

# サービスのコールバック関数
def handle_state_change(req):
    global sm
    rospy.loginfo("Received request to change state to START")
    if sm.is_running():
        sm.request_preempt()
    sm.set_initial_state(['Start'])
    rospy.sleep(1)
    sm.execute()
    return TriggerResponse(success=True, message="State machine reset to START")

# ステートマシンとROSノードの初期化
def main():
    rospy.init_node('state_machine_reset')
    global sm
    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('Start', Start(), transitions={'to_proc1': 'Proc1'})
        smach.StateMachine.add('Proc1', Proc1(), transitions={'to_proc2': 'Proc2'})
        smach.StateMachine.add('Proc2', Proc2(), transitions={'to_finish': 'Finish'})
        smach.StateMachine.add('Finish', Finish(), transitions={'done': 'Start'})  # Loop back to start for simplicity

    # サービスの設定
    s = rospy.Service('/compulsory_state_change/state', Trigger, handle_state_change)

    # ステートマシンの実行
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()