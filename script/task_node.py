#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros
# from hsrlib.hsrif import HSRInterfaces

from state_machine import standard, guide, get_task, move


class InteractiveCleanupStateMachine():
    def __init__(self) -> None:
        """
        必要なモジュールを初期化
        """
        self.start_state = rospy.get_param(rospy.get_name() + "/start_state", "Start")

        # ステートマシンの宣言
        self.sm = smach.StateMachine(outcomes=["exit"])

        with self.sm:
            smach.StateMachine.add(
                "Init",
                standard.Init(["next", "except"]),
                transitions={"next": "Wait4Start", "except": "Except"},
            )
            smach.StateMachine.add(
                "Wait4Start",
                standard.Wait4Start(["next", "except"]),
                transitions={"next": self.start_state, "except": "Except"},
            )
            smach.StateMachine.add(
                "Start",
                standard.Start(["next", "except"]),
                transitions={"next": "RecognizeTargetPoint", "except": "Except"},
            )

            # Interactive cleanup
            smach.StateMachine.add(
                "RecognizeTargetPoint",
                get_task.GetTask(["move", "except"]),
                transitions={
                    "move": "Move2Pickup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Move2Pickup",
                guide.GuideHuman(["pickup", "loop", "except"]),
                transitions={
                    "pickup": "Pickup",
                    "loop": "Move2Pickup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Pickup",
                guide.GuideHuman(["move", "loop", "except"]),
                transitions={
                    "move": "Move2Cleanup",
                    "loop": "Pickup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Move2Cleanup",
                move.Move(["cleanup", "loop", "except"]),
                transitions={
                    "cleanup": "Cleanup",
                    "loop": "Move2Cleanup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Cleanup",
                guide.GuideHuman(["success", "loop", "except"]),
                transitions={
                    "success": "Finish",
                    "loop": "Cleanup",
                    "except": "Except",
                },
            )

            smach.StateMachine.add(
                "Finish",
                standard.Finish(["finish"]),
                transitions={"finish": "exit"},
            )
            smach.StateMachine.add(
                "Except",
                standard.Except(["except", "recovery"]),
                transitions={
                    "except": "exit",
                    "recovery": "Init"
                },
            )

    def delete(self) -> None:
        del self.sm

    def run(self) -> None:
        self.sm.execute()


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    cls = InteractiveCleanupStateMachine()
    rospy.on_shutdown(cls.delete)
    try:
        cls.run()
    except rospy.exceptions.ROSException as e:
        rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        rospy.logerr("[" + rospy.get_name() + "]: " + str(e))


if __name__ == "__main__":
    main()
