#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import smach
import smach_ros

from state_machine import standard
from state_machine import pickup, recog_target_point, recognition, move, cleanup


class InteractiveCleanupStateMachine():
    def __init__(self) -> None:
        """
        必要なモジュールを初期化
        """
        self.start_state = rospy.get_param("~start_state", "Start")

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
                recog_target_point.RecogTargetPoint(["move", "except"]),
                transitions={
                    "move": "Move2Pickup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Move2Pickup",
                move.Move(["next", "loop", "except"]),
                transitions={
                    "next": "Pickup",
                    "loop": "Move2Pickup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Pickup",
                pickup.Pickup(["move", "loop", "except"]),
                transitions={
                    "move": "Move2Cleanup",
                    "loop": "Pickup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Move2Cleanup",
                move.Move(["next", "loop", "except"]),
                transitions={
                    "next": "Cleanup",
                    "loop": "Move2Cleanup",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Cleanup",
                cleanup.CleanUp(["success", "loop", "except"]),
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
