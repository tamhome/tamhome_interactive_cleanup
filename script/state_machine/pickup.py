#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import random
import tf2_ros
import actionlib
import numpy as np
import tf.transformations
from tamlib.utils import Logger
# from googletrans import Translator
from tamlib.cv_bridge import CvBridge
from sigverse_hsrlib import HSRBMoveIt, MoveJoints
from image_geometry import PinholeCameraModel
from tamlib.tf import Transform, quaternion2euler, transform_pose, euler2quaternion

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from tam_make_response.msg import MakeResponseAction, MakeResponseGoal, MakeResponseResult

from std_srvs.srv import SetBool, SetBoolRequest
# from tam_grasp.srv import GraspPoseEstimationService, GraspPoseEstimationServiceRequest
from tam_object_detection.srv import LangSamObjectDetectionService, LangSamObjectDetectionServiceRequest
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest


class Pickup(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="DEBUG")

        self.detection_type = rospy.get_param("~detection_type", "yolov8")
        self.grasp_from = rospy.get_param("~grasp_from", "floor")

        self.moveit = HSRBMoveIt()
        self.tamtf = Transform()
        self.move_joints = MoveJoints()

        self.head_rgbd_frame = "head_rgbd_sensor_link"
        self.base_frame = "odom"
        self.odom_frame = "odom"

        self.loginfo("wait for object detection service start")
        detection_service_name = "sigverse/hsr_head_rgbd/object_detection/service"
        self.srv_detection = rospy.ServiceProxy(detection_service_name, ObjectDetectionService)
        rospy.wait_for_service(detection_service_name, timeout=10)
        self.loginfo("connected to object detection service")

        self.loop_counter = 0

    def is_hand_collision(self, th=2.0) -> bool:
        """HSRのハンドが物体に衝突しているかどうかを判定する関数（実装途中）
        Args:
            th(float): 衝突判定のしきい値
        """
        return False

    def grasp_failure(self, prompt="grasp, not_detected, give_me, white cup"):
        self.logwarn("detection failure")
        return "loop"

    def contains_japanese(self, text):
        try:
            # 文字列をUTF-8でバイト列にエンコード
            text.encode('utf-8').decode('ascii')
        except UnicodeDecodeError:
            # ASCIIデコードが失敗した場合（＝全角文字が含まれている場合）
            return True
        else:
            return False

    # def translate_to_english_if_japanese(self, text):
    #     if self.contains_japanese(text):
    #         translator = Translator()
    #         translation = translator.translate(text, src="ja", dest='en')
    #         return translation.text
    #     else:
    #         return text

    def broadcast_tf(self, odom_frame, target_frame, position, orientation):
        broadcaster = tf2_ros.TransformBroadcaster()
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = odom_frame
        transform.child_frame_id = target_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]

        # 発行するトランスフォームを送信
        self.logdebug(transform)
        broadcaster.sendTransform(transform)

    def execute(self, userdata):
        """物体検出と把持をする関数
        """
        # 現在のゴールを削除
        self.moveit.delete()
        target_object_name = "toy_penguin"

        # 首を下に向ける
        self.move_joints.neutral()
        rospy.sleep(4)
        self.move_joints.move_head(0, -0.4)
        rospy.sleep(3)

        # object detection
        if self.detection_type == "yolov8":
            det_req = ObjectDetectionServiceRequest(
                confidence_th=0.5,
                iou_th=0.5,
                max_distance=5.0,
                use_latest_image=True,
            )
            self.logdebug(det_req)

            detections = self.srv_detection(det_req).detections
            self.logdebug(detections)
            if detections.is_detected is False:
                # userdata.grasp_failure = True
                prompt = "grasp, not_detected, give_me, " + target_object_name
                return self.grasp_failure(prompt=prompt)

            max_score_idx = None
            max_score = -1
            for i, box in enumerate(detections.bbox):
                if box.name != target_object_name:
                    continue
                score = box.score
                if score > max_score:
                    max_score = score
                    max_score_idx = i
            if max_score_idx is None:
                self.loop_counter += 1
                if self.loop_counter > 30:
                    self.loop_counter = 0
                    prompt = "grasp, not_found, give_me, " + target_object_name
                    return self.grasp_failure(prompt)
                else:
                    return "loop"

            self.loginfo(detections.bbox[max_score_idx].name)
            # grasp pose estimation
            open_angle = 1.2
            grasp_pose = Pose(
                detections.pose[max_score_idx].position,
                Quaternion(0, 0, 0, 1),
            )

            grasp_pose_base: Pose = self.tamtf.get_pose_with_offset(
                target_frame=self.base_frame,
                source_frame=self.head_rgbd_frame,
                offset=grasp_pose,
            )

            self.loginfo(grasp_pose_base)

            # self.loginfo(detections.pose[max_score_idx].frame_id)

            # TODO: 把持点推定を導入
            # gpe_req = GraspPoseEstimationServiceRequest(
            #     depth=detections.depth,
            #     mask=detections.segments[max_score_idx],
            #     pose=pose,
            #     camera_frame_id=detections.pose[max_score_idx].frame_id,
            #     frame_id=self.description.frame.base,
            # )
            # gpe_res = self.srv_grasp(gpe_req)
            # if gpe_res.success is False:
            #     userdata.grasp_failure = True
            #     prompt = "grasp, calculate_path, give_me, " + target_object_name
            #     return self.grasp_failure(prompt)


        # # Lang samによる把持
        # else:
        #     goal_pose = joints.get_neutral_pose()
        #     tilt_degree = random.randint(-40, -20)
        #     pan_degree = random.randint(-5, 5)
        #     goal_pose["arm_lift_joint"] = 0.1
        #     goal_pose["head_tilt_joint"] = np.deg2rad(tilt_degree)
        #     goal_pose["head_pan_joint"] = np.deg2rad(pan_degree)
        #     self.hsrif.whole_body.move_to_joint_positions(goal_pose, sync=True)

        #     det_req = LangSamObjectDetectionServiceRequest(use_latest_image=True, confidence_th=0.4, iou_th=0.4, prompt=target_object_name)
        #     detections = self.srv_detection(det_req).detections
        #     print(detections)

        #     max_score_idx = None
        #     score_th = 0.37
        #     max_score = -1
        #     for i, box in enumerate(detections.bbox):
        #         # if box.name != target_object_name:
        #         #     self.logdebug("wrong target.")
        #         #     continue
        #         # else:
        #         #     self.logdebug("target object.")

        #         score = box.score
        #         if score < score_th:
        #             continue
        #         if score > max_score:
        #             max_score = score
        #             max_score_idx = i
        #             self.logdebug("max score 更新")

        #     self.loginfo("max_score_idx is {}".format(max_score_idx))
        #     if max_score_idx is None:
        #         self.loop_counter += 1
        #         if self.loop_counter > 30:
        #             self.loop_counter = 0
        #             prompt = "grasp, not_found, give_me, " + target_object_name
        #             return self.grasp_failure(prompt)
        #         else:
        #             return "loop"
        #     # このposeはhead_camera座標系
        #     pose = Pose(
        #         detections.pose[max_score_idx].position,
        #         detections.pose[max_score_idx].orientation
        #     )
        #     # base座標に変換
        #     grasp_pose = self.tamtf.get_pose_with_offset(
        #         self.description.frame.base,
        #         self.camera_frame_id,
        #         offset=pose,
        #     )
        #     self.loginfo("把持目標地点")
        #     print(grasp_pose)

        #     # 遠すぎる場合は掴みにいかない
        #     if grasp_pose.position.x > 1.2:
        #         prompt = "grasp, far, give_me, " + target_object_name
        #         return self.grasp_failure(prompt)

        # self.grasp_failure_count = 0

        # Reach to object
        # 角度を正面からの把持姿勢に設定
        # grasp_pose_pre = grasp_pose
        # offset = 0.2
        # grasp_pose_pre.position.x -= offset
        # grasp_pose_pre.position.z += 0.07

        # 物体の中心位置にTFを発行し，その位置に移動
        self.move_joints.gripper(3.14)

        # 把持前の姿勢に移動
        grasp_pose_base_pre = grasp_pose_base
        grasp_pose_base_pre.position.z = grasp_pose_base.position.z + 0.1
        grasp_pose_base_pre.orientation = euler2quaternion(0, -1.57, np.pi)
        res = self.moveit.move_to_pose(grasp_pose_base_pre, self.base_frame)
        rospy.sleep(2)

        # 把持姿勢に遷移
        grasp_pose_base_second = grasp_pose_base_pre
        grasp_pose_base_second.position.z = grasp_pose_base_pre.position.z - 0.1
        res = self.moveit.move_to_pose(grasp_pose_base_second, self.base_frame)
        rospy.sleep(4)

        # 把持
        self.move_joints.gripper(0)
        rospy.sleep(4)
        self.move_joints.move_arm_by_line(+0.03, "arm_lift_joint")
        rospy.sleep(4)
        self.move_joints.gripper(0)

        # res = self.hsrif.whole_body.move_end_effector_pose(
        #     grasp_pose_pre,
        #     self.description.frame.base,
        # )
        # if res is False:
        #     prompt = "grasp, cannot_reach_object, give_me, " + target_object_name
        #     return self.grasp_failure(prompt)

        # # impedance
        # self.hsrif.whole_body.impedance_config = "grasping"
        # self.hsrif.whole_body.move_end_effector_by_line(
        #     (0, 0, 1), offset, sync=False
        # )
        # is_collision = False
        # retry_counter = 0
        # while self.hsrif.whole_body.is_moving():
        #     if self.is_hand_collision(2.0):
        #         is_collision = True
        #         self.loginfo("Collision DETECTED")
        #         self.hsrif.whole_body.cancel_goal()

        #         # 少し下がる
        #         self.hsrif.whole_body.move_end_effector_by_line(
        #             (0, 0, -1), 0.2, sync=True
        #         )
        #         rospy.sleep(1)

        #         if retry_counter < 1:
        #             say_message = {
        #                 "ja": "失敗したのだ．再挑戦するのだ",
        #                 "en": "collision detected. I'll try again."
        #             }
        #             self.hsrif.tts.say_queue(say_message[self.language], self.language)
        #             self.loginfo("把持再挑戦")
        #             # ハンドを上に上げる
        #             grasp_pose_pre.position.z += 0.05
        #             res = self.hsrif.whole_body.move_end_effector_pose(
        #                 grasp_pose_pre,
        #                 self.description.frame.base,
        #             )
        #             rospy.sleep(0.5)

        #             self.hsrif.whole_body.move_end_effector_by_line(
        #                 (0, 0, 1), offset, sync=False
        #             )
        #             retry_counter += 1
        #         else:
        #             prompt = "grasp, collision, give_me, " + target_object_name
        #             return self.grasp_failure(prompt=prompt)

        # self.hsrif.gripper.apply_force(0.6, delicate=is_delicate)

        # 移動姿勢にする
        self.move_joints.go()

        # ハンドを上に上げる
        # self.hsrif.whole_body.move_end_effector_by_line(
        #     (1, 0, 0), 0.1, sync=True
        # )

        # # 少し下がる
        # self.hsrif.whole_body.move_end_effector_by_line(
        #     (0, 0, -1), offset+0.1, sync=True
        # )

        # TODO: 把持チェック

        rospy.sleep(0.5)

        return "move"
