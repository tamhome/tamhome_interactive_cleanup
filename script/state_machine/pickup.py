#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import smach
import random
import tf2_ros
import actionlib
import numpy as np
from typing import List, Any
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
        smach.State.__init__(
            self, outcomes=outcomes,
            input_keys=[],
            output_keys=["status"]
        )
        Logger.__init__(self, loglevel="DEBUG")

        self.detection_type = rospy.get_param("~detection_type", "lang_sam")
        self.grasp_from = rospy.get_param("~grasp_from", "floor")

        self.moveit = HSRBMoveIt()
        self.tamtf = Transform()
        self.move_joints = MoveJoints()

        self.head_rgbd_frame = "head_rgbd_sensor_link"
        self.base_frame = "odom"
        self.odom_frame = "odom"

        self.loginfo("wait for object detection service start")
        if self.detection_type == "yolov8":
            detection_service_name = "sigverse/hsr_head_rgbd/object_detection/service"
            self.srv_detection = rospy.ServiceProxy(detection_service_name, ObjectDetectionService)
        else:
            detection_service_name = "/hsr_head_rgbd/lang_sam/object_detection/service"
            self.srv_detection = rospy.ServiceProxy(detection_service_name, LangSamObjectDetectionService)

        rospy.wait_for_service(detection_service_name, timeout=10)
        self.loginfo("connected to object detection service")
        # self.prompt = "blue_tumbler. ketchup. ground_pepper. salt. sauce. soysauce. sugar. canned_juice. plastic_bottle. cubic_clock. bear_doll. dog_doll. rabbit_doll. toy_car. toy_penguin. toy_duck. nursing_bottle. apple. banana. cigarette. hourglass. rubik_cube. spray_bottle. game_controller. piggy_bank. matryoshka"
        self.prompt = "small object"

        self.loop_counter = 0

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """
        Calculate the Euclidean distance between two Pose objects.

        Parameters:
        pose1 (Pose): The first pose.
        pose2 (Pose): The second pose.

        Returns:
        float: The Euclidean distance between pose1 and pose2.
        """
        x1, y1, z1 = pose1.position.x, pose1.position.y, pose1.position.z
        x2, y2, z2 = pose2.position.x, pose2.position.y, pose2.position.z

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def calc_nearest_object(self, target_point: List, detected_objects: Any) -> int:
        """指差しの座標と最も近いオブジェクトを検出
        """
        target_pose = Pose()
        target_pose.position.x = target_point[0]
        target_pose.position.y = target_point[1]
        target_pose.position.z = target_point[2]
        target_pose.orientation = euler2quaternion(0, 0, 0)

        min_distance = np.inf
        target_idx = 0

        for idx, detected_object in enumerate(detected_objects):
            detected_object_pose_map: Pose = self.tamtf.get_pose_with_offset(
                target_frame="map",
                source_frame=self.head_rgbd_frame,
                offset=detected_object,
            )

            distance = self.calculate_distance(target_pose, detected_object_pose_map)
            if distance < min_distance:
                min_distance = distance
                target_idx = idx

        return target_idx

    def is_hand_collision(self, th=2.0) -> bool:
        """HSRのハンドが物体に衝突しているかどうかを判定する関数（実装途中）
        Args:
            th(float): 衝突判定のしきい値
        """
        return False

    def grasp_failure(self, prompt="grasp, not_detected, give_me, white cup"):
        self.logwarn("detection failure")
        return "loop"

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
        target_point = rospy.get_param("/interactive_cleanup/pickup/point", 0)
        if target_point == 0:
            self.grasp_failure("plz repointing")
            return "loop"

        rospy.sleep(1)
        target_object_name = "rabbit_doll"

        # 首を下に向ける
        self.move_joints.go()
        rospy.sleep(2)
        self.move_joints.move_head(0, -0.4)
        rospy.sleep(2)

        # object detection
        if self.detection_type == "yolov8":
            det_req = ObjectDetectionServiceRequest(
                confidence_th=0.5,
                iou_th=0.5,
                max_distance=5.0,
                use_latest_image=True,
            )
            self.logdebug(det_req)
        else:
            det_req = LangSamObjectDetectionServiceRequest(use_latest_image=True, confidence_th=0.4, iou_th=0.4, prompt=self.prompt)

        detections = self.srv_detection(det_req).detections
        self.logdebug(detections)
        if detections.is_detected is False:
            # userdata.grasp_failure = True
            prompt = "grasp, not_detected, give_me, " + target_object_name
            return self.grasp_failure(prompt=prompt)

        # 目標位置に最も近いオブジェクトを探索
        target_idx = self.calc_nearest_object(target_point, detections.pose)

        # max_score_idx = None
        # max_score = -1
        # for i, box in enumerate(detections.bbox):
        #     if box.name != target_object_name:
        #         continue
        #     score = box.score
        #     if score > max_score:
        #         max_score = score
        #         max_score_idx = i
        # if max_score_idx is None:
        #     self.loop_counter += 1
        #     if self.loop_counter > 30:
        #         self.loop_counter = 0
        #         prompt = "grasp, not_found, give_me, " + target_object_name
        #         return self.grasp_failure(prompt)
        #     else:
        #         return "loop"

        self.loginfo(detections.bbox[target_idx].name)
        # grasp pose estimation
        open_angle = 1.2
        grasp_pose = Pose(
            detections.pose[target_idx].position,
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
        rospy.sleep(2)

        # 把持
        self.move_joints.gripper(0)
        rospy.sleep(2)
        self.move_joints.move_arm_by_line(+0.03, "arm_lift_joint")
        rospy.sleep(2)
        self.move_joints.gripper(0)

        # 移動姿勢にする
        self.move_joints.go()

        # TODO: 把持チェック

        userdata.status = "clean_up"
        return "move"
