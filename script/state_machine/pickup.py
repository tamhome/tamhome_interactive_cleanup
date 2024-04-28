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
from interactive_cleanup.msg import InteractiveCleanupMsg
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from tam_make_response.msg import MakeResponseAction, MakeResponseGoal, MakeResponseResult
from std_srvs.srv import SetBool, SetBoolRequest
# from tam_grasp.srv import GraspPoseEstimationService, GraspPoseEstimationServiceRequest
from tam_object_detection.srv import LangSamObjectDetectionService, LangSamObjectDetectionServiceRequest
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest
from tamhome_skills import Grasp


class Pickup(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(
            self, outcomes=outcomes,
            input_keys=[],
            output_keys=["status"]
        )
        Logger.__init__(self, loglevel="INFO")

        self.detection_type = rospy.get_param("~detection_type", "lang_sam")
        self.grasp_from = rospy.get_param("~grasp_from", "floor")

        self.moveit = HSRBMoveIt()
        self.tamtf = Transform()
        self.move_joints = MoveJoints()
        self.tam_grasp = Grasp()

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

        self.pub_to_moderator = rospy.Publisher("/interactive_cleanup/message/to_moderator", InteractiveCleanupMsg, queue_size=5)

        rospy.wait_for_service(detection_service_name, timeout=100)
        self.loginfo("connected to object detection service")
        # self.prompt = "blue_tumbler. ketchup. ground_pepper. salt. sauce. soysauce. sugar. canned_juice. plastic_bottle. cubic_clock. bear_doll. dog_doll. rabbit_doll. toy_car. toy_penguin. toy_duck. nursing_bottle. apple. banana. cigarette. hourglass. rubik_cube. spray_bottle. game_controller. piggy_bank. matryoshka"
        self.prompt = "object"

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
        offset = np.random.uniform(-0.1, 0.1)
        self.move_joints.move_head(0, -0.5+offset)
        rospy.sleep(2)

        # object detection
        if self.detection_type == "yolov8":
            det_req = ObjectDetectionServiceRequest(
                confidence_th=0.5,
                iou_th=0.5,
                max_distance=5.0,
                use_latest_image=True,
            )
            self.loginfo(det_req)
        else:
            det_req = LangSamObjectDetectionServiceRequest(
                use_latest_image=True,
                confidence_th=0.4,
                iou_th=0.4,
                prompt=self.prompt,
                max_distance=5.0,
            )
            self.loginfo(det_req)

        detections = self.srv_detection(det_req).detections
        self.logtrace(detections)
        if detections.is_detected is False:
            # userdata.grasp_failure = True
            prompt = "grasp, not_detected, give_me, " + target_object_name
            return self.grasp_failure(prompt=prompt)

        # 目標位置に最も近いオブジェクトを探索
        self.loginfo(detections)
        target_idx = self.calc_nearest_object(target_point, detections.pose)

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

        self.move_joints.gripper(3.14)
        rospy.sleep(1)

        use_moveit = False
        if use_moveit:
            # 把持前の姿勢に移動
            grasp_pose_base_pre = grasp_pose_base
            grasp_pose_base_pre.position.z = grasp_pose_base.position.z + 0.1
            # grasp_pose_base_pre.orientation = euler2quaternion(0, -1.57, np.pi)
            grasp_pose_base_pre.orientation = euler2quaternion(3.14, 0, 0)
            res = self.moveit.move_to_pose(grasp_pose_base_pre, self.base_frame)
            rospy.sleep(2)

            # 把持姿勢に遷移
            grasp_pose_base_second = grasp_pose_base_pre
            grasp_pose_base_second.position.z = grasp_pose_base_pre.position.z - 0.03
            res = self.moveit.move_to_pose(grasp_pose_base_second, self.base_frame)
            rospy.sleep(2)
        else:
            default_grasp_joint = [0.0, -2.30, 0.0, 0.73, 0.0]
            self.move_joints.move_arm_by_pose(
                default_grasp_joint[0],
                default_grasp_joint[1],
                default_grasp_joint[2],
                default_grasp_joint[3],
                default_grasp_joint[4],
            )
            rospy.sleep(2.5)
            self.tam_grasp._set_z_axis(pose_odom=grasp_pose_base, grasp_type="front", tolerance=0.03)
            self.tam_grasp._set_xy_axis(pose_odom=grasp_pose_base, grasp_type="front")

        # 把持
        self.move_joints.gripper(0)
        rospy.sleep(2)
        self.move_joints.move_arm_by_line(+0.03, "arm_lift_joint")
        rospy.sleep(2)
        self.move_joints.gripper(0)

        # 移動姿勢にする
        self.move_joints.go()
        rospy.sleep(7)

        # TODO: 把持チェック
        self.tam_grasp.grasp_check()

        msg = InteractiveCleanupMsg()
        msg.message = "Object_grasped"
        msg.detail = "Object_grasped"
        self.pub_to_moderator.publish(msg)

        try:
            msg = rospy.wait_for_message("/interactive_cleanup/message/to_robot", InteractiveCleanupMsg, timeout=1)
            if msg.message == "Task_failed":
                return "except"
            else:
                self.logsuccess("I can grasp correct object!")
        except Exception as e:
            self.logwarn(e)
            pass


        userdata.status = "clean_up"
        return "move"
