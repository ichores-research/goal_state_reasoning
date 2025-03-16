#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server, estimate_pointing_gesture, estimate_poses
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult, GenericImgProcAnnotatorFeedback, GenericImgProcAnnotatorGoal
import actionlib
from sensor_msgs.msg import Image, RegionOfInterest

import tf
import tf.transformations as tf_trans
import numpy as np
import open3d as o3d
import yaml
import os
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion

from utils import *

class PoseCalculator:
    def __init__(self):
        self.image_publisher = rospy.Publisher('/pose_estimator/image_with_roi', Image, queue_size=10)
        self.bridge = CvBridge()

        self.models = load_models("/root/task/datasets/ycb_ichores/models", "/root/config/ycb_ichores.yaml")
        self.grasp_annotations = load_grasp_annotations("/root/task/datasets/ycb_ichores/grasp_annotations", "/root/config/ycb_ichores.yaml")

        self.color_frame_id = rospy.get_param('/pose_estimator/color_frame_id')
        self.grasp_frame_id = rospy.get_param('/pose_estimator/grasp_frame_id')

        self.marker_id = 0

    def detect_objects(self, rgb):
        rospy.wait_for_service('detect_objects')
        try:
            detect_objects_service = rospy.ServiceProxy('detect_objects', detectron2_service_server)
            response = detect_objects_service(rgb)
            return response.detections.detections
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def estimate_object_poses(self, rgb, depth, detection):
        rospy.wait_for_service('estimate_poses')
        try:
            estimate_poses_service = rospy.ServiceProxy('estimate_poses', estimate_poses)
            response = estimate_poses_service(detection, rgb, depth)
            return response.poses
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def detect_pointing_gesture(self, rgb, depth):
        rospy.wait_for_service('detect_pointing_gesture')
        try:
            detect_pointing_gesture_service = rospy.ServiceProxy('detect_pointing_gesture', estimate_pointing_gesture)
            response = detect_pointing_gesture_service(rgb, depth)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def load_grasps(self, obj_names):
        grasps = []
        for idx, obj_name in enumerate(obj_names):
            grasps_obj_frame = self.grasp_annotations.get(obj_name, None)['grasps']
            grasps.append(grasps_obj_frame)

        return grasps

    def transform_grasps(self, grasps_obj_frame, obj_poses):
        grasps = []
        for idx, (grasp_obj_frame, obj_pose) in enumerate(zip(grasps_obj_frame, obj_poses)):
            grasps_world_frame = transform_grasp_obj2world(grasp_obj_frame, obj_pose.pose)
            grasps.append(grasps_world_frame)

        return grasps
    
    def publish_annotated_image(self, rgb, detections):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        for detection in detections:
            xmin = int(detection.bbox.ymin)
            ymin = int(detection.bbox.xmin)
            xmax = int(detection.bbox.ymax)
            ymax = int(detection.bbox.xmax)

            font_size = 1.0
            line_size = 3

            cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), line_size)

            class_name = detection.name
            score = detection.score
            label = f"{class_name}: {score:.2f}"
            cv2.putText(cv_image, label, (xmin, ymin - 20), cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 255, 0), line_size)

        # Publish annotated image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_publisher.publish(annotated_image_msg)

        # Display image for debugging
        # cv2.imshow("Annotated Image", cv_image)
        # cv2.waitKey(10)

    def publish_mesh_marker(self, cls_name, quat, t_est):
        vis_pub = rospy.Publisher("/gdrnet_meshes_estimated", Marker, latch=True)
        model_data = self.models.get(cls_name, None)
        model_vertices = np.array(model_data['vertices'])/1000

        marker = Marker()
        marker.header.frame_id = self.grasp_frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TRIANGLE_LIST
        marker.ns = cls_name
        marker.action = Marker.ADD
        marker.pose.position.x = t_est[0]
        marker.pose.position.y = t_est[1]
        marker.pose.position.z = t_est[2]

        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA

        shape_vertices = 3*int((model_vertices.shape[0] - 1)/3)
        for i in range(shape_vertices):
            pt = Point(x = model_vertices[i, 0], y = model_vertices[i, 1], z = model_vertices[i, 2])
            marker.points.append(pt)
            rgb = ColorRGBA(r = 1, g = 0, b = 0, a = 1.0)
            marker.colors.append(rgb)

        vis_pub.publish(marker)

    def publish_grasp_marker(self, transformed_grasps):
        marker_pub = rospy.Publisher("/gdrnet_grasps_vis", MarkerArray, latch=True)

        marker_array = MarkerArray()
        align_x_to_z = tf_trans.quaternion_from_euler(0, np.pi / 2, 0)

        for idx, grasp_matrix in enumerate(transformed_grasps):
            grasp_matrix = np.array(grasp_matrix).reshape(4, 4)

            position = grasp_matrix[:3, 3]
            orientation_quat = tf_trans.quaternion_from_matrix(grasp_matrix)
            adjusted_orientation = tf_trans.quaternion_multiply(orientation_quat, align_x_to_z)

            marker = Marker()
            marker.header.frame_id = self.color_frame_id        # this is only for visualization!
            marker.header.stamp = rospy.Time.now()
            marker.ns = "grasp_arrows"
            marker.id = idx

            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.x = adjusted_orientation[0]
            marker.pose.orientation.y = adjusted_orientation[1]
            marker.pose.orientation.z = adjusted_orientation[2]
            marker.pose.orientation.w = adjusted_orientation[3]

            marker.scale.x = 0.1
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        marker_pub.publish(marker_array)


# main of example script for iChores Pipeline
# if you want to build your own rosnode, build it like this

if __name__ == "__main__":
    rospy.init_node("calculate_poses")
    try:
        pose_calculator = PoseCalculator()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # get RGB and Depth messages from the topics
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_ichores_pipeline/src/task/test_obj_det_dev.pytopic'), Image)
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            # ###############################
            # DETECTION EXAMPLE
            # ###############################

            t0 = time.time()
            detections = pose_calculator.detect_objects(rgb)
            time_detections = time.time() - t0

            if detections is not None:
                pose_calculator.publish_annotated_image(rgb, detections)
                for detection in detections:
                    print(detection.name)

            print()
            print("... received object detection.")

            t0 = time.time()
            if detections is None or len(detections) == 0:
                print("nothing detected")
            else:

                # ###############################
                # POSE ESTIMATION EXAMPLE
                # ###############################

                estimated_poses_camFrame = []
                object_names = []

                try:
                    for detection in detections:
                        if not detection.name == "036_wood_block":
                            estimated_pose = pose_calculator.estimate_object_poses(rgb, depth, detection)[0]
                            estimated_poses_camFrame.append(estimated_pose)
                            object_names.append(detection.name)
                     
                except Exception as e:
                    rospy.logerr(f"{e}")

                time_object_poses = time.time() - t0

                # for grasping it is necessary to transform estimated_poses_camFrame to the base_link (or whatever is equivalent on Tiago)

                # ###############################
                # GRASP LOADING EXAMPLE
                # ###############################
                #
                # here it is essential in which frame the estimated_poses are
                # we are using the camera frame

                grasp_tfs = None
                if len(estimated_poses_camFrame) > 0:
                    grasp_tfs = pose_calculator.load_grasps(object_names)   

                # transform grasps to camera frame (where object poses are)
                estimated_grasps_camFrame = None
                if len(estimated_poses_camFrame) > 0:
                    estimated_grasps_camFrame = pose_calculator.transform_grasps(grasp_tfs, estimated_poses_camFrame) 

                # ###############################
                # BEWARE! the publish_grasp_marker function is only for visualization!
                # thats why we use the grasps in the camera frame
 
                if estimated_grasps_camFrame is not None:
                    for grasp in estimated_grasps_camFrame:
                        pose_calculator.publish_grasp_marker(grasp)

                # ###############################
                # POINTING GESTURE DETECTION EXAMPLE
                # ###############################

                print('Perform Pointing Detection...')
                t0 = time.time()
                joint_positions = pose_calculator.detect_pointing_gesture(rgb, depth)
                time_pointing = time.time() - t0
                print('... received pointing gesture.')

                # New step: Check which object the human is pointing to
                t0 = time.time()
                #if len(estimated_poses_camFrame) > 0 and joint_positions is not None:
                if len(estimated_poses_camFrame) > 0 and joint_positions is not None:
                    elbow = joint_positions.elbow
                    wrist = joint_positions.wrist
                    min_distance = float('inf')
                    pointed_object = None
                    threshold = 0.3  # 0.5 meters              
                    
                    print(estimated_poses_camFrame)

                    pointed_object_pose = None
                    for idx, estimated_pose in enumerate(estimated_poses_camFrame):
                        object_position = estimated_pose.pose.position
                        distance = calculate_distance_to_line(object_position, elbow, wrist)
                        if distance < min_distance:
                            min_distance = distance
                            pointed_object = estimated_pose.name
                            pointed_object_pose = estimated_pose.pose

                    if min_distance < threshold:
                        R = np.array([pointed_object_pose.orientation.x, pointed_object_pose.orientation.y,  pointed_object_pose.orientation.z, pointed_object_pose.orientation.w])
                        t = np.array([pointed_object_pose.position.x, pointed_object_pose.position.y, pointed_object_pose.position.z])
                        pose_calculator.publish_mesh_marker(pointed_object, R, t)
                        print(f"The human is pointing to the object: {pointed_object}")
                        print()

                time_point_checker = time.time() - t0
                # Print the timed periods
                print(f"Time for object detection: {time_detections:.2f} seconds")
                print(f"Time for pointing detection: {time_pointing:.2f} seconds")
                print(f"Time for object pose estimation: {time_object_poses:.2f} seconds")
                print(f"Time for pointing checker: {time_point_checker:.2f} seconds")
                print()
                rate.sleep()

    except rospy.ROSInterruptException:
        pass

