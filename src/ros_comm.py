#!/usr/bin/env python3
import time
import rospy
import enum
from object_detector_msgs.srv import detectron2_service_server,  estimate_poses
from sensor_msgs.msg import Image
import json

class Task(enum.Enum):
    GET_OBJECT_NAMES = "get_object_names"
    GET_OBJECT_POSE = "get_object_pose"
    GET_POINTING_SEQUENCE = "get_pointing_sequence"
    PICK_OBJECT = "pick_object"
    PLACE_OBJECT = "place_object"
    RELEASE_PICKED_OBJECT = "release_picked_object"


def detect_objects(rgb=None):
    if rgb is None:
        rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
    rospy.wait_for_service('detect_objects')
    try:
        detect_objects_service = rospy.ServiceProxy('detect_objects', detectron2_service_server)
        response = detect_objects_service(rgb)
        return response.detections.detections
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None


def estimate_object_pose(rgb, depth, detection):
    
    rospy.wait_for_service('estimate_poses')
    
    try:
        estimate_poses_service = rospy.ServiceProxy('estimate_poses', estimate_poses)
        response = estimate_poses_service(detection, rgb, depth)
        return response.poses
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def object_pose_estimation(object_name):
    rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
    depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)
    detections = detect_objects(rgb)

    if detections is None or len(detections) == 0:
        return "Nothing detected"
    else:

        estimated_pose_camFrame = None

        try:
            for detection in detections:
                if detection.name == object_name:
                    estimated_pose_camFrame = estimate_object_pose(rgb, depth, detection)[0]
                    break
                
        except Exception as e:
            rospy.logerr(f"{e}")
            return "Pose estimation failed"
        return estimated_pose_camFrame


def robot_execute(task, message=None):
    if task == Task.GET_OBJECT_NAMES.value: 
        detections = detect_objects()
        response = [detection.name for detection in detections]
    elif task == Task.GET_OBJECT_POSE.value:
        object_name = message
        response = object_pose_estimation(object_name)
         
    else:
        response = "Unknown task"

    return response

if __name__ == "__main__":
    rospy.init_node('object_detection_node')
    #time.sleep(10)
    rate = rospy.Rate(200)
    try:
        while not rospy.is_shutdown():
            detections = detect_objects()
            for detection in detections:
                print(object_pose_estimation(detection.name))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass