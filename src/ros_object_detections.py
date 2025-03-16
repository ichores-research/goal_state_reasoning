#!/usr/bin/env python3
import os
import rospy
from object_detector_msgs.srv import detectron2_service_server,  estimate_poses
from sensor_msgs.msg import Image
from ycb_objects import *


DATASET = os.environ.get("DATASET", "ycb_ichores")
OBJECT_INFO = get_ycb_objects_info(DATASET)


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

def get_object_pose(object_name):
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

def get_object_poses():
    object_poses = []
    detections = detect_objects()
    for detection in detections:
        #TODO cretae dict
        #TypeError: 'PoseWithConfidence' object is not subscriptable
        object_poses.append(get_object_pose(detection.name)) 
    return object_poses

def parse_scene_for_placing(object_in_the_gripper=None):
    scene = {}
    obj_poses = get_object_poses()
    print(obj_poses)
    scene['objects on the table'] = []
    for obj in obj_poses:
        scene['objects on the table'].append(
        { 
            'name': obj["name"],
            'position': [round(obj["pose"]["position"]["x"] , 2), 
                         round(obj["pose"]["position"]["y"] , 2), 
                         round(obj["pose"]["position"]["z"] , 2)
                        ],
            'diameter': OBJECT_INFO[obj["name"]]['diameter']
        })
        scene['table limitations'] = TABLE_HEURISTIC
        if object_in_the_gripper is not None:
            scene['object in the gripper'] = {'name': object_in_the_gripper, 'diameter': OBJECT_INFO[object_in_the_gripper]['diameter']}
    return scene




if __name__ == "__main__":
    rospy.init_node('object_detection_node')
    rate = rospy.Rate(200)
    try:
        while not rospy.is_shutdown():
            detections = detect_objects()
            for detection in detections:
                print(get_object_pose(detection.name))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass