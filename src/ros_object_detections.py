#!/usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server
from sensor_msgs.msg import Image
import json


def detect_objects():
    rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
    rospy.wait_for_service('detect_objects')
    try:
        detect_objects_service = rospy.ServiceProxy('detect_objects', detectron2_service_server)
        response = detect_objects_service(rgb)
        return response.detections.detections
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None

def serialize_detections(detections):
    """Serialize detections to a JSON-compatible format."""
    if detections is None:
        return json.dumps([]) #Return empty list if no detections

    serialized_detections = []
    for detection in detections:
        serialized_detection = {
            "name": detection #detection.name
            
            # "score": detection.score,
            # "bbox": {
            #     "xmin": detection.bbox.xmin,
            #     "ymin": detection.bbox.ymin,
            #     "xmax": detection.bbox.xmax,
            #     "ymax": detection.bbox.ymax,
            #},
            # Add other relevant fields as needed
        }
        serialized_detections.append(serialized_detection)
    return json.dumps(serialized_detections)

