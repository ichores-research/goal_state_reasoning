#!/usr/bin/env python3
import rospy
import enum
from object_detection import *
from ycb_objects import get_ycb_objects_info
from pick_and_place import pick_object


DATASET = os.environ.get("DATASET", "ycb_ichores")
OBJECT_INFO = get_ycb_objects_info(DATASET)



class Task(enum.Enum):
    GET_OBJECT_NAMES = "get_object_names"
    GET_OBJECT_POSE = "get_object_pose"
    GET_OBJECT_POSES = "get_object_poses"
    GET_POINTING_SEQUENCE = "get_pointing_sequence"
    PICK_OBJECT = "pick_object"
    PLACE_OBJECT = "place_object"
    RELEASE_PICKED_OBJECT = "release_picked_object"



def robot_execute(task, message=None):

    if task == Task.GET_OBJECT_NAMES.value: 
        detections = detect_objects()
        response = [detection.name for detection in detections]

    elif task == Task.GET_OBJECT_POSE.value:
        response = None
        object_name = message
        for i in range(3):
            response = get_object_pose(object_name)
            if response is not None:
                break
        if response is None:
            response = "Object pose could not be estimated."

    elif task == Task.GET_OBJECT_POSES.value:
        object_name = message
        response = get_object_poses()

    elif task == Task.PLACE_OBJECT.value:
        coords = tuple(message)
        #TODO: Implement robot arm movement
        response = "success"

    elif task == Task.PICK_OBJECT.value:
        object_name = message
        object_info = OBJECT_INFO.get(object_name, None)
        if object_info is None:
            return f"Object {object_name} not found in dataset."
        
        pick_success = pick_object(object_info)
        
        response = "success" if pick_success else f"Failed to pick {object_name}."
    else:
        response = "Unknown task"

    return response

