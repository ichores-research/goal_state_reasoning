#!/usr/bin/env python3
import rospy
import enum
from ros_object_detections import *


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
        object_name = message
        response = get_object_pose(object_name)
    elif task == Task.GET_OBJECT_POSES.value:
        object_name = message
        response = get_object_poses()
    #TODO correct this, dont pass command
    elif task == Task.PLACE_OBJECT.value:
        object_name, where = message.split(";")
        scene_info = parse_scene_for_placing(message)
        scene_info["command"] = f"place {object_name} {where}"
        response = scene_info
        
    else:
        response = "Unknown task"

    return response

