#!/usr/bin/env python3
import rospy
import enum
from ros_object_detections import *
from pick_and_place_prague import PickAndPlaceCommandSender

pick_and_place_cmd_sender = PickAndPlaceCommandSender()


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
    elif task == Task.PLACE_OBJECT.value:
        coords = tuple(message)
        pick_and_place_cmd_sender.send_arm_pose(coords)
        response = "success"
    elif task == Task.PICK_OBJECT.value:
        object_name = message
        num_attempts = 5 #number of attempts to estimate grasps
        for i in range(num_attempts):
            grasp_pose = get_best_tf_grasp(object_name)
            if grasp_pose is not None:
                break
            else:
                return "failed"

        pick_and_place_cmd_sender.send_gripper_command("01")
        response = "success"
    else:
        response = "Unknown task"

    return response

