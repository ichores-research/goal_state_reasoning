"""
Name: tools.py

Description:
    This module contains mock definitions of tools for a future LLM (Language Model) Agent.
    These tools simulate various data parsing functions that will eventually be implemented.

    Current tools include:
    - Gesture recognition
    - Head position recognition
    - Object classification
    - [TBD] Scene understanding (object relative positions)
    - Pick object
    - Place object on top of target object

"""

import os
from langchain.agents import tool, Tool
from typing import List

if os.environ.get("TEST_RUN") != "TRUE":
    from ros_comm import robot_execute, Task

from placing_reasoner import PlaceReasoner


PICKED_OBJECT = None
OBJECTS_IN_SCENE = None

# Tool Definitions
@tool
def get_object_list(text:str) -> List[str]:
    """Returns a list of objects that are visible in the scene"""

    # Return example objects if in test run mode
    if os.environ.get("TEST_RUN") == "TRUE":
        obj_list = ["010_potted_meat_can", "029_plate", "011_banana"]
        if PICKED_OBJECT is not None:
            obj_list.remove(PICKED_OBJECT) 
        return obj_list
    
    # If not in test run mode, get the list of objects from the robot
    # Use a global variable to store the objects in the scene
    # this is a workaround for yolo or gdrnpp unstable detection
    # Once object are detected, they are assumed to be in the scene
    global OBJECTS_IN_SCENE
    obj_list = robot_execute(Task.GET_OBJECT_NAMES.value, "")
    if OBJECTS_IN_SCENE is None:
        OBJECTS_IN_SCENE = obj_list
    else:
        OBJECTS_IN_SCENE = set(OBJECTS_IN_SCENE).union(set(obj_list)) 

    if len(OBJECTS_IN_SCENE) == 0:
        return "No objects detected in the scene."
    
    return OBJECTS_IN_SCENE



@tool 
def get_object_pose(object_name:str) -> dict:
    """Get the position and orientation of the object in the scene"""
    if os.environ.get("TEST_RUN") == "TRUE":
        # Return a mock position for the object in test run mode
        return {
            "name": object_name,
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0]
        }
    
    return robot_execute(Task.GET_OBJECT_POSE.value, object_name)

@tool
def pick_object(object_name:str) -> str:
    """Pick an object with your robotic arm"""

    if os.environ.get("TEST_RUN") == "TRUE":
        return f'You have picked up {object_name}'

    
    global PICKED_OBJECT
    object_name = object_name.strip()
    object_name = object_name.strip("'")
    available_objects = get_object_list("")

    if PICKED_OBJECT is not None:
        return f'Your robotic arm is busy holding {PICKED_OBJECT}'
    elif object_name not in available_objects:
        return f'{object_name} is not available in the scene.'
    else:
        try:
            robot_execute(Task.PICK_OBJECT.value, object_name)
            PICKED_OBJECT = object_name
            return f'You have picked up {object_name}'
        except:
            return f'You cannot pick {object_name}. Try again or do somethig else.'
    


@tool
def place_object(where: str) -> str:
    """ Place the picked up object in a specific position. 
    `where` argument should describe the target position, eg. to the left of the mug, near the apple, etc."""

    global PICKED_OBJECT
    try:
        
        if os.environ.get("TEST_RUN") != "TRUE":
            #PlaceReasoner is a singleton class preinitialized in agent.py
            #Uses the same llm object as agent.py
            place_reasoner = PlaceReasoner()
            placing_coords = place_reasoner.run(PICKED_OBJECT, where)

            robot_execute(Task.PLACE_OBJECT.value, f"{placing_coords}")

        result = f'You have placed {PICKED_OBJECT} {where}'
        PICKED_OBJECT = None
        return result
    
    except BaseException as e:
        print(e)
        return f'You cannot place {PICKED_OBJECT} {where}. {e} Try again or do somethig else.'

@tool
def release_picked_object(text: str) -> str:
    """If your robotic arm is busy release the picked up object back on the table"""
    global PICKED_OBJECT
    if PICKED_OBJECT is None:
        return f'You are not holding any objects'
    else:
        try:
            result = f'You have released {PICKED_OBJECT}'
            PICKED_OBJECT = None
            return result
        except:
            return f'You cannot release {PICKED_OBJECT}. Try again or do somethig else.'



# Tool List
TOOL_LIST = [
    get_object_list, 
    get_object_pose,
    pick_object,
    place_object,
    release_picked_object
]


# Function to find a tool by namewhen tool is not found
def find_tool_by_name(tool_name: str) -> Tool:
    for tool in TOOL_LIST:
        if tool.name == tool_name:
            return tool
    return None
