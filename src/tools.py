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

from langchain.agents import tool, Tool
from typing import List
from ros_comm import robot_execute, Task

PICKED_OBJECT = None


# Tool Definitions
@tool
def get_object_list(text:str) -> List[str]:
    """Returns a list of objects that are visible in the scene"""
    return robot_execute(Task.GET_OBJECT_NAMES.value, "")


# @tool
# def get_pointing_sequence(text:str) -> tuple:
#     """Returns a sequence of objects that the user pointed to with their hand while issuing a command"""
#     return tuple()




@tool
def pick_object(object_name:str) -> str:
    """Pick an object with your robotic arm"""
    global PICKED_OBJECT
    object_name = object_name.strip()
    available_objects = get_object_list("")

    if PICKED_OBJECT is not None:
        return f'Your robotic arm is busy holding {PICKED_OBJECT}'
    else:
        PICKED_OBJECT = object_name
        return f'You have picked up {object_name}'
    # else:
    #     return f'You cannot pick {object_name}, try again or do somethig else.'
    

@tool 
def get_object_position(object_name:str) -> dict:
    """Get the position and orientation of the object in the scene"""
    return robot_execute(Task.GET_OBJECT_POSE.value, object_name)

# @tool
# def place_object(target_object_name:str) -> str:
#     """Place the picked up object on top of target object"""
#     global PICKED_OBJECT
#     target_object_name = target_object_name.strip()
#     available_objects = get_object_list("")

#     if any(target_object_name.lower() in item.lower() for item in available_objects):
#         result = f'You have placed {PICKED_OBJECT} on top of {target_object_name}'
#         PICKED_OBJECT = None
#         return result
#     else:
#         return f'You cannot place {PICKED_OBJECT} on top of {target_object_name}'


@tool
def place_object(x:float, y:float, z:float) -> str:
    """ Place the picked up object in a specific position. Before using this tool get positions of objects in the scene."""
    global PICKED_OBJECT

    result = f'You have placed {PICKED_OBJECT} at position ({x}, {y}, {z})'
    PICKED_OBJECT = None
    return result


@tool
def release_picked_object(text: str) -> str:
    """If your robotic arm is busy release the picked up object back on its original position"""
    global PICKED_OBJECT
    if PICKED_OBJECT is None:
        return f'You are not holding any objects'
    else:
        result = f'You have released {PICKED_OBJECT}'
        PICKED_OBJECT = None
        return result



# Tool List
TOOL_LIST = [
    get_object_list, 
    #get_pointing_sequence, 
    pick_object,
    place_object,
    release_picked_object
]

# Function to find a tool by namewhen tool is not found
def find_tool_by_name(tool_name: str) -> Tool:
    if tool_name is None:
        return "Available tools are: " + ", ".join([tool.name for tool in TOOL_LIST])
    for tool in TOOL_LIST:
        if tool.name == tool_name:
            return tool
    
