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
import random


random.seed(42)

YCB_OBJECTS = [
    'Pringles', 'Coffee', 'Cheez it cracker box', 'Sugar', 'Tomato soup', 'Mustard',
    'Jello pudding', 'Jello', 'Spam', 'Banana', 'Apple', 'Lemon', 'Peach', 'Pear', 
    'Orange', 'Plum', 'Soft scrub', 'Windex', 'Bowl', 'Mug', 'Plate'
]

PICKED_OBJECT = ""

# Utility function to return a random subset of YCB objects
def get_random_object_subset() -> List[str]:
    num_objects = random.randint(1, len(YCB_OBJECTS))
    return random.sample(YCB_OBJECTS, num_objects)



# Tool Definitions
@tool
def get_object_list(text:str) -> List[str]:
    """Returns a list of objects that are visible in the scene"""
    return get_random_object_subset()


@tool
def get_pointing_sequence(text:str) -> tuple:
    """Returns a sequence of objects that the user pointed to with their hand while issuing a command"""
    return tuple(get_random_object_subset())



@tool
def get_head_fixations(text:str) -> dict:
    """Returns a dictionary containing the sequence of objects the user was fixating on,
    along with the corresponding durations of their gaze on each object"""
    objects = get_random_object_subset()
    gaze_durations = {obj: random.uniform(1, 10) for obj in objects}
    return gaze_durations



@tool
def pick_object(object_name:str) -> str:
    """Pick an object with your robotic arm"""
    global PICKED_OBJECT
    object_name = object_name.strip()
    available_objects = get_object_list("")

    if PICKED_OBJECT != "":
        return f'Your robotic arm is busy holding {PICKED_OBJECT}'
    elif any(object_name.lower() in item.lower() for item in available_objects):
        PICKED_OBJECT = object_name
        return f'You have picked up {object_name}'
    else:
        return f'You cannot pick {object_name}'
    

@tool
def place_object(target_object_name:str) -> str:
    """Place the picked up object on top of target object"""
    global PICKED_OBJECT
    target_object_name = target_object_name.strip()
    available_objects = get_object_list("")

    if any(target_object_name.lower() in item.lower() for item in available_objects):
        result = f'You have placed {PICKED_OBJECT} on top of {target_object_name}'
        PICKED_OBJECT = ""
        return result
    else:
        return f'You cannot place {PICKED_OBJECT} on top of {target_object_name}'


@tool
def release_picked_object(text: str) -> str:
    """If your robotic arm is busy release the picked up object back on its original position"""
    global PICKED_OBJECT
    if PICKED_OBJECT == "":
        return f'You are not holding any objects'
    else:
        result = f'You have released {PICKED_OBJECT}'
        PICKED_OBJECT = ""
        return result

@tool
def none_tool_err(text: str) -> str:
    """Returns list of avaibale tools when tool is not found"""
    return f"Tool with name None not found. Available tools: {TOOL_LIST}"

# Tool List
TOOL_LIST = [
    get_object_list, 
    get_pointing_sequence, 
    get_head_fixations,
    pick_object,
    place_object,
    release_picked_object,
    none_tool_err
]

# Function to find a tool by name
def find_tool_by_name(tool_name: str) -> Tool:
    if tool_name is None:
        return none_tool_err
    for tool in TOOL_LIST:
        if tool.name == tool_name:
            return tool
    raise ValueError(f"Tool with name '{tool_name}' not found.")
