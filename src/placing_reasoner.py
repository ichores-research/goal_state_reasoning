#!/usr/bin/env python3
"""
Name: placing_reasoner.py

Description:
    This file provides functionality for running a llm 
    with coordinate reasoning prompt.
    The model outputs coordinates where a held object should be places.

"""

from langchain_core.prompts import PromptTemplate
#from langchain_openai import ChatOpenAI
from langchain_ollama.llms import OllamaLLM
from callbacks import AgentCallbackHandler
from langchain.output_parsers import PydanticOutputParser
from langchain_core.prompts import PromptTemplate
from langchain_core.messages import AIMessage
#from ros_object_detections import parse_scene_for_placing
#import rospy
#from ros_comm import robot_execute, Task


class Singleton(type):
    """
    The Singleton class uses metaclass.
    The singleton metaclass is needed to initialize place reasoner in agent.py
    But Agent is not able to inject reasoner class in tool call.
    Therefore tools.py should be able to get preinitialized instancee of PlaceReasoner.
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]




class PlaceReasoner(metaclass=Singleton):
    def __init__(self, llm):
        
        self.template = """
        You are the controller of a Tiago PAL robot with one gripper. 

        The input is represented by 4 components:
            1. 3D objects located on the table
            2. One object held by you in the gripper
            3. A description of what to do with held object
            4. Table coordinate limitations

        Each object on the table is represented by its name, position (x,y,z) and diameter.
        The object held by you is described by name and diameter.
        The table is represented by four corners. Corners limit possible coordinate values. 

        The output is a point (x,y,z) where the held object should be released.
        The object can only be released on the table.

        After we have the output we will use motion planning system to operate the gripper.

        The following are example inputs and outputs.

        Your output is only one line and starts with "Output:", please do not output other redundant words. 

        Input: {{'objects on the table': [
        {{'name': '029_plate', 'position': [0.07, 0.35, 1.45], 'diameter': 0.26}},
        {{'name': '011_banana', 'position': [0.06, 0.3, 1.55], 'diameter': 0.2}},
        ],
        'object in the gripper': {{'name': '025_mug', 'diameter': 0.13}},
        'command': 'place the mug to the right of the plate'}},
        'table corners': {{'left back': [-0.51,0.27,1.77],'right back': [0.5,0.27,1.72],'left front': [-0.48,0.37,1.20],'right front': [0.08,0.1,0.20]}}
        }}
        Output: (0.25, 0.31, 1.51)

        Input: {{'objects on the table': [
        {{'name': '013_apple', 'position': [-0.01, 0.44, 2.45], 'diameter': 0.08}},
        {{'name': '014_lemon', 'position': [-0.09, 0.29, 1.33], 'diameter': 0.07}},
        {{'name': '011_banana', 'position': [-0.18, 0.29, 1.49], 'diameter': 0.2}},
        ],
        'object in the gripper': {{'name': '025_mug', 'diameter': 0.13}},
        'command': 'place the mug near the fruits'}},
        'table corners': {{'left back': [-0.51,0.27,1.77],'right back': [0.5,0.27,1.72],'left front': [-0.48,0.37,1.20],'right front': [0.08,0.1,0.20]}}
        }}
        Output: (0.12, 0.33, 1.32)

        Input: {{'objects on the table': [
        {{'name': '011_banana', 'position': [0.23, 0.31, 1.3], 'diameter': 0.2}},
        {{'name': '013_apple', 'position': [0.02, 0.41, 1.97], 'diameter': 0.08}},
        {{'name': '025_mug', 'position': [-0.12, 0.32, 1.57], 'diameter': 0.13}},
        ],
        'object in the gripper': {{'name': '006_mustard_bottle', 'diameter': 0.2}},
        'command': 'release the mustard bottle'}},
        'table corners': {{'left back': [-0.51,0.27,1.77],'right back': [0.5,0.27,1.72],'left front': [-0.48,0.37,1.20],'right front': [0.08,0.1,0.20]}}
        }}
        Output: (-0.19, 0.34, 1.31)

        Input: {input}
        """

        self.llm = llm

        self.prompt = PromptTemplate(
            template=self.template,
            input_variables=["input"]
        )

        # And a query intended to prompt a language model to populate the data structure.
        self.pipe = self.prompt | self.llm | self.coords_validation
    
    def coords_validation(self, ai_message: AIMessage):
        if ai_message[:7] != "Output:":
            raise ValueError("Badly formed answer. Please start with 'Output:'")
        try:
            output = tuple(ai_message[7:].strip())
            return output
        except ValueError:
            raise ValueError("Badly formed answer. Please provide coordinates as a tuple (x,y,z)")
        finally:
            return ai_message

    def run(self, object_in_the_gripper, where):
        input = parse_scene_for_placing(object_in_the_gripper)
        input["command"] = where
        output = self.pipe.invoke({"input": str(input)})
        return self.parser.invoke(output)
        


if __name__ == "__main__":
    #rospy.init_node('placing_reasoner')
    reasoner = PlaceReasoner(
        OllamaLLM(
            model="llama3.2:1b",
            temperature=0,
            stop=["\nObservation", "Observation"],
            callbacks=[AgentCallbackHandler()],
            device="cuda"
        ))
    # Place SPAM canned meat in the scene
    #reasoner.run( object_in_the_gripper="013_apple", where="near the canned meat")
    input = """{{'objects on the table': [
    {{'name': '010_potted_meat_can', 'position': [0.21, 0.31, 1.5], 'diameter': 0.13}},
    {{'name': '029_plate', 'position': [0.06, 0.34, 1.39], 'diameter': 0.26}},
    ],
    'object in the gripper': {{'name': '011_banana', 'diameter': 0.2}},
    'command': 'place the banana on the plate'}},
    'table corners': {{'left back': [-0.51,0.27,1.77],'right back': [0.5,0.27,1.72],'left front': [-0.48,0.37,1.20],'right front': [0.08,0.1,0.20]}}
    }}"""
    output = reasoner.pipe.invoke({"input": str(input)})
    print( output)

    