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
from pydantic import BaseModel, Field, field_validator
from ros_object_detections import parse_scene_for_placing
import rospy
from ros_comm import robot_execute, Task


class Singleton(type):
    """
    The Singleton class uses metaclass.
    The singleton metaclass is needed to initialize place reasoner in agent.py
    But Agent is not able to inject reasoner class in tool call.
    Therefore tools.py should be able to get preinitialized instancee of PlaceReasoner.
    """

    instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


# Define your desired data structure.
class OutputCoords(BaseModel):
    output_coords: str = Field(description="coordinates where the object should be placed")
    
    # You can add custom validation logic easily with Pydantic.
    @field_validator("output_coords") 
    def coords_validation(cls, field):
        if field[:7] != "Output:":
            raise ValueError("Badly formed answer. Please start with 'Output:'")
        try:
            tuple(field[7:].strip())
        except ValueError:
            raise ValueError("Badly formed answer. Please provide coordinates as a tuple (x,y,z)")
        return field


class PlaceReasoner(metaclass=Singleton):
    def __init__(self, llm):
        
        self.template = """
        You are the controller of a Tiago PAL robot with one gripper. 

        The input is represented by 4 components:

            3D objects located on the table
            One object held by you in the gripper
            A description of what to do with held object.
            Table coordinate limitations

        Each object on the table is represented by its name, position (x,y,z) and diameter.
        The object held by you is described by name and diameter.
        The table is represented by four corners. Corners limit possible coordinate values. 

        The output is a point (x,y,z) where the held object should be released.
        The object can only be released on the table.

        After we have the output we will use motion planning system to operate the gripper.

        The following are example inputs and outputs.

        Your output is only one line and starts with "Output:", please do not output other redundant words. 

        Input: {{'objects on the table': [
        {{'name': '024_bowl', 'position': [-0.33, 0.06, 1.06], 'diameter': 0.16}},
        {{'name': '011_banana', 'position': [0.29, 0.08, 0.88], 'diameter': 0.2}},
        {{'name': '006_mustard_bottle', 'position': [0.04, -0.01, 0.99], 'diameter': 0.2}},
        {{'name': '017_orange', 'position': [-0.04, 0.07, 0.62], 'diameter': 0.07}},
        ],
        'object in the gripper': {{'name': '013_apple', 'diameter': 0.08}},
        'command': 'place apple into the bowl'}}
        'table corners': {{"left back": [-0.51,0.27,1.77],"right back": [0.5,0.27,1.72],"left front": [-0.48,0.37,1.20],"right front": [0.08,0.1,0.20]}}
        Output: (-0.19, -0.03, 0.77)

        Input: {{'objects on the table': [
        {{'name': '011_banana', 'position': [0.39, 0.22, 0.93], 'diameter': 0.2}},
        {{'name': '017_orange', 'position': [0.31, 0.07, 0.79], 'diameter': 0.07}},
        {{'name': '014_lemon', 'position': [0.26, 0.19, 0.94], 'diameter': 0.07}},
        ],
        'object in the gripper': {{'name': '025_mug', 'diameter': 0.13}},
        'command': 'place the mug near fruits'}}
        'table corners': {{"left back": [-0.51,0.27,1.77],"right back": [0.5,0.27,1.72],"left front": [-0.48,0.37,1.20],"right front": [0.08,0.1,0.20]}}
        }}
        Output: (0.11, 0.1, 0.82)

        Input: {input}
        """

        self.llm = llm
        # Set up a parser + inject instructions into the prompt template.
        self.parser = PydanticOutputParser(pydantic_object=OutputCoords)

        self.prompt = PromptTemplate(
            template=self.template,
            input_variables=["input"]
        )

        # And a query intended to prompt a language model to populate the data structure.
        self.pipe = self.prompt | self.llm
        
    def run(self, object_in_the_gripper, where):
        input = parse_scene_for_placing(object_in_the_gripper)
        input["command"] = where
        output = self.pipe.invoke({"input": str(input)})
        return self.parser.invoke(output)
        


if __name__ == "__main__":
    rospy.init_node('placing_reasoner')
    reasoner = PlaceReasoner(
        OllamaLLM(
            model="llama3:70b",
            temperature=0,
            stop=["\nObservation", "Observation"],
            callbacks=[AgentCallbackHandler()],
            device="cuda"
        ))
    # Place SPAM canned meat in the scene
    input = robot_execute(Task.PLACE_OBJECT.value, "013_apple; near the canned meat")
    reasoner.run(input)

    