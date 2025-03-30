#!/usr/bin/env python3
"""
Name: placing_reasoner.py

Description:
    This file provides functionality for running a llm 
    with coordinate reasoning prompt.
    The model outputs coordinates where a held object should be places.

"""

import os
from langchain_core.prompts import PromptTemplate
from langchain_ollama.llms import OllamaLLM
from callbacks import AgentCallbackHandler
from langchain_core.prompts import PromptTemplate
from langchain_core.messages import AIMessage
if os.environ.get("TEST_RUN") != "TRUE":
    try:
        import rospy
        from ros_object_detections import parse_scene_for_placing
    except ImportError:
        pass
if os.environ.get("TEST_RUN") == "TRUE":
    try:
        from langchain_openai import ChatOpenAI
        import dotenv
        dotenv.load_dotenv()
    except:
        pass


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
        The table is represented by six coordinate values: min_x, max_x, min_y, max_y, min_z, max_z.

        The output is a point (x,y,z) where the held object should be released.
        The object can only be released on the table.

        After we have the output we will use motion planning system to operate the gripper.

        The following are example inputs and outputs.

        Input: {{'objects on the table': [
        {{'name': '029_plate', 'position': [0.07, 0.35, 1.45], 'diameter': 0.26}},
        {{'name': '011_banana', 'position': [0.06, 0.3, 1.55], 'diameter': 0.2}},
        ],
        'object in the gripper': {{'name': '025_mug', 'diameter': 0.13}},
        'command': 'place the mug to the right of the plate'}},
        'table limitations': {{'min_x': -0.51, 'max_x':0.5, 'min_y': 'no limits', 'max_y':0.5, 'min_z':1.2, 'max_z' : 2.5}}
        }}
        Output: (0.25, 0.31, 1.51)

        Your output is only one line and starts with "Output:", please do not output other redundant words. 
        The y coordinate cannot be greater than max_y.
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

        try:
            ai_message = ai_message.content
        except AttributeError:
            pass
        if ai_message[:7] != "Output:":
            return ai_message

        return ai_message[7:].strip()


    def run(self, object_in_the_gripper, where):
        if os.environ.get("TEST_RUN") == "TRUE":
            input = {}
            input['objects on the table'] = []
            input['objects on the table'].append({'name': '010_potted_meat_can', 'position': [0.21, 0.31, 1.5], 'diameter': 0.13})
            input['objects on the table'].append({'name': '029_plate', 'position': [0.06, 0.34, 1.39], 'diameter': 0.26})
            input['objects on the table'].append({'name': '011_banana', 'position': [-0.01, 0.2, 1.4], 'diameter': 0.2})
            input['table corners']= {'left back': [-0.51,0.27,1.77],'right back': [0.5,0.27,1.72],'left front': [-0.48,0.37,1.20],'right front': [0.08,0.1,0.20]}
            
            for obj in input['objects on the table']:
                if obj['name'] == object_in_the_gripper:
                    input['objects on the table'].remove(obj)
                    break
            input['object in the gripper'] = {'name': object_in_the_gripper, 'diameter': 0.2}
        else:
            input = parse_scene_for_placing(object_in_the_gripper)
        input["command"] = where
        output = self.coords_validation(self.pipe.invoke({"input": str(input)}))
        return output
        


if __name__ == "__main__":
    if os.environ.get("TEST_RUN") != "TRUE":
        rospy.init_node('placing_reasoner')
    
        reasoner = PlaceReasoner(
            OllamaLLM(
                model="llama3:70b",
                temperature=0,
                stop=["\nObservation", "Observation"],
                callbacks=[AgentCallbackHandler()],
                device="cuda"
            ))
    else:
        reasoner = PlaceReasoner( 
            ChatOpenAI(
            model="gpt-3.5-turbo",
            temperature=0,
            stop=["\nObservation", "Observation"],
            callbacks=[AgentCallbackHandler()],
        ))
    # Place SPAM canned meat in the scene
    #reasoner.run( object_in_the_gripper="013_apple", where="near the canned meat")
    input = """{{'objects on the table': [
    {{'name': '010_potted_meat_can', 'position': [0.21, 0.31, 1.5], 'diameter': 0.13}},
    {{'name': '029_plate', 'position': [0.06, 0.34, 1.39], 'diameter': 0.26}},
    ],
    'object in the gripper': {{'name': '011_banana', 'diameter': 0.2}},
    'command': 'on the 029_plate'}},
    "table limitations": {{"min_x": -0.42, "max_x": 0.55, "min_y": "no limits", "max_y":0.1, "min_z":0.21, "max_z":2.11}}
    }}"""
    reasoner.pipe.invoke({"input": str(input)})

    


    