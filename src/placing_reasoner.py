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




class PlaceReasoner():
    def __init__(self, llm):
        
        self.template = """
        You are the controller of a Tiago PAL robot with one gripper. 

        The input is represented by 3 components:
        1. 3D objects located on the table
        2. One object held by you in the gripper
        3. A description of what to do with held object.
        Each object on the table is represented by its name and 3D bounding box.
        3D bounding box consist of 2 points (x,y,z), top left back and bottom right front corners.
        The table is described by coordinates of 4 corners. Table for corners represent minimum and maximum coordinate values.
        The object held by you is described by name and dimentions (max width, max depth, max height)

        The output is a point (x,y,z) where the top left back corner of the held object should be released.
        The object can only be released on the table.

        After we have the output we will use motion planning system to operate the gripper.

        The following are past and conecutive inputs and outputs.

        Your output is only one line and starts with "Output:", please do not output other redundant words. 

        Input: {{
            'objects on the table': [
                {{'name': 'banana', 'bbox': [[0, 0, 0.5], [2, 1, 0]]}}, 
                {{'name': 'plate', 'bbox': [[2, 0, 0.3], [4, 2, 0]]}}, 
                {{'name': 'table', 'bbox': [[-5, -5, 0], [15, 7, 0]]}}], 
            'object in the gripper': {{'name': 'orange', 'dimentions': [1, 1, 1]}}, 
            'command': ['place the orange on top of the plate']}}
        Output: (1.0, 0, -2)

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
        
    

    def run(self, input):
        output = self.pipe.invoke({"input": input})
        return self.parser.invoke(output)
        


if __name__ == "__main__":
    reasoner = PlaceReasoner(
        OllamaLLM(
            model="llama3.2:1b",
            temperature=0,
            stop=["\nObservation", "Observation"],
            callbacks=[AgentCallbackHandler()],
            device="cuda"
        ))
    reasoner.run("{'objects on the table': [{'name': 'bowl', 'bbox': [[0, 0, 1], [1, 1, 0]]}, {'name': 'apple', 'bbox': [[1.5, 0, 1], [2.5, 1, 0]]}, {'name': 'orange', 'bbox': [[3, 0, 1], [4, 1, 0]]}, {'name': 'table', 'bbox': [[-5, -5, 0], [15, 7, 0]]}], 'object in the gripper': {'name': 'banana', 'dimentions': [2, 1, 0.5]}, 'command': ['place banana on top of the bowl']}")