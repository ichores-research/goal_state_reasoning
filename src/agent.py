"""
Name: agent.py

Description:
    This file provides functionality for running a LangChain Agent
    The agent operates using ReAct algorithm.
    It can access tools (stubbed for now) needed for scene and command understanding.

"""

import os
from typing import Union

if os.environ.get("TEST_RUN") != "TRUE":
    import rospy

if os.environ.get("MODEL_HOST") == "openai":
    from langchain_openai import ChatOpenAI
    import dotenv
    dotenv.load_dotenv()
else:
    from langchain_ollama.llms import OllamaLLM

from langchain.schema import AgentAction, AgentFinish
from langchain.agents.format_scratchpad import format_log_to_str
from langchain.tools.render import render_text_description
from langchain.agents.output_parsers.react_single_input import ReActSingleInputOutputParser
from langchain_core.prompts import PromptTemplate

from callbacks import AgentCallbackHandler
from placing_reasoner import PlaceReasoner
from tools import TOOL_LIST, find_tool_by_name
import argparse


class Agent():
    def __init__(self):
        self.template = """
        A user gives you a command to execute. 
        Execute command using pick and place movements. 
        Answer with a sequence of pick and place movements.
        If the action is impossible to execute given the objects in the scene output: "Final Answer: I am not able to perform this action."
        
        You have access to the following tools:

        {tools}
        
        Use the following format:
        
        Question: the input question you must answer
        Thought: you should always think about what to do
        Action: the action to take, should be one of [{tool_names}]
        Action Input: the input to the action
        Observation: the result of the action
        ... (this Thought/Action/Action Input/Observation can repeat N times)
        Thought: I now know the final answer
        Final Answer: the final answer to the original input question
        
        Begin!
        
        Question: {input}
        Thought: {agent_scratchpad}
        """

        self.tools = TOOL_LIST

        self.prompt = PromptTemplate.from_template(template=self.template).partial(
            tools=render_text_description(self.tools),
            tool_names=", ".join([t.name for t in self.tools]),
        )
        if os.environ.get("MODEL_HOST") == "openai":
            self.llm = ChatOpenAI(
                model="gpt-4o-mini",
                temperature=0,
                stop=["\nObservation", "Observation"],
                callbacks=[AgentCallbackHandler()],
            )
        else:
            self.llm = OllamaLLM(
                model="llama3:70b",
                temperature=0,
                stop=["\nObservation", "Observation"],
                callbacks=[AgentCallbackHandler()],
                device="cuda"
            )



        # llm agentAvailable tools are: {[tool.name for tool in self.tools]}.
        self.agent = (
            {
                "input": lambda x: x["input"],
                "agent_scratchpad": lambda x: format_log_to_str(x["agent_scratchpad"]),
            }
            | self.prompt
            | self.llm
            | ReActSingleInputOutputParser()
        )

        #Singleton initialization
        PlaceReasoner(self.llm)

    def run(self, query):
        intermediate_steps = [] # reasoning steps
    
        # Initializing the ReAct loop
        agent_step = ""
        while not isinstance(agent_step, AgentFinish):
            ## First step: send inital prompt to llm
            # agent will return either action -> use tool
            # or an answer -> finish
            # Scratchpad stores a history of ReAct execution,
            # so the agent "knows" what have happened so far
            agent_step: Union[AgentAction, AgentFinish] = self.agent.invoke(
                {
                    "input": query,
                    "agent_scratchpad": intermediate_steps, 
                }
            )

            # print the answer
            print(agent_step)

            # Check if the agent "wants" to execute some action
            if isinstance(agent_step, AgentAction):
                tool_name = agent_step.tool
                print(type(tool_name), tool_name)

                if tool_name is None or tool_name == "None":
                    observation = "No action specified. To end the conversation, please state your final answer: 'Final Answer: ...'"
                else:
                    tool_to_use = find_tool_by_name(tool_name)

                    if tool_to_use is not None:
                        tool_input = agent_step.tool_input
                        observation = tool_to_use.func(str(tool_input))
                        print(f"{observation=}")
                    else:
                        observation = f"Tool {tool_name} not found. Available tools are: {[tool.name for tool in TOOL_LIST]}. "
                intermediate_steps.append((agent_step, str(observation)))

        if isinstance(agent_step, AgentFinish):
            print("### AgentFinish ###")
            print(agent_step.return_values)
            print("### Intermediate Steps ###", len(intermediate_steps))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "query",
        type=str,
        help="Query for the ReAct Agent",
        default="make a bowl of fruits",
    )
    args = parser.parse_args()

    if os.environ.get("TEST_RUN") != "TRUE":
        # Initialize the ROS node
        rospy.init_node(f'LLM_planner{__name__}')
   
    agent = Agent()
    agent.run(args.query)