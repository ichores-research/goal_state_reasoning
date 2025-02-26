"""
Name: agent.py

Description:
    This file provides functionality for running a LangChain Agent
    The agent operates using ReAct algorithm.
    It can access tools (stubbed for now) needed for scene and command understanding.

Usage: python agent.py <command for robot to execute>

"""

import random
import threading
from typing import Union
from dotenv import load_dotenv
from langchain.agents.output_parsers.react_single_input import ReActSingleInputOutputParser
from langchain_core.prompts import PromptTemplate
#from langchain_openai import ChatOpenAI
from langchain_ollama.llms import OllamaLLM
from langchain_community.llms import VLLM
from langchain.schema import AgentAction, AgentFinish
from langchain.agents.format_scratchpad import format_log_to_str
from callbacks import AgentCallbackHandler
from langchain.tools.render import render_text_description
from tools import find_tool_by_name, TOOL_LIST
import argparse
from ros_comm import server


def main():
    load_dotenv() #load env vars from .env file - for openAI key

    # get query from command line arg
    parser = argparse.ArgumentParser()
    parser.add_argument("query", type=str, help="Query for the ReAct Agent")
    args = parser.parse_args()
    input = args.query
    
    
    ## Prompt template
    # inside it the actual prompt can be injected through {input}
    # {agent_scratchpad} makes the agent "remember" what it has already outputed
    template = """
    A user gives you a command to execute. 
    Execute command using pick and place movements. 
    First think if the command is possible to execute given the objects in the scene. 
    Answer with a sequence of pick and place movements. 
    Answer the following questions as best you can.
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
    
    Question: How to {input}?
    Thought: {agent_scratchpad}
    """

    #tools available for LLM
    random.seed(42) #set seed for the tools to return eg. the same scene for ths execution
    tools = TOOL_LIST

    prompt = PromptTemplate.from_template(template=template).partial(
        tools=render_text_description(tools),
        tool_names=", ".join([t.name for t in tools]),
    )
    llm = OllamaLLM(
        model="llama3:70b",
        temperature=0,
        stop=["\nObservation", "Observation"],
        callbacks=[AgentCallbackHandler()],
        device="cuda"
    )


    # llm agent
    agent = (
        {
            "input": lambda x: x["input"],
            "agent_scratchpad": lambda x: format_log_to_str(x["agent_scratchpad"]),
        }
        | prompt
        | llm
        | ReActSingleInputOutputParser()
    )

    intermediate_steps = [] # reasoning steps
    
    # Initializing the ReAct loop
    agent_step = ""
    while not isinstance(agent_step, AgentFinish):
        ## First step: send inital prompt to llm
        # agent will return either action -> use tool
        # or an answer -> finish
        # Scratchpad stores a history of ReAct execution,
        # so the agent "knows" what have happened so far
        agent_step: Union[AgentAction, AgentFinish] = agent.invoke(
            {
                "input": input,
                "agent_scratchpad": intermediate_steps, 
            }
        )

        # print the answer
        print(agent_step)

        # Check if the agent "wants" to execute some action
        if isinstance(agent_step, AgentAction):
            tool_name = agent_step.tool
            tool_to_use = find_tool_by_name(tool_name)
            tool_input = agent_step.tool_input
            observation = tool_to_use.func(str(tool_input))
            print(f"{observation=}")
            intermediate_steps.append((agent_step, str(observation)))

    if isinstance(agent_step, AgentFinish):
        print("### AgentFinish ###")
        print(agent_step.return_values)


if __name__ == "__main__":
    ros_comm_thread = threading.Thread(target=server)
    ros_comm_thread.start()
    main()