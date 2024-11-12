import os
from typing import List, Union
from dotenv import load_dotenv
from langchain.agents import tool, Tool
from langchain.agents.output_parsers.react_single_input import ReActSingleInputOutputParser
from langchain_core.prompts import PromptTemplate
from langchain_openai import ChatOpenAI
from langchain.schema import AgentAction, AgentFinish
from langchain.agents.format_scratchpad import format_log_to_str
from ..ReAct.callbacks import AgentCallbackHandler
from langchain.tools.render import render_text_description



#decorator creating langchain tool
@tool 
def get_text_len(text:str)-> int:
    """Returns the length of a text by character """
    text = text.strip("'\n").strip(
        '"'
    )
    return len(text)

def find_tool_by_name(tools: List[Tool], tool_name: str) -> Tool:
    for tool in tools:
        if tool.name == tool_name:
            return tool
    raise ValueError(f"Tool wtih name {tool_name} not found")

def main():
    load_dotenv() #load env vars from .env file
    
    ## Prompt template
    # inside it the actual prompt can be injected through {input}
    # {agent_scratchpad} makes the agent "remember" what it has already outputed
    template = """
    Answer the following questions as best you can. You have access to the following tools:

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

    #tools available for LLM
    tools = [get_text_len]

    prompt = PromptTemplate.from_template(template=template).partial(
        tools=render_text_description(tools),
        tool_names=", ".join([t.name for t in tools]),
    )
    llm = ChatOpenAI(
        temperature=0,
        stop=["\nObservation", "Observation"],
        callbacks=[AgentCallbackHandler()],
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
                "input": "What is the length of the word: DOG",
                "agent_scratchpad": intermediate_steps, 
            }
        )

        # print the answer
        print(agent_step)

        # Check if the agent "wants" to execute some action
        if isinstance(agent_step, AgentAction):
            tool_name = agent_step.tool
            tool_to_use = find_tool_by_name(tools, tool_name)
            tool_input = agent_step.tool_input
            observation = tool_to_use.func(str(tool_input))
            print(f"{observation=}")
            intermediate_steps.append((agent_step, str(observation)))

    if isinstance(agent_step, AgentFinish):
        print("### AgentFinish ###")
        print(agent_step.return_values)


if __name__ == "__main__":
    main()