# Goal State Reasoning
Using LLM Agent prompting algorithms for object goal state understanding.

--- 
## Project Description 
Agent gets a goal state verbal command from user. It has access to the following tools:
  - Gesture recognition (output: sequence of objects that user pointed to during verbal command)
  - Head position recognition (output: sequence of objects that user fixated on to during verbal command)
  - List of objects available on the scene
  - Possibility to pick an object
  - Possibility to place an object on top of another object

The list of tools will be refined with the developement of this project. *For now all the tools are mocked, see [ReAct/tools.py](https://github.com/ichores-research/goal_state_reasoning/blob/main/ReAct/tools.py).*

The agent processes the verbal command and optionally input from the tools (using one of [prompting algorithms](https://github.com/ichores-research/goal_state_reasoning/tree/main#prompting-algorithms)) and outputs a sequence of pick and place movements.


## Goal State
The user should be able to command the robot to acheive certain object goal state without explaining the nesseccary state.
Eg. instead of saying...

*Pick up the cereal box and put some into a bowl. Then  pick up the milk jug and pour some into the bowl*

...user should be able to ask the robot to simply command:

*Make a bowl of cereal for me.*

## LLM Agents
Agents are systems that use LLMs as reasoning engines to determine which actions to take and the inputs necessary to perform the action. After executing actions, the results can be fed back into the LLM to determine whether more actions are needed, or whether it is okay to finish. More on agents -> see [LangChain documentation](https://python.langchain.com/docs/tutorials/agents/)

### Prompting Algorithms
#### ReAct
[Yao, Shunyu, et al. "React: Synergizing reasoning and acting in language models." arXiv preprint arXiv:2210.03629 (2022).](https://arxiv.org/abs/2210.03629)
In ReAct prompting LLMs are setup to generate both reasoning traces and task-specific actions.

Generating reasoning traces allow the model to induce, track, and update action plans, and even handle exceptions. The action step allows to interface with and gather information from external sources such as knowledge bases or environments.

![ReAct example](https://github.com/user-attachments/assets/0b622b50-2e01-4868-b160-0e4e5f027a45)

#### Iteration of Thought \[TBD\]
[Radha, Santosh Kumar, et al. "Iteration of thought: Leveraging inner dialogue for autonomous large language model reasoning." arXiv preprint arXiv:2409.12618 (2024).](https://arxiv.org/abs/2409.12618)

![Iteration of Thought schema](https://github.com/user-attachments/assets/d2a44903-540f-44a4-a762-682db152b58c)


## How to run

```
cd docker
ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=$ROS_IP  DATASET=ycb_ichores CONFIG=params_tiago.yaml docker-compose up -d
```

TBD start with agent 
```
# temporary
docker-compose exec goal_state_reasoning bash

#inside container
python3 agent.py "command"
```
## Example Result (ReAct)
**Human:** 
Make a bowl of fruits

**LLM Response:**
First, I need to check what objects are available in the scene to see if we have fruits and a bowl. Then, I need to determine the sequence of objects the user pointed to or fixated on to understand the order in which they want the fruits placed in the bowl.

**Action:** get_object_list\
**Action Input:** "scene"\
**observation**=['Pringles', 'Coffee', 'Cheez it cracker box', 'Sugar', 'Tomato soup', 'Mustard', 'Jello pudding', 'Jello', 'Spam', 'Banana', 'Apple', 'Lemon', 'Peach', 'Pear', 'Orange', 'Plum', 'Soft scrub', 'Windex', 'Bowl', 'Mug', 'Plate']

I see that we have fruits (Banana, Apple, Lemon, Peach, Pear, Orange, Plum) and a Bowl in the scene.


**Action:** get_pointing_sequence\
**Action Input:** "sequence of objects pointed to"\
**observation**=['Bowl']

It seems the user only pointed to the bowl, so they likely want the fruits placed directly into the bowl.


**Action:** pick_object\
**Action Input:** "Banana"\
**observation**='You have picked up Banana'


**Action:** place_object\
**Action Input:** "Bowl"\
**observation**='You have placed Banana on top of Bowl'


**Final Answer:** To make a bowl of fruits, pick up each fruit (Banana, Apple, Lemon, Peach, Pear, Orange, Plum) one by one and place them in the bowl.


### Example 2 LLAMA3:70B

{'output': 'To make a bowl of fruits, I will execute the following pick and place movements:

1. Pick up "Bowl"
2. Place "Bowl" on "Table"
3. Pick up "Apple"
4. Place "Apple" in "Bowl"
5. Pick up "Banana"
6. Place "Banana" in "Bowl"
7. Pick up "Orange"
8. Place "Orange" in "Bowl"
9. Pick up "Peach"
10. Place "Peach" in "Bowl"
11. Pick up "Pear"
12. Place "Pear" in "Bowl"
And that\'s it! I have successfully made a bowl of fruits using the pick and place movements.'}