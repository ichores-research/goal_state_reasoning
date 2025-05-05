#!/bin/bash

# Build llm reasoner ros package
cd /root/catkin_ws/src
. /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin build


# Set the configuration file
source /root/catkin_ws/devel/setup.bash
rosparam load /root/config/${CONFIG} /pose_estimator;

# Pull ollama model
ollama pull llama3:70b

# Run LLM agent service
cd /root/goal_state_reasoning
python main.py