#!/bin/bash

# Build llm reasoner ros package
cd /root/catkin_ws/src
. /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin build


# Set the configuration file
source /root/catkin_ws/devel/setup.bash
rosparam load /root/config/${CONFIG} /pose_estimator;

# Serve ollama
echo "Going to serve ollama"
ollama serve &
echo "Served ollama"

# Pull ollama model
echo "Going to pull"
sleep 10 && ollama pull llama3.1:70b # TODO: CHEAP FIX, MAKE THIS RIGHT.
echo "Pulled model"

# Run LLM agent service
cd /root/goal_state_reasoning
python main.py