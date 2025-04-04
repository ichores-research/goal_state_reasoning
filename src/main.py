#!/usr/bin/env python3
import argparse
import os
from agent import Agent
from goal_state_reasoning_msgs.srv import LLM_planner_service, LLM_planner_serviceResponse
import rospy

# Initialize the agent globally, so it's created only once
agent = None



# Callback for the service
def handle_agent_call(req):
    global agent
    rospy.loginfo(f"Received request: {req.command}")

    # Check if the agent is already created
    if agent is None:
        rospy.loginfo("Creating the agent...")
        agent = Agent()  # Create the agent once

    try:
        agent.run(req.command)
        action_success = True  # Assume the action was successful
    except Exception as e:
        rospy.logerr(f"Error running LLM agent: {e}")
        action_success = False  # Assume the action was successful
    

    return LLM_planner_serviceResponse(success=action_success)

def LLM_planner_service_server():


    # Initialize the ROS node
    rospy.init_node('LLM_planner')

    # Advertise the Trigger service
    rospy.Service('/LLM_plan_and_exec', LLM_planner_service, handle_agent_call)

    rospy.loginfo("LLM planner service ready.")

    # Keep the service alive
    rospy.spin()


if __name__ == "__main__":
    # get query from command line arg
    parser = argparse.ArgumentParser()
    parser.add_argument("--test_run", action='store_true')
    parser.add_argument("--query", type=str, help="Query for the ReAct Agent", default="make a bowl of fruits")
    args = parser.parse_args()

    if not args.test_run:
        LLM_planner_service_server()

    else:
        os.environ["TEST_RUN"] = "TRUE"
        agent = Agent()
        agent.run(args.query)
    
    