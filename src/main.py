#!/usr/bin/env python3
import argparse
import os
from agent import Agent
import rospy
from ros_comm import robot_execute, Task

def main():
   
    # get query from command line arg
    parser = argparse.ArgumentParser()
    parser.add_argument("query", type=str, help="Query for the ReAct Agent", default="make a bowl of fruits")
    parser.add_argument("test_run", action='store_true')
    args = parser.parse_args()
    query = args.query

    # create agent
    agent = Agent()

    if not args.test_run:
        rospy.init_node("LLM_agent")
    else:
        os.environ["TEST_RUN"] = "TRUE"
    agent.run(query)

    # For testing purpouses
    
    # try:
    #     rate = rospy.Rate(10)

    #     while not rospy.is_shutdown():
    #         response = robot_execute(Task.GET_OBJECT_POSE.value, "017_orange")

    #         print(response)

    #         rate.sleep()

    # except rospy.ROSInterruptException:
    #     pass

if __name__=='__main__':
    main()
    