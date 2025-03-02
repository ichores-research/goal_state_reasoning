#!/usr/bin/env python3
import argparse
from agent import Agent
import rospy
from ros_comm import robot_execute, Task

def main():
   
    # get query from command line arg
    parser = argparse.ArgumentParser()
    parser.add_argument("query", type=str, help="Query for the ReAct Agent", default="place apple on the pan")
    args = parser.parse_args()
    query = args.query

    # create agent
    agent = Agent()

    rospy.init_node("LLM_agent")
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
    