cmake_minimum_required(VERSION 2.4.6)
project(goal_state_reasoning_msgs)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS 
  rospy
  message_generation 
  std_msgs 
)


# Declare the service files to be built
add_service_files(FILES
  LLM_planner_service.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
)
