import rospy
import moveit_commander
import sys

rospy.init_node("list_moveit_groups", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
group_names = robot.get_group_names()

print("Available MoveIt planning groups:")
for name in group_names:
    print(f" - {name}")
