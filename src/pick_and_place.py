import geometry_msgs
import moveit_commander
import rospy
from sensor_msgs.msg import Image as ROSImage
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PIL import Image
import numpy as np
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
import moveit_msgs

ARM_GROUP_NAME = "arm_torso"
HEAD_LINK_NAME = "head_2_link"
BASE_LINK_NAME = "base_footprint"


class PickandPlaceServer:
    def __init__(self):
        rospy.init_node("pick_and_place", anonymous=True)
        rospy.loginfo("pick and place objects")

        # Setup MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME, wait_for_servers=55.0)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Change end effector link
        self.move_group.set_end_effector_link("gripper_link")
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(30)
        self.move_group.set_num_planning_attempts(3)

        self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()

        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher(
            "/torso_controller/command", JointTrajectory, queue_size=1
        )
        self.head_cmd = rospy.Publisher(
            "/head_controller/command", JointTrajectory, queue_size=1
        )
        self.arm_cmd = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=1
        )
        self.gripper_cmd = rospy.Publisher(
            "/gripper_controller/command", JointTrajectory, queue_size=1
        )

    def lower_head(self):
        """
        Lowers the head of the robot to a predefined position.
        """
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ["head_1_joint", "head_2_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def move_to_positions(self, joint_names, positions_list, time_durations, publisher):
        """
        Moves the robot to the specified positions.

        :param joint_names: List of joint names.
        :param positions_list: List of joint position configurations.
        :param time_durations: List of time durations for each configuration.
        :param publisher: The ROS publisher to send the trajectory to.
        """
        rospy.loginfo("Moving to specified positions...")
        if len(positions_list) != len(time_durations):
            rospy.logerr("Mismatch between positions and time durations.")
            return

        jt = JointTrajectory()
        jt.joint_names = joint_names

        for positions, duration in zip(positions_list, time_durations):
            jtp = JointTrajectoryPoint()
            jtp.positions = positions
            jtp.time_from_start = rospy.Duration(duration)
            jt.points.append(jtp)

        publisher.publish(jt)
        rospy.loginfo("Trajectory published to %s", publisher.name)

    def prepare_robot(self):
        """
        Prepares the robot for operation by moving the torso and arm to a safe position.
        """
        rospy.loginfo("Unfolding arm safely")

        # Move torso first
        torso_joint_names = ["torso_lift_joint"]
        torso_positions_list = [[0.34]]  # Only one position for the torso
        torso_time_durations = [3.0]
        self.move_to_positions(
            torso_joint_names,
            torso_positions_list,
            torso_time_durations,
            self.torso_cmd,
        )

        # Move arm joints
        arm_joint_names = [
            "arm_1_link",
            "arm_2_link",
            "arm_3_link",
            "arm_4_link",
            "arm_5_link",
            "arm_6_link",
            "arm_7_link",
        ]
        arm_positions_list = [
            [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],
            [0.10, 0.47, -0.20, 1.56, -1.58, 0.25, 0.0],
            [0.10, 0.47, -0.20, 1.56, 1.60, 0.25, 1.19],
        ]
        arm_time_durations = [3.0, 8.5, 10.5]
        self.move_to_positions(
            arm_joint_names, arm_positions_list, arm_time_durations, self.arm_cmd
        )

        # Lower the head
        self.lower_head()
        rospy.loginfo("Robot prepared.")

    def pick(self, grasp_input, frame=HEAD_LINK_NAME):
        """
        Grasp input should contain:
        - grasp_position: [x, y, z] in head_link or base_link frame
        - pre_grasp_position: [x, y, z] in head_link or base_link frame
        - grasp_orientation: [x, y, z, w] in head_link or base_link frame
        - grasp_direction: [x, y, z] in head_link or base_link frame
        """
        rospy.loginfo("Prepare robot to pick object...")
        self.prepare_robot()
        rospy.loginfo("Object appeared in planning scene!")
        rospy.loginfo("Picking object...")

        #Convert grasp_input to base_link frame
        if frame == HEAD_LINK_NAME:
            listener = tf.TransformListener()
            listener.waitForTransform(HEAD_LINK_NAME, BASE_LINK_NAME, rospy.Time(0), rospy.Duration(4.0))

        self.move_arm(grasp_input['pre_grasp_position'])
        self.open_gripper()
        self.move_arm(grasp_input['grasp_position'], grasp_input['grasp_orientation'])
        self.close_gripper()
        
        rospy.loginfo("Picking object finished")
    
    def place(self, place_position, frame=HEAD_LINK_NAME):
        """
        Place position is a list of [x, y, z] in head_link or base_link frame
        """
        rospy.loginfo("Placing object...")

        #Convert place_position to base_link frame
        if frame == HEAD_LINK_NAME:
            listener = tf.TransformListener()
            listener.waitForTransform(BASE_LINK_NAME, HEAD_LINK_NAME, rospy.Time(0), rospy.Duration(4.0))
        self.move_arm(place_position)
        self.open_gripper()

        rospy.loginfo("Placing object finished")

    def set_gripper_pose(self, pose):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = pose
        point.time_from_start = rospy.Duration(1.0)
        joint_trajectory.points.append(point)

        self.gripper_pub.publish(joint_trajectory)

    def move_arm(self, target_position, target_orientation=None, frame=BASE_LINK_NAME):
        """
        target_pose is a list of [x, y, z] in head_link or base_link frame
        """

         # Get current gripper pose
        pose = self.move_group.get_current_pose().pose
        pose.position.x = target_position[0]
        pose.position.y = target_position[1]
        pose.position.z = target_position[2]
        if target_orientation is not None:
            pose.orientation.x = target_orientation[0]
            pose.orientation.y = target_orientation[1]
            pose.orientation.z = target_orientation[2]
            pose.orientation.w = target_orientation[3]
        
       

        self.move_group.set_pose_target(pose)

        rospy.loginfo("Planning...")
        (success, trajectory, time, error) = self.move_group.plan()
        rospy.logwarn(str(trajectory))
        rospy.loginfo("Planned.")

        # Vizualize the trajectory

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(trajectory)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        
        
        rospy.loginfo("Executing...")
        self.move_group.execute(trajectory)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        rospy.loginfo(f"Published action success: {success}")

        return success


    def open_gripper(self):
        self.set_gripper_pose([0.04, 0.04])
        
    
    def close_gripper(self):
        self.set_gripper_pose([0.0, 0.0])

 
    def wait_for_planning_scene_object(self, object_name='part'):
        rospy.loginfo("Waiting for object '" + object_name + "'' to appear in planning scene...")
        gps_req = GetPlanningSceneRequest()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

        part_in_scene = False
        while not rospy.is_shutdown() and not part_in_scene:
            # This call takes a while when rgbd sensor is set
            gps_resp = self.scene_srv.call(gps_req)
            # check if 'part' is in the answertable
            for collision_obj in gps_resp.scene.world.collision_objects:
                if collision_obj.id == object_name:
                    part_in_scene = True
                    break
            else:
                rospy.sleep(1.0)

        rospy.loginfo("'" + object_name + "'' is in scene!")

      


if __name__ == "__main__":
    try:
        server = PickandPlaceServer()

        #Example in head_link frame
        grasp_example_input = {
            'grasp_position': [0.09859165, 0.27464488,  1.3558509 ], 
            'pre_grasp_position': [0.09183268, 0.25901019,  1.25731221], 
            'grasp_orientation': [ 0.65864666,  0.07975477,  0.72624653, -0.17997146], 
            'grasp_direction': [0.00675897, 0.01563469,  0.09853869]
            }

        server.pick(grasp_example_input)
        server.place([0.20, -1.34, -0.20 ])

    except rospy.ROSInternalException:
        rospy.loginfo("Tiago Motion Controller shutting down.")
