
#!/usr/bin/env python
import subprocess
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
from ros_object_detections import get_best_tf_grasp, detect_objects, heuristic_top_grasp

HEAD_LINK_NAME = "head_1_link"
BASE_LINK_NAME = "base_link"



class PickAndPlaceCommandSender:
    def __init__(self):
        

        # Publishers for target_pose and target_gripper topics
        self.pose_pub = rospy.Publisher('/target_pose', Pose, queue_size=10)
        self.gripper_pub = rospy.Publisher('/target_gripper', String, queue_size=10)

        # Default pose values
        self.default_pose_right = [0.5, -0.5, 1.2]  # x, y, z
        self.default_pose_left = [0.5, 0.5, 1.2]  # x, y, z
        self.dafault_orientation = [0,0,0,0.5]

    def send_arm_pose(self, target_position, target_orientation=None):
        """
        Sends a Pose message to the /target_pose topic.
        """
        pose = Pose()

        # Set position (x, y, z)
        pose.position.x = target_position[0]
        pose.position.y = target_position[1]
        pose.position.z = target_position[2]

        if target_orientation is None:
            target_orientation = self.dafault_orientation

        pose.orientation.x = target_orientation[0]
        pose.orientation.y = target_orientation[1]
        pose.orientation.z = target_orientation[2]
        pose.orientation.w = target_orientation[3]

        arm = 'right'
        if pose.position.y >= 0.0:
            # this logic for determining which hand should be moved
            arm = 'left'

        # Log and publish the pose message
        rospy.loginfo(f"Sending {arm} arm to pose: {pose}")
        # Format the message as a YAML string
        pose_yaml = (
            f"{{position: {{x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}}}, "
            f"orientation: {{x: {pose.orientation.x}, y: {pose.orientation.y}, "
            f"z: {pose.orientation.z}, w: {pose.orientation.w}}}}}"
        )

        # Build the rostopic pub command
        cmd = [
            "rostopic", "pub", "/target_pose", "geometry_msgs/Pose",
            pose_yaml, "--once"
        ]

        # Run the command
        subprocess.run(cmd)

    def send_gripper_command(self, command):
        """
        Sends a String message to the /target_gripper topic.
        The command must be one of "00", "01", "10", "11".
        00 means open both gripers
        11 means close both grippers
        """
        if command in ["00", "01", "10", "11"]:
            rospy.loginfo(f"Sending gripper command: {command}")
            
            #Temporary workaround with subprocess
            # Build the command-line string
            topic_cmd = [
                "rostopic", "pub", "/target_gripper", "std_msgs/String",
                f"data: '{command}'", "--once"
            ]
            subprocess.run(topic_cmd)
        else:
            rospy.logwarn("Invalid gripper command. Please use one of '00', '01', '10', '11'.")

    def prepare_robot(self):
        rospy.loginfo(f"Preparing robot...")
        self.send_arm_pose(self.default_pose_right, self.dafault_orientation)
        self.send_arm_pose(self.default_pose_left, self.dafault_orientation)
        self.send_gripper_command("00")

   
    def transform_gdrnpp_input(self, position_gdrnpp, orientation_gdrnpp=None):
        
        #pose_in_head = position_gdrnpp
        listener = tf.TransformListener()

        # Define your point in the head_link frame
        pose_in_head = PoseStamped()
        pose_in_head.header.frame_id = "xtion_rgb_optical_frame"
        pose_in_head.header.stamp = rospy.Time(0)  # latest available

        #adjust axes order
        
        pose_in_head.pose.position.x = position_gdrnpp[2]
        pose_in_head.pose.position.y = position_gdrnpp[0] 
        pose_in_head.pose.position.z =  position_gdrnpp[1]

        if orientation_gdrnpp is not None:
            pose_in_head.pose.orientation.x = orientation_gdrnpp[0]
            pose_in_head.pose.orientation.y = orientation_gdrnpp[1] 
            pose_in_head.pose.orientation.z =  orientation_gdrnpp[2]
            pose_in_head.pose.orientation.w =  orientation_gdrnpp[3]


        # Wait for the transform to be available
        listener.waitForTransform("base_link", "xtion_rgb_optical_frame", rospy.Time(), rospy.Duration(4.0))

        try:
            pose_in_base = listener.transformPose("base_link", pose_in_head)
            print("Transformed pose:")
            print("Position:", pose_in_base.pose.position)
            print("Orientation:", pose_in_base.pose.orientation)
            return [ pose_in_base.pose.position.x,
                     pose_in_base.pose.position.y,
                     pose_in_base.pose.position.z], \
                    [ pose_in_base.pose.orientation.x,
                     pose_in_base.pose.orientation.y,
                     pose_in_base.pose.orientation.z,
                     pose_in_base.pose.orientation.w ]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform failed.")

  

    def pick(self, grasp_pose):
        """
        grasp_pose in base_link
        """
        rospy.loginfo("Prepare robot to pick object...")
        self.prepare_robot()
        rospy.loginfo("Object appeared in planning scene!")
        rospy.loginfo("Picking object...")


        # 1. Move arm to pregrasp pose
        orientation = quaternion_from_euler(1, 0, 0)
        self.send_arm_pose([grasp_pose.pose.position.x,
                            grasp_pose.pose.position.y,
                            grasp_pose.pose.position.z], orientation)
        
        


        # 3. Move arm to grasp pose
        self.send_arm_pose([grasp_pose.pose.position.x + 0.30,
                            grasp_pose.pose.position.y,
                            grasp_pose.pose.position.z], orientation)
        
        # 3. Close gripper
        gripper_cmd_close = "11"
        self.send_gripper_command(gripper_cmd_close)
        
        rospy.loginfo("Picking object finished")

        return "success"
    
    def place(self, place_position, frame=HEAD_LINK_NAME):
        """
        Place position is a list of [x, y, z] in head_link or base_link frame
        """
        rospy.loginfo("Placing object...")

        #Convert place_position to base_link frame
        #if frame == HEAD_LINK_NAME:
            
        
        # 1. Move arm to a pose a little bit above place pose
        place_position_transformed = self.transform_gdrnpp_input(place_position) 
        if place_position_transformed is None:
            return 

        #raise arm to a position higher than placeing pose
        place_position_transformed.point.z += 0.2

        self.send_arm_pose(place_position_transformed, quaternion_from_euler(1, 0, 0)) #downward orientation

        self.send_gripper_command("00")

        rospy.loginfo("Placing object finished")

        return "success"


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('pick_and_place_cmd_sender_node', anonymous=True)
        # Initialize and run the command sender
        sender = PickAndPlaceCommandSender()
        sender.prepare_robot()

        
        detections = detect_objects()
        for detection in detections:
            grasp_pose = heuristic_top_grasp(detection.name)
            print(grasp_pose)
            break
        #sender.place( [-0.05, 0.18, 0.64])
        if grasp_pose is not None:
            #sender.prepare_robot()
        
            # Now send both lists as needed
            sender.send_gripper_command("00")
            sender.pick(grasp_pose)
        #else:
        #    print("no grasp pose")
        
        
    except rospy.ROSInterruptException:
        pass