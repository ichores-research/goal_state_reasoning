#!/usr/bin/env python3
import os
import rospy
from object_detector_msgs.srv import detectron2_service_server,  estimate_poses
from sensor_msgs.msg import Image
from ycb_objects import *
import tf.transformations as tf_trans
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import PoseStamped

DATASET = os.environ.get("DATASET", "ycb_ichores")
OBJECT_INFO = get_ycb_objects_info(DATASET)


def detect_objects(rgb=None):
    if rgb is None:
        rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
    rospy.wait_for_service('/detect_objects')
    try:
        detect_objects_service = rospy.ServiceProxy('/detect_objects', detectron2_service_server)
        response = detect_objects_service(rgb)
        return response.detections.detections
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None


def estimate_object_pose(rgb, depth, detection):
    
    rospy.wait_for_service('estimate_poses')
    
    try:
        estimate_poses_service = rospy.ServiceProxy('estimate_poses', estimate_poses)
        response = estimate_poses_service(detection, rgb, depth)
        return response.poses
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def get_object_pose(object_name):
    rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
    depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)
    detections = detect_objects(rgb)

    if detections is None or len(detections) == 0:
        return None
    else:

        estimated_pose_camFrame = None

        try:
            for detection in detections:
                if detection.name == object_name:
                    estimated_pose_camFrame = estimate_object_pose(rgb, depth, detection)[0]
                    break
                
        except Exception as e:
            return "Pose estimation failed"
        return estimated_pose_camFrame

def get_object_poses():
    detections = detect_objects()
    return [get_object_pose(det.name) for det in detections]

def parse_scene_for_placing(object_in_the_gripper=None):
    scene = {}
    obj_poses = get_object_poses()
    scene['objects on the table'] = []
    for obj in obj_poses:
        scene['objects on the table'].append(
        { 
            'name': obj.name,
            'position': [round(obj.pose.position.x , 2), 
                         round(obj.pose.position.y , 2), 
                         round(obj.pose.position.z , 2)
                        ],
            'diameter': OBJECT_INFO[obj.name]['diameter']
        })
        scene['table limitations'] = TABLE_HEURISTIC
        if object_in_the_gripper is not None:
            scene['object in the gripper'] = {'name': object_in_the_gripper, 'diameter': OBJECT_INFO[object_in_the_gripper]['diameter']}
    return scene


def transform_grasps( grasps, pose):
    transformed_grasps = []

    # Convert object quaternion to a 4x4 transformation matrix, then extract the 3x3 rotation matrix
    obj_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    obj_transform = tf_trans.quaternion_matrix(obj_quat)  # This gives a 4x4 matrix
    obj_R = obj_transform[:3, :3]  # Extract the 3x3 rotation part
    obj_t = np.array([pose.position.x, pose.position.y, pose.position.z])  # Translation vector

    for grasp in grasps:
        # Convert the grasp to a 4x4 matrix
        grasp_matrix = np.array(grasp).reshape(4, 4)

        # Apply rotation and translation to transform the grasp to world coordinates
        transformed_grasp_matrix = np.eye(4)
        transformed_grasp_matrix[:3, :3] = np.dot(obj_R, grasp_matrix[:3, :3])  # Rotate
        transformed_grasp_matrix[:3, 3] = np.dot(obj_R, grasp_matrix[:3, 3]) + obj_t  # Rotate and translate

        # Flatten the transformed matrix and store it
        transformed_grasps.append(transformed_grasp_matrix.flatten())

    return np.array(transformed_grasps)


def get_pre_grasps(transformed_grasps, pre_grasp_distance = 0.1):
    pre_grasps_info = []
    align_x_to_z = tf_trans.quaternion_from_euler(0, np.pi / 2, 0)

    for grasp_matrix in transformed_grasps:
        grasp_matrix = np.array(grasp_matrix).reshape(4, 4)

        # Position of the Arrow (Base)
        # This is the pre-grasping point
        pre_grasp_position = grasp_matrix[:3, 3] 
        orientation_quat = tf_trans.quaternion_from_matrix(grasp_matrix)
        grasp_orientation = tf_trans.quaternion_multiply(orientation_quat, align_x_to_z)

        # Convert quaternion to rotation matrix
        rotation_matrix = tf_trans.quaternion_matrix(grasp_orientation)

        # Direction vector (initially along the x-axis)
        direction_vector = np.array([1.0, 0.0, 0.0])

        # Apply the rotation matrix to the direction vector
        rotated_direction = np.dot(rotation_matrix[:3, :3], direction_vector)

        # Scale the direction vector by the length of the arrow to find the distance from base to tip
        scaled_direction = rotated_direction * pre_grasp_distance

        end_position = pre_grasp_position + scaled_direction
        pre_grasps_info.append( {
                'grasp_position': end_position,
                'pre_grasp_position': pre_grasp_position, 
                'grasp_orientation': grasp_orientation,
                'grasp_diraction': scaled_direction
            }
        )
    return pre_grasps_info


def get_best_top_grasp(pre_grasps):
    """
    Compute the dot product of the rotated direction and the y-axis unit vector [0, 1, 0].

    Find the grasp with the maximum dot product: This ensures that the direction is closest to the y-axis.

    Check the direction: Ensure that the grasp's direction is pointing from high to low y (i.e., the direction should be negative along the y-axis).

    Return the grasp: Only the grasp that meets the above conditions should be returned.
    """

    best_top_grasp = None
    max_dot_product = -1 #the dot product of grasp direction and the y-axis unit vector (0, 1, 0)

    for grasp_info in pre_grasps:
        # Calculate the dot product with the y-axis unit vector (0, 1, 0)
        y_axis = np.array([0, 1, 0])
        grasp_diraction = grasp_info["grasp_diraction"]
        dot_product = np.dot(grasp_diraction, y_axis)

        # Check if the direction is close to the y-axis and points from high to low y (negative y-component)
        # this is to ensure top-grasp
        if dot_product > max_dot_product and grasp_diraction[1] < 0:
            max_dot_product = dot_product
            best_top_grasp = grasp_info
    return best_top_grasp


def get_object_grasping_points(object_name):
    if object_name not in OBJECT_INFO:
        print(f"Object {object_name} not found in dataset.")
        return None
    
    grasps_tfs = OBJECT_INFO[object_name]['grasps']

    pose = get_object_pose(object_name)
    
    if pose is None:
        print(f"Pose estimation failed for {object_name}.")
        return None

    # transform grasps to camera frame (where object poses are)
    try:
        estimated_grasps_camFrame = transform_grasps(grasps_tfs, pose.pose) 
    except Exception as e:
        print(f"Transformation failed for {object_name}: {e}")
        return None
    return estimated_grasps_camFrame


def get_grasping_coords(obj_name):
    """
    Retreive object preannotated grasps
    Calculate pre-grasping point and grasp direction and orientation
    Aims for the top-most grasp

    returns dict:
    {
        'grasp_position': # where gripper should be before closing,
        'pre_grasp_position': # where gripper should be before moving to grasp, 
        'grasp_orientation': # quaternion orientation of the gripper,
        'grasp_diraction': # direction of the gripper movement
    }
    """
    grasps = get_object_grasping_points(obj_name)
    if grasps is None:
        return None

    # Calculate pregrasping point and grasp direction
    pre_grasp = get_pre_grasps(grasps)

    # Calculate the pre-grasping point and grasp direction
    best_top_grasp = get_best_top_grasp(pre_grasp)
    return best_top_grasp


def matrix_to_pose(matrix, frame_id="head_link"):
    """Convert a 4x4 numpy transform matrix to a PoseStamped."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = matrix[0, 3]
    pose.pose.position.y = matrix[1, 3]
    pose.pose.position.z = matrix[2, 3]
    q = quaternion_from_matrix(matrix)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def get_z_alignment_score(pose):
    """Returns the alignment of the local Z-axis with world Z-axis."""
    rot = quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    z_axis = rot[:3, 2]  # local Z in world frame
    return z_axis[2]     # dot with global Z = [0, 0, 1]

def get_best_tf_grasp(object_name): 
    if object_name not in OBJECT_INFO:
        print(f"Object {object_name} not found in dataset.")
        return None

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Your list of 4x4 grasp transform matrices
    grasp_tf_matrices = OBJECT_INFO[object_name]['grasps']

    top_score = -float('inf')
    top_grasp_pose = None

    for mat in grasp_tf_matrices:
        tf_matrix = np.array(mat).reshape(4, 4)

        # Convert to PoseStamped in head_link
        grasp_pose = matrix_to_pose(tf_matrix, frame_id="xtion_rgb_optical_frame")

        try:
            # Transform to base_link
            transform = tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0), rospy.Duration(1.0))
            grasp_pose_base = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)

            # Score Z alignment
            score = get_z_alignment_score(grasp_pose_base.pose)

            if score > top_score:
                top_score = score
                top_grasp_pose = grasp_pose_base

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform failed for one grasp")

    # Final result
    if top_grasp_pose:

        
        print("Best top-down grasp found:")
        print("Position:", top_grasp_pose.pose.position)
        print("Orientation:", top_grasp_pose.pose.orientation)
        print("Z-alignment score:", top_score)
        
        
        return top_grasp_pose
    else:
        print("No valid grasp found.")
        return None


def heuristic_top_grasp(object_name):
    obj_pose = get_object_pose(object_name)
    if obj_pose is None:
        return
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0), rospy.Duration(1.0))
    grasp_pose_base = tf2_geometry_msgs.do_transform_pose(obj_pose, transform)
    grasp_pose_base.pose.position.z += 0.15
    grasp_pose_base.pose.position.x -= 0.35

    grasp_pose_base.pose.orientation.x  = 0
    grasp_pose_base.pose.orientation.y  = 0
    grasp_pose_base.pose.orientation.z  = 0
    grasp_pose_base.pose.orientation.w  = 1
    
    return grasp_pose_base


if __name__ == "__main__":
    rospy.init_node('object_detection_node')
    """
    rate = rospy.Rate(200)
    try:
        while not rospy.is_shutdown():
            detections = detect_objects()
            for detection in detections:
                pose = get_object_pose(detection.name)
                if pose is None:
                    continue
                try:
                    print(pose.name)
                    print(f"{round( pose.pose.position.x,2)}, {round(pose.pose.position.y,2)}, {round(pose.pose.position.z,2)}")
                except:
                    continue
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
  
    """
    detections = detect_objects()
    for detection in detections:
        print(heuristic_top_grasp(detection.name))
        break
    
    
    