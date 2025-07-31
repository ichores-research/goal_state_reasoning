#!/usr/bin/env python3

import numpy as np
from motion_msgs.srv import Prepare, Pick, PickRequest, PrepareRequest
from geometry_msgs.msg import Pose, PoseArray, Point32, PoseStamped
import rospy
from shape_msgs.msg import Mesh

import open3d as o3d
from shape_msgs.msg import Mesh, MeshTriangle
import tf.transformations as tft
import tf


def transform_grasp_obj2world(grasps, pose):
    transformed_grasps = []

    # Convert object quaternion to a 4x4 transformation matrix, then extract the 3x3 rotation matrix
    obj_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    obj_transform = tft.quaternion_matrix(obj_quat)  # This gives a 4x4 matrix
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



def o3d_to_shape_mesh(model):
    # Read PLY file using Open3D
    mesh_msg = Mesh()

    # Convert vertices
    vertices = np.asarray(model.vertices)
    triangles = np.asarray(model.triangles)

    # Add vertices
    for vertex in vertices:
        point = Point32()
        point.x = float(vertex[0]) / 1000.0
        point.y = float(vertex[1]) / 1000.0
        point.z = float(vertex[2]) / 1000.0
        mesh_msg.vertices.append(point)

    # Add triangles
    for triangle in triangles:
        mesh_triangle = MeshTriangle()
        mesh_triangle.vertex_indices = [int(triangle[0]), 
                                      int(triangle[1]), 
                                      int(triangle[2])]
        mesh_msg.triangles.append(mesh_triangle)

    return mesh_msg


def ndarray_to_pose_array(poses):
    pose_array = PoseArray()
    align_x_to_z = tft.quaternion_from_euler(0, np.pi / 2, 0)
    for pose in poses:
        matrix = pose.reshape(4,4)
        translation = matrix[:3, 3]
        orientation = tft.quaternion_from_matrix(matrix)
        adjusted_orientation = tft.quaternion_multiply(orientation, align_x_to_z)

        p = Pose()
        p.position.x = float(translation[0])
        p.position.y = float(translation[1])
        p.position.z = float(translation[2])
        p.orientation.x = float(adjusted_orientation[0])
        p.orientation.y = float(adjusted_orientation[1])
        p.orientation.z = float(adjusted_orientation[2])
        p.orientation.w = float(adjusted_orientation[3])
        pose_array.poses.append(p)
    return pose_array



def pick_object(mesh_path: str, grasps: np.ndarray, pose: Pose, **kwargs):
    prepare_service = rospy.ServiceProxy('/motion/prepare', Prepare)
    pick_service = rospy.ServiceProxy('/motion/pick', Pick)
    rospy.wait_for_service('/motion/prepare')
    rospy.wait_for_service('/motion/pick')

    # Prepare the robot for picking
    try:
        prepare_service(PrepareRequest())
    except rospy.ServiceException as e:
        print(f"Motion prepare call failed: {e}")
        return
    
    try:
        try:
            mesh = o3d.io.read_triangle_mesh(mesh_path)
            mesh_msg = o3d_to_shape_mesh(mesh)
        except Exception as e:
            print(f"Failed to read mesh from {mesh_path}: {e}")
            return

        grasps_transformed = transform_grasp_obj2world(grasps, pose)
        pose_array = ndarray_to_pose_array(grasps_transformed)

        # Create Pick message
        pick_req = PickRequest()
        pick_req.object_mesh = mesh_msg
        pick_req.object_pose = pose
        pick_req.grasps = pose_array
        

        # Call the pick service
        response = pick_service(pick_req)
        print(f"Pick service response: {response.success}, {response.message}")
    except Exception as e:
        print(f"An error occurred: {e}")



def test_pick(objects_info):
    


    listener = tf.TransformListener()
    listener.waitForTransform("xtion_rgb_optical_frame", "base_footprint", rospy.Time(), rospy.Duration(4.0))


    detections = detect_objects()
    if len(detections) == 0:
        print("No objects detected.")
        return

    for detection in detections:
        if detection.name == "011_banana":
            break

    pose_gdrnpp = get_object_pose(detection.name)
    if pose_gdrnpp is  None:
        print("Could not estimate object pose.")
        return


    pose_in_head = PoseStamped() #parsing to pose stamped
    pose_in_head.header.frame_id = "xtion_rgb_optical_frame"
    pose_in_head.header.stamp = rospy.Time(0)  # latest available

    pose_in_head.pose.position = pose_gdrnpp.pose.position
    pose_in_head.pose.orientation = pose_gdrnpp.pose.orientation

    print("Detected ", detection.name)
    print(f"At position :{round( pose_in_head.pose.position.x,2)}, {round(pose_in_head.pose.position.y,2)}, {round(pose_in_head.pose.position.z,2)}")



    try:
        pose_in_base = listener.transformPose("base_footprint", pose_in_head)
        print("Transformed pose:")
        print("Position:", pose_in_base.pose.position)
        print("Orientation:", pose_in_base.pose.orientation)
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Transform of the pose to base footprint failed.")
        return


    print("Attempting to pick...")

    object_info = objects_info.get(detection.name, None)
    if object_info is None:
        print(f"Object {object_name} not found in dataset.")
        return
    
    # objects are slightly incorporated in the table plane,
    # so this is moving them slightly higher
    pose_in_base.pose.position.z +=0

    pick_success = pick_object(
        mesh_path=object_info["mesh_path"],
        grasps = object_info["grasps"],
        pose=pose_in_base.pose

        )
    
    message = f"Picked {detection.name}!" if pick_success else f"Failed to pick {detection.name}"
    print(message)
    return
    
    

if __name__=="__main__":
    rospy.init_node('pick_and_place_test_node')

    import os
    from ycb_objects import get_ycb_objects_info
    from object_detection import *
    DATASET = os.environ.get("DATASET", "ycb_ichores")
    OBJECTS_INFO = get_ycb_objects_info(DATASET)
    try:
        test_pick(OBJECTS_INFO)

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
