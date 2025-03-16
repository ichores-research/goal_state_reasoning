import json
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import argparse

names = {
    1: "001_chips_can",
    2: "002_master_chef_can",
    3: "003_cracker_box",
    4: "004_sugar_box",
    5: "005_tomato_soup_can",
    6: "006_mustard_bottle",
    7: "008_pudding_box",
    8: "009_gelatin_box",
    9: "010_potted_meat_can",
    10: "011_banana",
    11: "013_apple",
    12: "014_lemon",
    13: "015_peach",
    14: "016_pear",
    15: "017_orange",
    16: "018_plum",
    17: "021_bleach_cleanser",
    18: "024_bowl",
    19: "025_mug",
    20: "029_plate"
}

name_to_id = {v: k for k, v in names.items()}

with open("../config/models_info_ycb_ichores.json", "r") as f:
    models_info = json.load(f)

def get_oriented_bounding_box(mesh):
    
    bbox = mesh.get_oriented_bounding_box()
    bbox.color = (0.0, 1.0, 0.0)
    return bbox


def load_model(ply_file):
    model = o3d.io.read_point_cloud(ply_file)
    vertices = np.asarray(model.points)/1000
    colors = np.asarray(model.colors) if model.colors else None

    return model, vertices, colors


def create_mesh_from_points(points, triangles=None):
    """
    Creates an Open3D TriangleMesh from a NumPy array of points.

    Args:
        points (numpy.ndarray): NumPy array of shape (N, 3) representing the points.
        triangles (numpy.ndarray, optional): NumPy array of shape (M, 3) representing triangle indices.
                                             If None, a point cloud is created instead of a mesh.

    Returns:
        open3d.geometry.TriangleMesh or open3d.geometry.PointCloud: The created mesh or point cloud.
    """
    if not isinstance(points, np.ndarray) or points.shape[1] != 3:
        raise ValueError("Points must be a NumPy array of shape (N, 3).")

    if triangles is None:
      # create a point cloud if triangles are not provided.
      pcd = o3d.geometry.PointCloud()
      pcd.points = o3d.utility.Vector3dVector(points)
      return pcd

    if not isinstance(triangles, np.ndarray) or triangles.shape[1] != 3:
        raise ValueError("Triangles must be a NumPy array of shape (M, 3).")

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)

    return mesh

def load_mesh(ply_file, position, orientation, scale=0.5):
    
    model, vertices, colors = load_model(ply_file)
    shape_vertices = 3*int((vertices.shape[0] - 1)/3)
    points = np.zeros((shape_vertices, 3))
    for i in range(shape_vertices):
        points[i] = [vertices[i, 0], vertices[i, 1], vertices[i, 2]]


    
    mesh = create_mesh_from_points(points)
    mesh.paint_uniform_color([0.8, 0.3, 0.3])

    rot = Rotation.from_quat([orientation[0], orientation[1], orientation[2], orientation[3]])
    rotation_matrix = rot.as_matrix()

    center = mesh.get_center()
    mesh.rotate(rotation_matrix, center=center)
    mesh.translate(position)
    mesh.scale(scale, center=center)
    return mesh




def create_table(limitations):
    min_x = limitations['min_x'] -0.1
    max_x = limitations['max_x'] - 0.1
    min_y = limitations['min_y'] + 0.25
    max_y = limitations['max_y'] + 0.25
    max_z = limitations['min_z'] - 0.39# this is correct
    min_z = limitations['min_z'] - 0.4

    
    
    
    # Compute width, height, and depth
    width = max_x - min_x
    height = max_y - min_y
    depth = max_z - min_z

    # Create the box mesh
    box = o3d.geometry.TriangleMesh.create_box(width=width, height=height, depth=depth)

    R = box.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))  # 90-degree rotation
    box.rotate(R, center=(0, 0, 0))  # Rotate around origin
    # Move the box to the correct position
    box.translate(( min_x, min_y, min_z))
    box.paint_uniform_color([0.7, 0.7, 0.7])  # Light gray
    
    return box


def visualize_model_with_bbox(model_id, ply_file, position, orientation, scale=1):
    
    mesh = load_mesh(ply_file, position, orientation)
    mesh.scale(scale, center=mesh.get_center())
    bbox = get_oriented_bounding_box(mesh)
    bbox.scale(scale, center=bbox.get_center())

    o3d.visualization.draw_geometries([mesh, bbox], window_name="Object with Correctly Rotated Bounding Box")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_name", type=str, required=True, help="Model name")
    parser.add_argument("--position", type=str, required=True, help="Position (x, y, z)")
    parser.add_argument("--orientation", type=str, required=True, help="Orientation (x, y, z, w)")
    args = parser.parse_args()

    model_name = args.model_name
    position = np.array([float(x) for x in args.position.split(",")])
    orientation = np.array([float(x) for x in args.orientation.split(",")])

    model_id = name_to_id[model_name]
    ply_file = f"obj_{str(model_id).zfill(6)}.ply"
    
    
    visualize_model_with_bbox(model_id, ply_file, position, orientation)