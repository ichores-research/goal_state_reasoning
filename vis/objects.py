import json

import numpy as np
from bbox import get_oriented_bounding_box, load_mesh, name_to_id, create_table
import open3d as o3d
from scipy.spatial.transform import Rotation
from llm_input2 import table_heuristic

with open("example_configs.json", "r") as file:
    scenes = json.load(file)["scenes"]
    

with open("../config/models_info_ycb_ichores.json", "r") as file:
    models_info = json.load(file)

if __name__ == "__main__":
    # Create a coordinate frame (axes)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    for scene in scenes:
        objects = scene["objects"]
        vis_objects = []
        meshes = []
        bboxes = []
        
        #table
        limitations = table_heuristic(objects, models_info, table_size = 1)
        table = create_table(limitations)
        
        for obj in objects:
            
            model_id = name_to_id[obj["name"]]
            position = (
                obj["pose"]["position"]["x"], 
                obj["pose"]["position"]["y"], 
                obj["pose"]["position"]["z"])
            orientation = (
                obj["pose"]["orientation"]["x"], 
                obj["pose"]["orientation"]["y"], 
                obj["pose"]["orientation"]["z"], 
                obj["pose"]["orientation"]["w"])
            ply_file = f"obj_{str(model_id).zfill(6)}.ply"
            
            mesh = load_mesh(ply_file, position, orientation)
            bbox = get_oriented_bounding_box(mesh)
            meshes.append(mesh)
            bboxes.append(bbox)
        
        
        

        o3d.visualization.draw_geometries(bboxes + meshes +[table, axes] , window_name=f"Scene Task {scene['command']}")
     
        target_model_id = name_to_id[scene["correct_answer"]["name"]]
        target_position = (
                scene["correct_answer"]["pose"]["position"]["x"], 
                scene["correct_answer"]["pose"]["position"]["y"], 
                scene["correct_answer"]["pose"]["position"]["z"])
        target_orientation = (
                scene["correct_answer"]["pose"]["orientation"]["x"], 
                scene["correct_answer"]["pose"]["orientation"]["y"], 
                scene["correct_answer"]["pose"]["orientation"]["z"], 
                scene["correct_answer"]["pose"]["orientation"]["w"])
        target_ply_file = f"obj_{str(model_id).zfill(6)}.ply"
        target_mesh = load_mesh(target_ply_file, target_position, target_orientation)
        target_bbox = get_oriented_bounding_box(target_mesh)
        o3d.visualization.draw_geometries(bboxes + meshes + [target_mesh, target_bbox, axes] +[table] , window_name="Scene Correct Answer")