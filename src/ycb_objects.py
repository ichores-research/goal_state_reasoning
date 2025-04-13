import json
import os
import numpy as np
import yaml

# TODO real table estiamtion
TABLE_HEURISTIC = {
  "table corners": {"left back": [-0.45, -0.25, 1.2],"right back": [0.45, -0.25, 1.2],"left front": [-0.45, 0.16, 0.8],"right front": [0.45, 0.16, 0.8]}
}



def load_grasp_annotations(folder_path, yaml_file_path):
    with open(yaml_file_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    
    names = yaml_data.get('names', {})
    grasp_annotations = {}

    for obj_id, obj_name in names.items():
        filename = f"obj_{int(obj_id):06d}.npy"
        file_path = os.path.join(folder_path, filename)
        if os.path.exists(file_path):
            grasps = np.load(file_path)
            grasp_annotations[obj_name] = {'grasps': grasps}
    return grasp_annotations


def get_ycb_objects_info(dataset):
    with open(f"../config/{dataset}.yaml", 'r') as file:
        id_to_name = yaml.safe_load(file)["names"]
        name_to_id = {v: k for k, v in id_to_name.items()}

    with open(f"../config/models_info_{dataset}.json", 'r') as file:
        models_info = json.load(file)
        # The object parameters need to be adjusted to coordinate scale
        # which is 1000 times smaller
        # this info is based on taks/ scripts from ichores pipeline
        diameters = { int(model_id) : round( models_info[model_id]["diameter"] / 1000 , 2) for model_id in models_info}

    grasp_annotations = load_grasp_annotations(f"datasets/{dataset}/grasp_annotations", f"../config/{dataset}.yaml")

    objects_info = { 
        name  : {
            "id" : id, 
            "diameter": diameters[id], 
            "grasps": grasp_annotations.get(name, None)['grasps']
        } for name, id in name_to_id.items()
    }
    return objects_info