import json
import yaml

# TODO real table estiamtion
TABLE_HEURISTIC = {
  "table corners": {"left back": [-0.51,0.27,1.77],"right back": [0.5,0.27,1.72],"left front": [-0.48,0.37,1.20],"right front": [0.08,0.1,0.20]}
}

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

    objects_info = { name  : {"id" : id, "diameter": diameters[id] } for name, id in name_to_id.items()}
    return objects_info