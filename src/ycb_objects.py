import json
import yaml

def get_ycb_objects_info(dataset):
    with open(f"../config/{dataset}.yaml", 'r') as file:
        id_to_name = yaml.safe_load(file)["names"]
        name_to_id = {v: k for k, v in id_to_name.items()}

    with open(f"../config/models_info_{dataset}.json", 'r') as file:
        models_info = json.load(file)
        # The object parameters need to be adjusted to coordinate scale
        # which is 1000 times smaller
        # this info is based on taks/ scripts from ichores pipeline
        diameters = { int(model) : round( model["diameter"] / 1000 , 2) for i, model in enumerate(models_info.keys())}

    objects_info= {k : {"name" : v, "diameter": diameters[k] } for k, v in name_to_id}
    return objects_info