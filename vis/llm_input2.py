import json
from bbox import name_to_id

def table_heuristic(objects, model_info, table_size = 5):
    min_z = 1000
    min_x = 1000
    max_x = -1000
    min_y = 1000
    max_y = -1000
    max_diameter = -1000
    for obj in objects:
        model_id = name_to_id[obj["name"]]
        if (model_info[str(model_id)]["diameter"] / 1000)>max_diameter:
            max_diameter = round(model_info[str(model_id)]["diameter"] / 1000,2)
        if obj["pose"]["position"]["x"] < min_x:
            min_x = obj["pose"]["position"]["x"]
        if obj["pose"]["position"]["x"] > max_x:
            max_x = obj["pose"]["position"]["x"]
        if obj["pose"]["position"]["y"] < min_y:
            min_y = obj["pose"]["position"]["y"]
        if obj["pose"]["position"]["y"] > max_y:
            max_y = obj["pose"]["position"]["y"]
        if obj["pose"]["position"]["z"] < min_z:
            
            min_z = obj["pose"]["position"]["z"] - (model_info[str(model_id)]["diameter"] / 2000)
    

    return {'min_x' : round(min_x - table_size*max_diameter,2) , 
            'max_x' : round(max_x + table_size*max_diameter,2) , 
            'min_y' : round(min_y - table_size*max_diameter,2) , 
            'max_y' : round(max_y + table_size*max_diameter,2) , 
            'min_z' : round(min_z,2),
            'max_z' : 'no limits'}

def main():
    with open("../config/models_info_ycb_ichores.json", "r") as f:
        models_info = json.load(f)
    
    with open("example_configs.json", "r") as f:
        scenes = json.load(f)["scenes"]
    
    for scene in scenes:
        print("\n\nInput: {'objects on the table': [")
        
        for obj in scene["objects"]:
            model_id = name_to_id[obj["name"]]
            diameter = models_info[str(model_id)]["diameter"]
            print(
                f"{{'name': '{obj['name']}', 'position': [{round(obj['pose']['position']['x'],2)}, {round(obj['pose']['position']['y'],2)}, {round(obj['pose']['position']['z'],2)}], 'diameter': {round(diameter/1000, 2)}}},"
            )
        

        obj = scene["correct_answer"]
        model_id = name_to_id[obj["name"]]
        diameter = models_info[str(model_id)]["diameter"]
        
        print(f"],\n'object in the gripper': {{'name': '{obj['name']}', 'diameter': {round(diameter/1000, 2)}}},")
        print(f"'command': '{scene['command']}'}},")
        limitations = "'table corners': {'left back': [-0.51,0.27,1.77],'right back': [0.5,0.27,1.72],'left front': [-0.48,0.37,1.20],'right front': [0.08,0.1,0.20]}"
        print(limitations)
        print("}")
        print(f"\nOutput: ({round(obj['pose']['position']['x'], 2)}, {round(obj['pose']['position']['y'], 2)}, {round(obj['pose']['position']['z'], 2)})")
        

if __name__ == "__main__":
    main()