"""
Testing checklist:
1) get robot and table in place -> table coordinates are set
2) choose scenario
3) place objects
4) take photo
5) take rviz screenshot
6) run test_placing_reasoner.py
7) get avg coords and visualize the object in rviz
8) take rviz  screenshot
*9) visualize all coords in rviz

Usage:
python test_placing_reasoner.py <obj_in_gripper> <command> --output_file <output_file> --rep <num_reps>
"""

import argparse
import csv
from datetime import datetime
import json

import numpy as np
from placing_reasoner import PlaceReasoner
from langchain_ollama.llms import OllamaLLM
from callbacks import AgentCallbackHandler
import rospy
from ros_object_detections import parse_scene_for_placing, get_object_pose



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the PlaceReasoner class.")
    parser.add_argument(
        "object_in_gripper",
        type=str,
        help="Object in gripper"
    )
    parser.add_argument(
        "command",
        type=str,
        help="Command to execute. WHERE to place held object"
    )
    parser.add_argument(
        "--output_file",
        type=str,
        default=f"test_output/test_placing_reasoner_{datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}.json",
        help="File to save the results"
    )
    parser.add_argument(
        "--rep",
        type=int,
        default=1,
        help="Number of test repetitions"
    )

    args = parser.parse_args()


    reasoner = PlaceReasoner(
            OllamaLLM(
                model="llama3:70b",
                temperature=0,
                stop=["\nObservation", "Observation"],
                callbacks=[AgentCallbackHandler()],
                device="cuda"
            ))
    rospy.init_node('test_placing_reasoner')
    outputs = []
    num_parsing_errors = 0
    for i in range(args.rep):
        
       
        try: 
            output = reasoner.run(args.object_in_gripper, args.command)
            output = eval(output)

            prompt = parse_scene_for_placing(args.object_in_gripper)
            
            for j, obj in enumerate(prompt['objects on the table']):
                orientation = get_object_pose(obj['name']).pose.orientation
                prompt['objects on the table'][j]['orientation'] =  [
                    round(orientation.x, 2), 
                    round(orientation.y, 2), 
                    round(orientation.z, 2), 
                    round(orientation.w, 2)]
        except Exception as e:
            num_parsing_errors += 1
            continue
        
        outputs.append({"output": output, "prompt": prompt, "command": args.command})
        print(f"Running test {i+1}/{args.rep}")
    
    json_file = args.output_file
    # Open the file in write mode and write the data as JSON
    with open(json_file, 'w') as file:
        json.dump(outputs, file, indent=4)

    print(f"Data saved to {json_file}")
    avg_coords = np.mean(np.array([output['output'] for output in outputs]), axis=0)
    print(f"Average output coordinates: {avg_coords}")
    print(f"Number of output parsing errors: {num_parsing_errors}")
