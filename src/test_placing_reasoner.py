import argparse
import csv
from datetime import datetime
import json

import numpy as np
from placing_reasoner import PlaceReasoner
from langchain_ollama.llms import OllamaLLM
from callbacks import AgentCallbackHandler
import rospy
from ros_object_detections import parse_scene_for_placing



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
        default=f"test_placing_reasoner_{datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}.json",
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
        output = reasoner.run(args.object_in_gripper, args.command)
        try: 
            output = eval(output)
        except:
            num_parsing_errors += 1
            pass
        prompt = parse_scene_for_placing(args.object_in_gripper)
        outputs.append({"output": output, "prompt": prompt})
    
    json_file = args.output_file
    # Open the file in write mode and write the data as JSON
    with open(json_file, 'w') as file:
        json.dump(outputs, file, indent=4)

    print(f"Data saved to {json_file}")
    avg_coords = np.mean(np.array([output['output'] for output in outputs]), axis=0)
    print(f"Average output coordinates: {avg_coords}")
    print(f"Number of output parsing errors: {num_parsing_errors}")
