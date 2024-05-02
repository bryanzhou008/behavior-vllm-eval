from igibson.transition_model_v3.eval_env import EvalEnv
from igibson.transition_model_v3.eval_env import EvalActions
import platform
import igibson.object_states as object_states
from tqdm import tqdm
import json


def get_flattened_ground_truth_goal_conditions(demo_path):
    res = {}
    env=EvalEnv(demo_path=demo_path,mode="headless", use_pb_gui=False)
    for i, condition in enumerate(env.task.goal_conditions):
        res[i+1] = condition.flattened_condition_options
    
    return res



def process_demos(file_path, save_path):
    # Read demo names from the .txt file
    with open(file_path, 'r') as file:
        demo_names = file.read().splitlines()

    # Initialize the dictionary to collect goal conditions
    demo_to_gt_goal_cond = {}

    # Process each demo name
    for demo_name in tqdm(demo_names):
        # Create the full path for each demo
        demo_path = f"/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/data/virtual_reality/{demo_name}.hdf5"

        # Call the function to get the goal conditions
        goal_conditions = get_flattened_ground_truth_goal_conditions(demo_path)

        # Add the result to the dictionary
        demo_to_gt_goal_cond[demo_name] = goal_conditions
        
        print("\n\n")
        print("saving ground truth goal conditions for demo: ", demo_name)
        print("ground truth goal conditions: ", goal_conditions)
        print("\n\n")

    # Save the results to a JSON file with proper indentation
    with open(save_path, 'w') as json_file:
        json.dump(demo_to_gt_goal_cond, json_file, indent=4)

# Example usage
input_file = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/100_selected_demos.txt"
output_file = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/gt_goal_conditions.json"

process_demos(input_file, output_file)