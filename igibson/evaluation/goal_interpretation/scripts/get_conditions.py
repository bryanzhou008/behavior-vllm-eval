from igibson.transition_model_v3.eval_env import EvalEnv
from tqdm import tqdm
import json
import os
from tqdm import tqdm

def get_conditions(demo_path):
    initial_conds = {}
    goal_conds = {}
    env = EvalEnv(demo_path=demo_path, mode="headless", use_pb_gui=False)
    for i, condition in enumerate(env.task.initial_conditions):
        initial_conds[i + 1] = condition.terms
    for i, condition in enumerate(env.task.goal_conditions):
        goal_conds[i + 1] = condition.flattened_condition_options

    return initial_conds, goal_conds

def process_demos(file_path, save_path):
    # Check if the output JSON file already exists and load existing data
    if os.path.exists(save_path):
        with open(save_path, 'r') as json_file:
            demo_to_conds = json.load(json_file)
    else:
        demo_to_conds = {}

    # Read demo names from the .txt file
    with open(file_path, 'r') as file:
        demo_names = file.read().splitlines()

    # Process each demo name with progress display using tqdm
    for demo_name in tqdm(demo_names):
        # Skip processing if demo_name is already in the JSON file
        if demo_name in demo_to_conds:
            continue

        # Create the full path for each demo
        demo_path = f"/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/data/virtual_reality/{demo_name}.hdf5"

        # Call the function to get the initial and goal conditions
        initial_conditions, goal_conditions = get_conditions(demo_path)

        # Add the result to the dictionary
        demo_to_conds[demo_name] = {
            "initial_conditions": initial_conditions,
            "goal_conditions": goal_conditions
        }

        # Print the saved conditions for transparency
        print("\n\n")
        print("Saving initial and goal conditions for demo: ", demo_name)
        print("Initial conditions: ", initial_conditions)
        print("Ground truth goal conditions: ", goal_conditions)
        print("\n\n")

        # Save the updated dictionary to the JSON file immediately
        with open(save_path, 'w') as json_file:
            json.dump(demo_to_conds, json_file, indent=4)

# Example usage
input_file = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/100_selected_demos.txt"
output_file = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/all_conditions.json"

process_demos(input_file, output_file)
