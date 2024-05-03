import igibson.object_states as object_states
from igibson.tasks.behavior_task import BehaviorTask
from igibson.utils.ig_logging import IGLogReader
from igibson.utils.utils import parse_config
import os
import igibson
from igibson.envs.igibson_env import iGibsonEnv
from igibson.transition_model_v3.eval_env import EvalEnv
from igibson.transition_model_v3.eval_env import EvalActions
import igibson.object_states as object_states
from tqdm import tqdm
import json
import openai
import time

from igibson.evaluation.goal_interpretation.utils import get_gpt_output


demo_to_conds_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/all_conditions.json"
demo_to_objs_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/all_objects.json"
demo_names_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/100_selected_demos.txt"
task_to_instructions_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/instructions_by_activity_name.json"
prompt_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/prompts/behavior_goal_interpretation.txt"
task_to_demo_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/task_to_demo.json"
prompt_save_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/llm_prompts.json"
save_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/gpt35_goal_interpretation.json"
MODEL_NAME = "gpt-3.5-turbo"

    
def main():
    '''
    This script is used to generate GPT predictions for goal conditions for the demos in the dataset.
    
    ----------------------------Required Inputs----------------------------
    base prompt to be modified (prompt_path)
    relevant objects (with all possible states) (demo_to_objs_path)
    initial and goal conditions (demo_to_conds_path)
    task instructions (task_to_instructions_path)
    list of demo names (demo_names_path)
    mapping from task name to demo name (task_to_demo_path)
    
    ----------------------------Produced Outputs----------------------------
    all final generated prompts (prompt_save_path)
    GPT predictions for goal conditions (save_path)
    
    '''
    with open(demo_to_conds_path, 'r') as json_file:
        demo_to_conds = json.load(json_file)

    with open(demo_to_objs_path, 'r') as json_file:
        demo_to_objs = json.load(json_file)

    with open(task_to_instructions_path, 'r') as json_file:
        task_to_instructions = json.load(json_file)
        
    with open(task_to_demo_path, 'r') as json_file:
        task_to_demos = json.load(json_file)

    with open(demo_names_path, 'r') as file:
        demo_names = file.read().splitlines()


    if os.path.exists(save_path):
        with open(save_path, 'r') as json_file:
            gpt_goal_interpretation_results = json.load(json_file)
    else:
        gpt_goal_interpretation_results = {}

    demo_to_llm_prompts = {}
    
    
    
    for key, value in tqdm(task_to_demos.items()):
        task_name = key
        demo_name = value
        
        if demo_name in gpt_goal_interpretation_results:
            continue

        initial_conditions = demo_to_conds[demo_name]["initial_conditions"]
        goal_conditions = demo_to_conds[demo_name]["goal_conditions"]
        objects = demo_to_objs[demo_name]
        instructions = task_to_instructions[task_name]
        
        initial_conditions_string = '\n'.join(str(lst) for lst in initial_conditions.values())
        objects_string = '\n'.join(f"{key}: {value}" for key, value in objects.items())
        instructions_string = json.dumps({"Task Name": task_name, "Goal Instructions": instructions}, indent=4)
        
        generic_prompt = open(prompt_path, 'r').read()
        
        prompt = generic_prompt.replace('<object_in_scene>', objects_string)
        prompt = prompt.replace('<all_initial_states>', initial_conditions_string)
        prompt = prompt.replace('<instructions_str>', instructions_string)
        
        # save the prompts as well
        demo_to_llm_prompts[demo_name] = prompt
        
        gpt_preds = json.loads(get_gpt_output(prompt, model=MODEL_NAME))
        
        gpt_goal_interpretation_results[demo_name] = gpt_preds
        
        # Print the saved conditions for transparency
        print("\n\n")
        print("Saving GPT goal conditions predictions for demo: ", demo_name)
        print("\n\n")

        # Save the updated dictionary to the JSON file immediately
        with open(save_path, 'w') as json_file:
            json.dump(gpt_goal_interpretation_results, json_file, indent=4)
        
        
    # save the prompts as well
    with open(prompt_save_path, 'w') as json_file:
        json.dump(demo_to_llm_prompts, json_file, indent=4)
    
    

if __name__ == "__main__":
    main()

    

    