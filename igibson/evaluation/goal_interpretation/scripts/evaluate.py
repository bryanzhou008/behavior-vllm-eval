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




demo_to_conds_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/all_conditions.json"
demo_to_objs_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/all_objects.json"
demo_names_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/100_selected_demos.txt"
task_to_instructions_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/instructions_by_activity_name.json"
prompt_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/prompts/behavior_goal_interpretation.txt"
task_to_demo_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/task_to_demo.json"
demo_to_prompt_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/llm_prompts.json"


with open(demo_to_conds_path, 'r') as json_file:
    demo_to_conds = json.load(json_file)

with open(demo_to_objs_path, 'r') as json_file:
    demo_to_objs = json.load(json_file)

with open(demo_to_prompt_path, 'r') as json_file:
    demo_to_prompt = json.load(json_file)

with open(task_to_instructions_path, 'r') as json_file:
    task_to_instructions = json.load(json_file)
    
with open(task_to_demo_path, 'r') as json_file:
    task_to_demos = json.load(json_file)

with open(demo_names_path, 'r') as file:
    demo_names = file.read().splitlines()
    



gpt35_results_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/gpt35_goal_interpretation.json"
gpt4_results_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/gpt4_goal_interpretation.json"

with open(gpt35_results_path, 'r') as json_file:
    gpt35_results = json.load(json_file)

with open(gpt4_results_path, 'r') as json_file:
    gpt4_results = json.load(json_file)



def flatten_goals(goal_data):
    """Flatten goal data into a single list of conditions."""
    return [condition for goal_type in goal_data.values() for condition in goal_type]

def check_satisfaction(predicted_conditions, ground_truth_conditions):
    satisfied_conditions = []
    unsatisfied_conditions = []
    for condition in ground_truth_conditions:
        if condition in predicted_conditions:
            satisfied_conditions.append(condition)
        else:
            unsatisfied_conditions.append(condition)
    return satisfied_conditions, unsatisfied_conditions



def evaluate_goals(predicted_goals, ground_truth_goals):
    """Evaluate predicted goals against ground truth goals."""
    # Flatten the predicted goals
    predicted_conditions = flatten_goals(predicted_goals)
    
    
    all_satisfied_conditions = []
    all_unsatisfied_conditions = []
    
    # check each goal in ground_truth_goals
    for key, value in ground_truth_goals.items():
        if len(value) == 1:
            satisfied_conditions, unsatisfied_conditions = check_satisfaction(predicted_conditions, value[0])
        # if there are multiple ways to satisfy the goal, choose the one that satisfies the most number of conditions
        else:
            satisfied_nums = [len([cond for cond in option if cond in predicted_conditions]) for option in value]
            max_satisfied_option = value[satisfied_nums.index(max(satisfied_nums))]
            satisfied_conditions, unsatisfied_conditions = check_satisfaction(predicted_conditions, max_satisfied_option)
        
        all_satisfied_conditions.extend(satisfied_conditions)
        all_unsatisfied_conditions.extend(unsatisfied_conditions) 
            
    # Compute evaluation metrics
    true_positives = len(all_satisfied_conditions)
    false_positives = len(predicted_conditions) - true_positives
    false_negatives = len(all_unsatisfied_conditions)
    precision = true_positives / (true_positives + false_positives) if true_positives + false_positives > 0 else 0
    recall = true_positives / (true_positives + false_negatives) if true_positives + false_negatives > 0 else 0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
    
    return {
        'accuracy': true_positives / len(predicted_conditions) if predicted_conditions else 0,
        'precision': precision,
        'recall': recall,
        'f1_score': f1_score,
        'all_satisfied_conditions': all_satisfied_conditions,
        'all_unsatisfied_conditions': all_unsatisfied_conditions,
        'predicted_conditions': predicted_conditions
    }
    





all_satisfied_conditions = []
all_unsatisfied_conditions = []
all_predicted_conditions = []

gpt4_results_evaluated = {}
gpt35_results_evaluated = {}


for demo in demo_names:
    goal_conds = demo_to_conds[demo]['goal_conditions']
    gpt4_pred = gpt4_results[demo]
    gpt35_pred = gpt35_results[demo]
    gpt4_result = evaluate_goals(gpt4_pred, goal_conds)
    gpt35_result = evaluate_goals(gpt35_pred, goal_conds)
    
    # all_satisfied_conditions.extend(results['all_satisfied_conditions'])
    # all_unsatisfied_conditions.extend(results['all_unsatisfied_conditions'])
    # all_predicted_conditions.extend(flatten_goals(gpt4_pred))
    gpt4_results_evaluated[demo] = gpt4_result
    gpt35_results_evaluated[demo] = gpt35_result
    
    

save_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/gpt4_goal_interpretation_evaluated.json"
with open(save_path, 'w') as json_file:
    json.dump(gpt4_results_evaluated, json_file, indent=4)

save_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/gpt35_goal_interpretation_evaluated.json"
with open(save_path, 'w') as json_file:
    json.dump(gpt35_results_evaluated, json_file, indent=4)
    

# print("all_satisfied_conditions: ", len(all_satisfied_conditions))
# print("all_unsatisfied_conditions: ", len(all_unsatisfied_conditions))
# print("all_predicted_conditions: ", len(all_predicted_conditions))
#  # Compute evaluation metrics
# true_positives = len(all_satisfied_conditions)
# false_positives = len(all_predicted_conditions) - true_positives
# false_negatives = len(all_unsatisfied_conditions)
# precision = true_positives / (true_positives + false_positives) if true_positives + false_positives > 0 else 0
# recall = true_positives / (true_positives + false_negatives) if true_positives + false_negatives > 0 else 0
# f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0


# result = {
#         'accuracy': true_positives / len(all_predicted_conditions) if all_predicted_conditions else 0,
#         'precision': precision,
#         'recall': recall,
#         'f1_score': f1_score,
#         'all_satisfied_conditions': all_satisfied_conditions,
#         'all_unsatisfied_conditions': all_unsatisfied_conditions
#     }

# print(result)