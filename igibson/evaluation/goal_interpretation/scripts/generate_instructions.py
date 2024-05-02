
from tqdm import tqdm
import json
import openai
import time
import re

oai_key_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/openai_api_key.txt"
prompt_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/prompts/goal_description_generation_prompt.txt"
natural_language_goal_conds_path = "/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/natural_language_goal_conditions.json"


# Function to retrieve openai api key
def get_openai_key(key_path):
	with open(key_path) as f:
		key = f.read().strip()

	print("Reading OpenAI API key from: ", key_path)
	return key

def get_gpt_output(message, model="gpt-3.5-turbo-0125", max_tokens=512, temperature=0, json_object=False):
    if json_object:
        if isinstance(message, str) and not 'json' in message.lower():
            message = 'You are a helpful assistant designed to output JSON. ' + message
    if openai.__version__.startswith('0.'):
        if isinstance(message, str):
            messages = [{"role": "user", "content": message}] 
        else:
            messages = message
        try:
            chat = openai.ChatCompletion.create(
                model=model, messages=messages
            ) 
        except Exception as e:
            print(f'{e}\nTry after 1 min')
            time.sleep(61)
            chat = openai.ChatCompletion.create(
                model=model, messages=messages
            ) 
        reply = chat.choices[0].message.content 
    else:
        if isinstance(message, str):
            messages = [{"role": "user", "content": message}] 
        else:
            messages = message
        kwargs = {"response_format": { "type": "json_object" }} if json_object else {}
        try:
            chat = openai.OpenAI().chat.completions.create(
                messages=messages,
                model=model,
                temperature=temperature,
                **kwargs
                )
        except Exception as e:
            print(f'{e}\nTry after 1 min')
            time.sleep(61)
            chat = openai.OpenAI().chat.completions.create(
                messages=messages,
                model=model,
                temperature=temperature,
                **kwargs
                )
        reply = chat.choices[0].message.content 
    return reply


with open(natural_language_goal_conds_path, 'r') as json_file:
    natural_language_goal_conds = json.load(json_file)

oai_key = get_openai_key(oai_key_path)






# Dictionary to hold the activity names and their corresponding natural goal conditions
instructions_by_activity_name = {}
    
    

for key, value in tqdm(natural_language_goal_conds.items()):
    
    generic_prompt = open(prompt_path, 'r').read()
    activity_name = key
    
    prompt = generic_prompt.replace('<task_name>', str(key))
    prompt = prompt.replace('<symbolic_goals>', str(value))
    
    out_json = json.loads(get_gpt_output(prompt))
    task_instructions = out_json['natural_language_goals']
    
    instructions_by_activity_name[activity_name] = task_instructions
    
    


# Save the dictionary as a JSON file
with open('/Users/bryan/Desktop/wkdir/behavior-vllm-eval/igibson/evaluation/goal_interpretation/assets/instructions_by_activity_name.json', 'w') as file:
    json.dump(instructions_by_activity_name, file, indent=4)