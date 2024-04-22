from igibson.transition_model_v2.eval_env import EvalActions
from igibson.transition_model_v2.eval_env import EvalEnv
import platform
import io
from contextlib import redirect_stdout
import json
import fire

def evaluate_action_seqeunce(demo_path,action_path,rst_path,headless=True):
    env=EvalEnv(demo_path=demo_path,mode="headless" if headless else "gui_non_interactive",
        use_pb_gui=(not headless and platform.system() != "Darwin"),)
    
    with open(action_path, 'r') as f:
        actions=json.load(f)

    f=io.StringIO()
    rst={
        "all_action_execution_true":True,
        "unknown_execution_error":False,
        "all_condition_satisfied":False,
    }
    with redirect_stdout(f):
        print("Addressable Objects:")
        for obj in env.addressable_objects:
            print(obj.name)
        print("-----------------------------------------------")
        print("Initial Conditions: ")
        for condition in env.task.initial_conditions:
            print(condition.terms)
        print("-----------------------------------------------")
        print("Goal Conditions: ")
        for condition in env.task.goal_conditions:
            print(condition.terms)
        print("------------Action Execution Begins-------------")
        for action in actions:
            try:
                action_execuble=(action['action'], action['object'])
                print("Action: ",action_execuble)
                _,_,_,_,flag=env.step(action_execuble)
                if not flag:
                    rst["all_action_execution_true"]=False
                print("Post Effects: ",flag,env.task.check_success())

            except Exception as e:
                rst["unknown_execution_error"]=True
                print("Execution Error:",e)
            print("************************************************")

        rst["all_condition_satisfied"]=env.task.check_success()[0]
        print(rst)
        
    with open(rst_path, 'w') as t_f:
        t_f.write(f.getvalue())

    return rst
    
if __name__ == "__main__":
    fire.Fire(evaluate_action_seqeunce)
"""
python D:\GitHub\behavior-vllm-eval\igibson\transition_model_v2\scripts\evaluate_action_sequence.py "D:\GitHub\behavior-vllm-eval\igibson\data\virtual_reality\bottling_fruit_0_Wainscott_0_int_0_2021-05-24_19-46-46.hdf5" "D:\GitHub\behavior-vllm-eval\igibson\transition_model_v2\data\annotations\bottling_fruit_0_Wainscott_0_int_0_2021-05-24_19-46-46.json" "test.log"
"""

