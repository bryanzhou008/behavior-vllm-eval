import os
from igibson.transition_model_v0.behavior_eval_env import BehaviorEvalEnv
import json
from igibson.transition_model_v0.action_utils import *

def main():
    demo_name="bottling_fruit_0_Wainscott_0_int_0_2021-05-24_19-46-46"
    demo_path=os.path.join(r"D:\GitHub\behavior-vllm-eval\igibson\data\virtual_reality", demo_name+".hdf5")
    env=BehaviorEvalEnv(demo_path)
    actions=[

        {'action': 'OPEN', 'object': 'fridge_97'},
        {'action': 'RIGHT_GRASP', 'object': 'strawberry_0'},
        {'action': 'LEFT_GRASP', 'object': 'peach_0'},
        {'action': 'LEFT_RELEASE', 'object': 'peach_0'},
        {'action': 'RIGHT_RELEASE', 'object': 'strawberry_0'},
        {'action': 'RIGHT_GRASP', 'object': 'carving_knife_0'},
        {'action': 'SLICE', 'object': 'strawberry_0'},
        {'action': 'SLICE', 'object': 'peach_0_multiplexer'},
        {'action': 'RIGHT_RELEASE', 'object': 'carving_knife_0'},
        # {'action': 'RIGHT_GRASP', 'object': 'jar_1'},
        {'action': 'OPEN', 'object': 'jar_1'},
        {'action': 'LEFT_GRASP', 'object': 'strawberry_0_part_0'},
        {'action': 'LEFT_PLACE_INSIDE', 'object': 'jar_1'},
        {'action': 'LEFT_GRASP', 'object': 'strawberry_0_part_1'},
        {'action': 'LEFT_PLACE_INSIDE', 'object': 'jar_1'},
        {'action': 'CLOSE', 'object': 'jar_1'},
        # {'action': 'RIGHT_RELEASE', 'object': 'jar_0'},
        # {'action': 'RIGHT_GRASP', 'object': 'jar_0'},
        {'action': 'OPEN', 'object': 'jar_0'},
        {'action': 'LEFT_GRASP', 'object': 'peach_0_part_0'},
        {'action': 'LEFT_PLACE_INSIDE', 'object': 'jar_0'},
        {'action': 'LEFT_GRASP', 'object': 'peach_0_part_1'},
        {'action': 'LEFT_PLACE_INSIDE', 'object': 'jar_0'},
        {'action': 'CLOSE', 'object': 'jar_0'},
        # {'action': 'RIGHT_RELEASE', 'object': 'jar_0'},


        ]
    for action in actions:
        action_execuble=(action['action'], action['object'])
        env.step(action_execuble)
    

if __name__ == "__main__":
    main()