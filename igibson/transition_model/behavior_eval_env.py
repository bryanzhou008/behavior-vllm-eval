import argparse
import time
import gym.spaces
import numpy as np
import pybullet as p
from typing import Tuple
from igibson import object_states
from igibson.envs.behavior_env import BehaviorEnv
from igibson.external.pybullet_tools.utils import CIRCULAR_LIMITS
from igibson.object_states.on_floor import RoomFloor
from igibson.object_states.utils import sample_kinematics
from igibson.objects.articulated_object import URDFObject
from igibson.objects.multi_object_wrappers import ObjectMultiplexer,ObjectGrouper
from igibson.objects.object_base import BaseObject
from igibson.robots.behavior_robot import BRBody, BREye, BRHand
from igibson.utils.ig_logging import IGLogReader
from igibson.transition_model.actions_primitives import ActionPrimitives
from igibson.transition_model.action_execution import ActionExecution
import yaml
import igibson
import os

class BehaviorEvalEnv(BehaviorEnv):
    """
    iGibson Environment (OpenAI Gym interface)
    """

    def defalt_init(self,demo_path):
        task = IGLogReader.read_metadata_attr(demo_path, "/metadata/atus_activity")
        if task is None:
            task = IGLogReader.read_metadata_attr(demo_path, "/metadata/task_name")

        task_id = IGLogReader.read_metadata_attr(demo_path, "/metadata/activity_definition")
        if task_id is None:
            task_id = IGLogReader.read_metadata_attr(demo_path, "/metadata/task_instance")

        scene_id = IGLogReader.read_metadata_attr(demo_path, "/metadata/scene_id")

        config_file = os.path.join(igibson.example_config_path, "behavior_segmentation_replay.yaml")
        print("Loading config from", config_file)
        with open(config_file, "r") as f:
            config = yaml.safe_load(f)

        config["task"] = task
        config["task_id"] = task_id
        config["scene_id"] = scene_id
        self.config = config
        

    def __init__(
        self,
        demo_path=None,
        config_file=None,
        mode="iggui",
        action_timestep=1 / 10.0,
        physics_timestep=1 / 240.0,
        device_idx=0,
        render_to_tensor=False,
        automatic_reset=False,
        seed=0,
        action_filter="mobile_manipulation",
    ):
        """
        @param config_file: config_file path
        @param scene_id: override scene_id in config file
        @param mode: headless, simple, iggui
        @param action_timestep: environment executes action per action_timestep second
        @param physics_timestep: physics timestep for pybullet
        @param device_idx: which GPU to run the simulation and rendering on
        @param render_to_tensor: whether to render directly to pytorch tensors
        @param automatic_reset: whether to automatic reset after an episode finishes
        @param seed: RNG seed for sampling
        @param action_filter: see BehaviorEnv
        """
        if demo_path is None and config_file is None:
            raise ValueError("You must provide demo_path or config_file.")

        if demo_path is not None:
            self.defalt_init(demo_path)
            config_file=self.config

        super(BehaviorEvalEnv, self).__init__(
            config_file=config_file,
            mode=mode,
            action_timestep=action_timestep,
            physics_timestep=physics_timestep,
            device_idx=device_idx,
            render_to_tensor=render_to_tensor,
            action_filter=action_filter,
            seed=seed,
            automatic_reset=automatic_reset,
        )

        self.robots[0].initial_z_offset = 0.7
        self.action_execution = ActionExecution(self.scene, self.robots[0])
        self.control_function={
            ActionPrimitives.NAVIGATE_TO.value: self.action_execution.navigate_to,
            ActionPrimitives.LEFT_GRASP.value: self.action_execution.left_grasp,
            ActionPrimitives.RIGHT_GRASP.value: self.action_execution.right_grasp,
            ActionPrimitives.LEFT_PLACE_ONTOP.value: self.action_execution.left_place_ontop,
            ActionPrimitives.RIGHT_PLACE_ONTOP.value: self.action_execution.left_place_ontop,
            ActionPrimitives.LEFT_PLACE_INSIDE.value: self.action_execution.left_place_inside,
            ActionPrimitives.RIGHT_PLACE_INSIDE.value: self.action_execution.right_place_inside,
            ActionPrimitives.RIGHT_RELEASE.value: self.action_execution.right_release,
            ActionPrimitives.LEFT_RELEASE.value: self.action_execution.left_release,
            ActionPrimitives.PLACE_ON_TOP.value: self.action_execution.right_place_ontop,
            ActionPrimitives.PLACE_INSIDE.value: self.action_execution.right_place_inside,
            ActionPrimitives.OPEN.value: self.action_execution.open,
            ActionPrimitives.CLOSE.value: self.action_execution.close,
            ActionPrimitives.BURN.value: self.action_execution.burn,
            ActionPrimitives.COOK.value: self.action_execution.cook,
            ActionPrimitives.CLEAN.value: self.action_execution.clean,
            ActionPrimitives.SLICE.value: self.action_execution.slice,
            ActionPrimitives.SOAK.value: self.action_execution.soak,
            ActionPrimitives.FREEZE.value: self.action_execution.freeze,
            ActionPrimitives.UNFREEZE.value: self.action_execution.unfreeze,    
        }
        

    def load_action_space(self):
        
        self.addressable_objects = list(
            set(self.task.simulator.scene.objects_by_name.values()) | set(self.task.object_scope.values())
        )

        #Filter out the robots.
        self.addressable_objects = [obj for obj in self.addressable_objects if not isinstance(obj, type(self.robots[0]))]

        #deal with multiplexed objects
        for obj in self.addressable_objects:
           if isinstance(obj, ObjectMultiplexer):
               for sub_obj in obj._multiplexed_objects:
                   if isinstance(sub_obj, URDFObject):
                        self.addressable_objects.append(sub_obj)
                   elif isinstance(sub_obj,ObjectGrouper):
                        for sub_sub_obj in sub_obj.objects:
                            if isinstance(sub_sub_obj, URDFObject):
                                self.addressable_objects.append(sub_sub_obj)


        self.obj_name_to_id = {obj.name: idx for idx,obj in enumerate(self.addressable_objects)}
        self.obj_name_to_obj = {obj.name: obj for obj in self.addressable_objects}
        self.num_objects = len(self.addressable_objects)
        self.action_space = gym.spaces.Tuple(
            [gym.spaces.Discrete(len(ActionPrimitives)),gym.spaces.Discrete(self.num_objects)]
        )
    
    def get_body_ids(self, include_self=False):
        ids = []
        for object in self.scene.get_objects():
            if isinstance(object, URDFObject):
                ids.extend(object.body_ids)

        if include_self:
            ids.append(self.robots[0].parts["left_hand"].get_body_id())
            ids.append(self.robots[0].parts["body"].get_body_id())

        return ids

    def step(self, action):
        action_idx, obj_idx = action
        if isinstance(action_idx, str):
            action_idx=ActionPrimitives[action_idx].value
        if isinstance(obj_idx, str):
            obj_idx=self.obj_name_to_id[obj_idx]

        obj = self.addressable_objects[obj_idx]
        if not (isinstance(obj, BRBody) or isinstance(obj, BRHand) or isinstance(obj, BREye)):
            self.control_function[action_idx](obj)
        else:
            print(f"Invalid action: {ActionPrimitives[action_idx].name} on {obj.name}")
        state, reward, done, info = super(BehaviorEvalEnv, self).step(np.zeros(17))
        print("PRIMITIVE satisfied predicates:", info["satisfied_predicates"])
        return state, reward, done, info

    def reset(self):
        obs = super(BehaviorEvalEnv, self).reset()
        for hand in ["left_hand", "right_hand"]:
            self.robots[0].parts[hand].set_close_fraction(0)
            self.robots[0].parts[hand].trigger_fraction = 0
            self.robots[0].parts[hand].force_release_obj()
        return obs



    




