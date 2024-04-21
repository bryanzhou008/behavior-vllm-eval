from enum import IntEnum
from .action_env import ActionEnv
from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.ig_logging import IGLogReader
import os
import igibson
from igibson.utils.utils import parse_config
from igibson.tasks.behavior_task import BehaviorTask
from igibson.objects.multi_object_wrappers import ObjectMultiplexer,ObjectGrouper
from igibson.objects.articulated_object import URDFObject
from igibson.robots import BaseRobot
import gym
from igibson.object_states.on_floor import RoomFloor

TASK_RELEVANT_OBJECTS_ONLY = True

class EvalActions(IntEnum):
    NAVIGATE_TO = 0
    LEFT_GRASP = 1
    RIGHT_GRASP = 2
    LEFT_PLACE_ONTOP = 3
    RIGHT_PLACE_ONTOP = 4
    LEFT_PLACE_INSIDE = 5
    RIGHT_PLACE_INSIDE = 6
    RIGHT_RELEASE = 10
    LEFT_RELEASE = 11
    PLACE_ON_TOP = 12
    PLACE_INSIDE = 13
    OPEN = 14
    CLOSE = 15
    BURN = 16
    COOK = 17
    CLEAN = 18
    FREEZE = 19
    UNFREEZE = 20
    SLICE = 21
    SOAK = 22
    DRY = 23
    STAIN = 24
    TOGGLE_ON = 25
    TOGGLE_OFF = 26
    UNCLEAN = 27
    UNSOAK = 28

class EvalEnv:

    def defalt_init(self,demo_path):
        task = IGLogReader.read_metadata_attr(demo_path, "/metadata/atus_activity")
        if task is None:
            task = IGLogReader.read_metadata_attr(demo_path, "/metadata/task_name")

        task_id = IGLogReader.read_metadata_attr(demo_path, "/metadata/activity_definition")
        if task_id is None:
            task_id = IGLogReader.read_metadata_attr(demo_path, "/metadata/task_instance")

        scene_id = IGLogReader.read_metadata_attr(demo_path, "/metadata/scene_id")

        config_filename = os.path.join(igibson.configs_path, "behavior_robot_mp_behavior_task.yaml")
        config = parse_config(config_filename)
        

        config["task"] = task
        config["task_id"] = task_id
        config["scene_id"] = scene_id
        config["robot"]["show_visual_head"] = True
        config["image_width"]=512
        config["image_height"]=512
        self.config = config
    
    def __init__(self,config=None,demo_path=None,**kwargs) -> None:
        assert config is not None or demo_path is not None
        self.config=config
        if demo_path is not None:
            self.defalt_init(demo_path)
        self.env = iGibsonEnv(config_file=self.config,**kwargs)
        self.robot=self.robots[0]
        self.simulator=self.env.simulator
        self.get_relevant_objects()
        self.action_env=ActionEnv(self.simulator,self.scene,self.robot,self.addressable_objects)
        self.control_function={
            EvalActions.NAVIGATE_TO.value: self.action_env.navigate,
            EvalActions.LEFT_GRASP.value: self.action_env.left_grasp,
            EvalActions.RIGHT_GRASP.value: self.action_env.right_grasp,
            EvalActions.LEFT_PLACE_ONTOP.value: self.action_env.left_place_ontop,
            EvalActions.RIGHT_PLACE_ONTOP.value: self.action_env.right_place_ontop,
            EvalActions.LEFT_PLACE_INSIDE.value: self.action_env.left_place_inside,
            EvalActions.RIGHT_PLACE_INSIDE.value: self.action_env.right_place_inside,
            EvalActions.RIGHT_RELEASE.value: self.action_env.right_release,
            EvalActions.LEFT_RELEASE.value: self.action_env.left_release,
            EvalActions.OPEN.value: self.action_env.open,
            EvalActions.CLOSE.value: self.action_env.close,
            EvalActions.CLEAN.value: self.action_env.clean_dust,
            EvalActions.SLICE.value: self.action_env.slice,
        }

    def get_relevant_objects(self):

        if TASK_RELEVANT_OBJECTS_ONLY and isinstance(self.task, BehaviorTask):
            self.addressable_objects = set([
                item
                for item in self.task.object_scope.values()
                if isinstance(item, URDFObject) or isinstance(item, RoomFloor) or isinstance(item, ObjectMultiplexer)
            ])
        else:
            self.addressable_objects = set(self.scene.objects_by_name.values())
            if isinstance(self.task, BehaviorTask):
                self.addressable_objects.update(self.task.object_scope.values())

        obj_in_multiplexer = set()
        #deal with multiplexed objects
        for obj in self.addressable_objects:
           if isinstance(obj, ObjectMultiplexer):
               for sub_obj in obj._multiplexed_objects:
                   if isinstance(sub_obj, URDFObject):
                        obj_in_multiplexer.add(sub_obj)
                   elif isinstance(sub_obj,ObjectGrouper):
                        for sub_sub_obj in sub_obj.objects:
                            if isinstance(sub_sub_obj, URDFObject):
                                obj_in_multiplexer.add(sub_sub_obj)
        self.addressable_objects.update(obj_in_multiplexer)
        self.addressable_objects = list(self.addressable_objects)
        # Filter out the robots.
        self.addressable_objects = [obj for obj in self.addressable_objects if not isinstance(obj, BaseRobot)]
        self.obj_name_to_obj = {obj.name: obj for obj in self.addressable_objects}
        self.obj_name_to_idx = {obj.name: idx for idx, obj in enumerate(self.addressable_objects)}

    def get_action_space(self):
        self.num_objects = len(self.addressable_objects)
        return gym.spaces.Tuple(
            [gym.spaces.Discrete(len(EvalActions)),gym.spaces.Discrete(len(self.addressable_objects))]
        )
    
    def step(self,action):
        action_idx,object_idx=action
        if isinstance(action_idx,str):
            action_idx=EvalActions[action_idx].value
        if isinstance(object_idx,str):
            object_idx=self.obj_name_to_idx[object_idx]
        self.control_function[action_idx](self.addressable_objects[object_idx])
        self.simulator.step()
        obs, reward, done, info = self.env.step(None)
        return obs, reward, done, info

    @property
    def scene(self):
        return self.env.scene

    @property
    def task(self):
        return self.env.task

    @property
    def robots(self):
        return self.env.robots
    
        

    
