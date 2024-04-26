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
from enum import Enum, unique,auto
TASK_RELEVANT_OBJECTS_ONLY = True


@unique
class EvalActions(IntEnum):
    NAVIGATE_TO=auto()
    LEFT_GRASP =auto()
    RIGHT_GRASP =auto()
    LEFT_PLACE_ONTOP =auto()
    RIGHT_PLACE_ONTOP =auto()
    LEFT_PLACE_INSIDE =auto()
    RIGHT_PLACE_INSIDE =auto()
    RIGHT_RELEASE =auto()
    LEFT_RELEASE =auto()
    OPEN =auto()
    CLOSE =auto()
    BURN =auto()
    COOK =auto()
    CLEAN =auto()
    FREEZE =auto()
    UNFREEZE =auto()
    SLICE =auto()
    SOAK =auto()
    DRY =auto()
    # STAIN =auto()
    TOGGLE_ON =auto()
    TOGGLE_OFF =auto()
    # UNCLEAN =auto()
    LEFT_PLACE_NEXTTO=auto()
    RIGHT_PLACE_NEXTTO=auto()
    LEFT_TANSFER_CONTENTS_INSIDE=auto()
    RIGHT_TANSFER_CONTENTS_INSIDE=auto()
    LEFT_TANSFER_CONTENTS_ONTOP=auto()
    RIGHT_TANSFER_CONTENTS_ONTOP=auto()

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
            EvalActions.LEFT_PLACE_NEXTTO.value: self.action_env.left_place_nextto,
            EvalActions.RIGHT_PLACE_NEXTTO.value: self.action_env.right_place_nextto,
            EvalActions.LEFT_PLACE_INSIDE.value: self.action_env.left_place_inside,
            EvalActions.RIGHT_PLACE_INSIDE.value: self.action_env.right_place_inside,
            EvalActions.RIGHT_RELEASE.value: self.action_env.right_release,
            EvalActions.LEFT_RELEASE.value: self.action_env.left_release,
            EvalActions.OPEN.value: self.action_env.open,
            EvalActions.CLOSE.value: self.action_env.close,
            EvalActions.CLEAN.value: self.action_env.clean_dust,
            EvalActions.SLICE.value: self.action_env.slice,
            EvalActions.SOAK.value: self.action_env.soak,
            EvalActions.DRY.value: self.action_env.dry,
            EvalActions.FREEZE.value: self.action_env.freeze,
            EvalActions.UNFREEZE.value: self.action_env.unfreeze,
            EvalActions.LEFT_TANSFER_CONTENTS_INSIDE.value: self.action_env.left_transfer_contents_inside,
            EvalActions.RIGHT_TANSFER_CONTENTS_INSIDE.value: self.action_env.right_transfer_contents_inside,
            EvalActions.LEFT_TANSFER_CONTENTS_ONTOP.value: self.action_env.left_transfer_contents_ontop,
            EvalActions.RIGHT_TANSFER_CONTENTS_ONTOP.value: self.action_env.right_transfer_contents_ontop,
            EvalActions.TOGGLE_ON.value: self.action_env.toggle_on,
            EvalActions.TOGGLE_OFF.value: self.action_env.toggle_off,
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
        # Filter out the robots and ObjectMultiplexer.
        self.addressable_objects = [obj for obj in self.addressable_objects if not isinstance(obj, BaseRobot) and not isinstance(obj, ObjectMultiplexer)]
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
        flag=self.control_function[action_idx](self.addressable_objects[object_idx])
        #self.simulator.step()
        #obs, reward, done, info = self.env.step(None)
        return None, None, None, None,flag

    def final_step(self):
        self.action_env.teleport_all()
        self.simulator.step()
    @property
    def scene(self):
        return self.env.scene

    @property
    def task(self):
        return self.env.task

    @property
    def robots(self):
        return self.env.robots
    
        

    
