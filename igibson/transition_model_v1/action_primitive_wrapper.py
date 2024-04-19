import gym

from igibson.object_states.robot_related_states import InHandOfRobot
from igibson.objects.articulated_object import URDFObject
from igibson.robots import BaseRobot
from igibson.tasks.behavior_task import BehaviorTask
from igibson.action_primitives.starter_semantic_action_primitives import StarterSemanticActionPrimitives,StarterSemanticActionPrimitive
from igibson.objects.multi_object_wrappers import ObjectMultiplexer,ObjectGrouper
from igibson.object_states.on_floor import RoomFloor

class SemanticActionPrimitivesWrapper(StarterSemanticActionPrimitives):
    def __init__(self, task, scene, robot,arm='right_hand',task_relevant_objects_only=False):
        self.task_relevant_objects_only = task_relevant_objects_only
        self.arm = arm
        super().__init__(task, scene, robot)
        


    def get_action_space(self):
        
        if self.task_relevant_objects_only and isinstance(self.task, BehaviorTask):
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

        self.num_objects = len(self.addressable_objects)
        return gym.spaces.Tuple(
            [gym.spaces.Discrete(len(StarterSemanticActionPrimitive)),gym.spaces.Discrete(self.num_objects)]
        )
    
    def is_state_true(self,obj,state):
        if hasattr(obj, 'states') and state in obj.states and obj.states[state].get_value():
            return True
        return False

    def _get_obj_in_hand(self):
        obj_in_hand_id = self.robot._ag_obj_in_hand[self.arm] 
        obj_in_hand = self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None

        #deal with multiplexed objects
        if obj_in_hand is not None:
            if isinstance(obj_in_hand, ObjectMultiplexer):
                for sub_obj in obj_in_hand._multiplexed_objects:
                   if isinstance(sub_obj, URDFObject):
                        if self.is_state_true(sub_obj,InHandOfRobot):
                            obj_in_hand = sub_obj
                            return obj_in_hand
                   elif isinstance(sub_obj,ObjectGrouper):
                        for sub_sub_obj in sub_obj.objects:
                            if isinstance(sub_sub_obj, URDFObject):
                                if self.is_state_true(sub_obj,InHandOfRobot):
                                    obj_in_hand = sub_obj
                                    return obj_in_hand
        return obj_in_hand