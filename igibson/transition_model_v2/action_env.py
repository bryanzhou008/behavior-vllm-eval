import gym
import numpy as np
from igibson.action_primitives.starter_semantic_action_primitives import ActionPrimitiveError
from igibson.robots import BaseRobot
from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
import igibson.object_states as object_states
from igibson.simulator import Simulator
from igibson.objects.object_base import BaseObject
from igibson.objects.articulated_object import URDFObject
from collections import defaultdict
from pyquaternion import Quaternion
from .inside_tree import InsideTree

class ActionEnv:
    def __init__(
        self,simulator,scene,robot,addressable_objects
    ):
        self.simulator:Simulator = simulator
        self.scene: InteractiveIndoorScene = scene
        self.robot: BaseRobot = robot
        self.addressable_objects = addressable_objects
        self.robot_inventory = {'right_hand':None,'left_hand':None}
        self.inside_tree=InsideTree(self.addressable_objects) # to teleport inside relationship

    ##################### primitive actions #####################










    ##################### helper functions #####################
    def teleport_inside(self,obj1:URDFObject):
        if obj1 in self.record_inside and len(self.record_inside[obj1])>0:
            for obj2 in self.record_inside[obj1]:
                self.set_inside(obj2,obj1)
            self.update_inside_record()

    def set_inside(self,obj1:URDFObject,obj2:URDFObject):
        # obj1.set_position(obj2.get_position())
        obj1.state[object_states.Inside].set_value(obj2,True)
    
    def set_ontop(self,obj1:URDFObject,obj2:URDFObject):
        # target_pos = obj2.get_position()
        # target_pos[2] += 0.5 * obj1.bounding_box[2] + 0.5 * obj2.bounding_box[2]
        # obj1.set_position(target_pos)
        obj1.state[object_states.OnTop].set_value(obj2,True)

    def set_under(self,obj1:URDFObject,obj2:URDFObject):
        # target_pos = obj2.get_position()
        # target_pos[2] -= 0.5 * obj1.bounding_box[2] + 0.5 * obj2.bounding_box[2]
        # target_pos[2] = max(target_pos[2],0.5 * obj1.bounding_box[2])
        # obj1.set_position(target_pos)
        obj1.state[object_states.Under].set_value(obj2,True)

    def set_next_to(self,obj1:URDFObject,obj2:URDFObject):
        obj1.state[object_states.NextTo].set_value(obj2,True)

    def set_in_hand(self,obj:URDFObject,hand:str):
        weight=1 if hand=='right_hand' else -1
        target_pos = self.robot.get_position()
        target_pos[2] += self.robot.aabb_center[2]
        target_pos[2] +=0.2*weight
        obj.set_position(target_pos)

    def update_inventory(self):
        for hand,obj in self.robot_inventory.items():
            if obj is not None:
                self.set_in_hand(obj,hand)
                self.teleport_inside(obj)

    def navigate_to(self,obj:URDFObject):
        def get_robot_pos(obj:URDFObject):
            # get robot position according to object position
            obj_pos, obj_ori = obj.get_position_orientation()
            vec_standard = np.array([0, -1, 0])
            rotated_vec = Quaternion(obj_ori[[3, 0, 1, 2]]).rotate(vec_standard)
            bbox = obj.bounding_box
            robot_pos = np.zeros(3)
            robot_pos[0] = obj_pos[0] + rotated_vec[0] * bbox[1] * 0.5 + rotated_vec[0]
            robot_pos[1] = obj_pos[1] + rotated_vec[1] * bbox[1] * 0.5 + rotated_vec[1]
            robot_pos[2] = 0.25
    
            return robot_pos
        self.robot.set_position(get_robot_pos(obj))
        self.update_inventory()
        print(f"navigated to {obj.name}, InReachOfRobot: {obj.state[object_states.InReachOfRobot].get_value(self.robot)}")
    
    def navigate_if_needed(self,obj:URDFObject):
        if not obj.state[object_states.InReachOfRobot].get_value(self.robot):
            self.navigate_to(obj)

    def check_interactability(self,obj1):
        # currently just checking if object is inside a closed object or not
        for obj2 in self.record_inside.keys():
            if obj1 in self.record_inside[obj2] and object_states.Open in obj2.states and not obj2.states[object_states.Open].get_value():
                print(f"{obj1.name} is inside closed {obj2.name}, not interactable")
                return False
        return True
        
    
    ##################### primitive action #####################
    def grasp(self,obj:URDFObject,hand:str):
        ## pre conditions
        if not self.check_interactability(obj):
            return False
        if self.robot_inventory[hand] is not None:
            print(f"{hand} is already holding {self.robot_inventory[hand].name}")
            return False
        ## post effects
        self.navigate_if_needed(obj)
        self.robot_inventory[hand]=obj
        self.set_in_hand(obj,hand)
        self.teleport_inside(obj)
        return True
    
    def place_inside(self,obj:URDFObject,hand:str):
        ## pre conditions
        if not self.check_interactability(obj):
            return False
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        if object_states.Open in obj.states and not obj.states[object_states.Open].get_value():
            print(f"{obj.name} is closed, cannot place inside")
            return False
        ## post effects
        self.robot_inventory[hand]=None
        return True




    ##################### for behavior task eval #####################
    def left_grasp(self,obj):
        return self.grasp(obj,'left_hand')
    
    def right_grasp(self,obj):
        return self.grasp(obj,'right_hand')
    
    def left_place_ontop(self,obj):
        return self.place(obj,'left_hand','PLACE_ON_TOP')
    
    def right_place_ontop(self,obj):
        return self.place(obj,'right_hand','PLACE_ON_TOP')
    
    def left_place_inside(self,obj):
        return self.place(obj,'left_hand','PLACE_INSIDE')
    
    def right_place_inside(self,obj):
        return self.place(obj,'right_hand','PLACE_INSIDE')
    
    def right_release(self,obj):
        return self.release(obj,'right_hand')
    
    def left_release(self,obj):
        return self.release(obj,'left_hand')
    
    def open(self,obj):
        flag=self.open_close(obj,'right_hand','OPEN')
        if flag:
            return flag
        return self.open_close(obj,'left_hand','OPEN')
    
    def close(self,obj):
        flag=self.open_close(obj,'right_hand','CLOSE')
        if flag:
            return flag
        return self.open_close(obj,'left_hand','CLOSE')()
    
    
        



        

        
           



    
        