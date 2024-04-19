import gym

from igibson.action_primitives.starter_semantic_action_primitives import ActionPrimitiveError
from igibson.eval_gibson.action_primitive_wrapper import SemanticActionPrimitivesWrapper
from igibson.robots import BaseRobot
from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
import igibson.object_states as object_states
from igibson.simulator import Simulator
MAX_ATTEMPTS = 5

class ActionEnv:
    def __init__(
        self,simulator,scene,robot
    ):
        self.simulator:Simulator = simulator
        self.scene: InteractiveIndoorScene = scene
        self.robot: BaseRobot = robot
        self.right_controller= SemanticActionPrimitivesWrapper(None,self.scene,self.robot,arm='right_hand')
        self.left_controller= SemanticActionPrimitivesWrapper(None,self.scene,self.robot,arm='left_hand')
        self.controller={
            'right_hand':self.right_controller,
            'left_hand':self.left_controller
        }
        self.inventory = {'right_hand':None,'left_hand':None}

    def execute_controller(self,ctrl_gen):
        for action in ctrl_gen:
            self.robot.apply_action(action)
            self.simulator.step()

    def grasp_motion_primitive(self,obj,hand):
        ctrl_gen = self.controller[hand].grasp(obj)
        if self.inventory[hand] is not None:
            print(f"Hand {hand} is already holding an object {self.inventory[hand]}.")
            return False
        for i in range(MAX_ATTEMPTS):
            try:
                print(f"Trying to GRASP {obj.name} with {hand}.")
                self.execute_controller(ctrl_gen)
                print(f"GRASP {obj.name} with {hand} succeeded!")
                self.inventory[hand] = obj
                return True
            except ActionPrimitiveError:
                print(f"Attempt {i + 1} to grasp {obj.name} with {hand} failed. Retry until {MAX_ATTEMPTS}.")
                continue
        print(f"Failed to grasp {obj} with {obj.name} after {MAX_ATTEMPTS} attempts.")
        return False
    
    def place_motion_primitive(self,obj,hand,motion_name):
        assert motion_name in ["PLACE_ON_TOP","PLACE_INSIDE"]

        if motion_name == "PLACE_INSIDE":
            ctrl_gen = self.controller[hand].place_inside(obj)
        else:
            ctrl_gen = self.controller[hand].place_on_top(obj)

        if self.inventory[hand] is None:
            print(f"Hand {hand} is not holding any object.")
            return False
        obj_in_hand = self.inventory[hand]
        for i in range(MAX_ATTEMPTS):
            try:
                print(f"Trying to {motion_name} {obj_in_hand.name} {obj.name} with {hand}.")
                self.execute_controller(ctrl_gen)
                print(f"{motion_name} {obj_in_hand.name} {obj.name} with {hand} succeeded!")
                self.inventory[hand] = None
                return True
            except ActionPrimitiveError:
                print(f"Attempt {i + 1} to {motion_name} {obj_in_hand.name} {obj.name} with {hand} failed. Retry until {MAX_ATTEMPTS}.")
                continue
        print(f"Failed to place {motion_name} {obj_in_hand.name} {obj.name} with {hand} after {MAX_ATTEMPTS} attempts.")
        return False
    
    def toggle_motion_primitive(self,obj,hand,motion_name):

        assert motion_name in ["TOGGLE_ON","TOGGLE_OFF"]

        if self.inventory[hand] is not None:
            print(f"Hand {hand} is holding an object {self.inventory[hand]}.")
            return False

        if motion_name == "TOGGLE_OFF":
            ctrl_gen = self.controller[hand].toggle_off(obj)
        else:
            ctrl_gen = self.controller[hand].toggle_on(obj)

        for i in range(MAX_ATTEMPTS):
            try:
                print(f"Trying to {motion_name} {obj.name}.")
                self.execute_controller(ctrl_gen)
                print(f"{motion_name} {obj.name} succeeded!")
                return True
            except ActionPrimitiveError:
                print(f"Attempt {i + 1} to {motion_name} {obj.name} failed. Retry until {MAX_ATTEMPTS}.")
                continue
        print(f"Failed to {motion_name} {obj.name} after {MAX_ATTEMPTS} attempts.")
        return False
    
    def navigate_motion_primitive(self,obj):
        ctrl_gen = self.controller['right_hand']._navigate_to_obj(obj)
        for i in range(MAX_ATTEMPTS):
            try:
                print(f"Trying to NAVIGATE_TO {obj.name}.")
                self.execute_controller(ctrl_gen)
                print(f"NAVIGATE_TO {obj.name} succeeded!")
                return True
            except ActionPrimitiveError:
                print(f"Attempt {i + 1} to navigate to {obj.name} failed. Retry until {MAX_ATTEMPTS}.")
                continue
        print(f"Failed to navigate to {obj.name} after {MAX_ATTEMPTS} attempts.")
        return False
    
    def open_close_motion_primitive(self,obj,hand,motion_name):
        assert motion_name in ["OPEN","CLOSE"]
        if self.inventory[hand] is not None:
            print(f"Hand {hand} is holding an object {self.inventory[hand]}.")
            return False
        ctrl_gen = self.controller[hand].open(obj) if motion_name == "OPEN" else self.controller[hand].close(obj)
        for i in range(MAX_ATTEMPTS):
            try:
                print(f"Trying to {motion_name} {obj.name}.")
                self.execute_controller(ctrl_gen)
                print(f"{motion_name} {obj.name} succeeded!")
                return True
            except ActionPrimitiveError:
                print(f"Attempt {i + 1} to {motion_name} {obj.name} failed. Retry until {MAX_ATTEMPTS}.")
                continue
        print(f"Failed to {motion_name} {obj.name} after {MAX_ATTEMPTS} attempts.")
        return False
    
    def release_motion_primitive(self,obj,hand):
        ctrl_gen = self.controller[hand]._execute_release()
        if self.inventory[hand] is None:
            print(f"Hand {hand} is not holding any object.")
            return False
        obj_in_hand = self.inventory[hand]
        for i in range(MAX_ATTEMPTS):
            try:
                print(f"Trying to RELEASE {obj_in_hand.name} with {hand}.")
                self.execute_controller(ctrl_gen)
                print(f"RELEASE {obj_in_hand.name} with {hand} succeeded!")
                self.inventory[hand] = None
                return True
            except ActionPrimitiveError:
                print(f"Attempt {i + 1} to release {obj_in_hand.name} with {hand} failed. Retry until {MAX_ATTEMPTS}.")
                continue
        print(f"Failed to release {obj_in_hand.name} with {hand} after {MAX_ATTEMPTS} attempts.")
        return False
    
    def grasp(self,obj,hand):
        if self.inventory[hand] is not None:
            print(f"Hand {hand} is already holding an object {self.inventory[hand]}.")
            return False
        return self.grasp_motion_primitive(obj,hand)
    
    def place(self,obj,hand,motion_name):
        assert motion_name in ["PLACE_ON_TOP","PLACE_INSIDE"]
        if self.inventory[hand] is None:
            print(f"Hand {hand} is not holding any object.")
            return False
        return self.place_motion_primitive(obj,hand,motion_name)
    
    def toggle(self,obj,hand,motion_name):
        assert motion_name in ["TOGGLE_ON","TOGGLE_OFF"]
        if self.inventory[hand] is not None:
            print(f"Hand {hand} is holding an object {self.inventory[hand]}.")
            return False
        return self.toggle_motion_primitive(obj,hand,motion_name)
    
    def navigate(self,obj):
        return self.navigate_motion_primitive(obj)
    
    def open_close(self,obj,hand,motion_name):
        assert motion_name in ["OPEN","CLOSE"]
        if self.inventory[hand] is not None:
            print(f"Hand {hand} is holding an object {self.inventory[hand]}.")
            return False
        return self.open_close_motion_primitive(obj,hand,motion_name)
    
    def release(self,obj,hand):
        if self.inventory[hand] is None:
            print(f"Hand {hand} is not holding any object.")
            return False
        return self.release_motion_primitive(obj,hand)
    
    # high level actions
    def slice(self,obj):
        flag=self.navigate(obj)
        if not flag:
            print(f"Slice failed, failed to navigate to {obj.name}.")
            return False
        
        if not (hasattr(obj, "states") and object_states.Sliced in obj.states):
            print("Slice failed, object cannot be sliced")
            return False
        
        if obj.states[object_states.Sliced].get_value():
            print("Slice failed, object is already sliced")
            return False
        
        has_slicer=False
        for inventory_obj in self.inventory.values():
            if hasattr(inventory_obj, "states") and object_states.Slicer in inventory_obj.states:
                has_slicer=True
                break
        if not has_slicer:
            print("Slice failed, no slicer in inventory")
            return False
        
        obj.states[object_states.Sliced].set_value(True)
        print(f"Slice {obj.name} success")
        return True
    
    def clean_dust(self,obj):
        flag=self.navigate(obj)
        if not flag:
            print(f"Clean failed, failed to navigate to {obj.name}.")
            return False
        
        if not (hasattr(obj, "states") and object_states.Dusty in obj.states):
            print("Clean failed, object cannot be cleaned")
            return False
        
        if not obj.states[object_states.Dusty].get_value():
            print("Clean failed, object is already cleaned")
            return False
        
        has_cleaner=False
        for inventory_obj in self.inventory.values():
            if hasattr(inventory_obj, "states") and object_states.CleaningTool in inventory_obj.states:
                has_cleaner=True
                break
        if not has_cleaner:
            print("Clean failed, no cleaner in inventory")
            return False
        
        obj.states[object_states.Dusty].set_value(False)
        print(f"Clean {obj.name} success")
        return True
    
    def clean_stain(self,obj):
        flag=self.navigate(obj)
        if not flag:
            print(f"Clean Stain failed, failed to navigate to {obj.name}.")
            return False
        
        if not (hasattr(obj, "states") and object_states.Stained in obj.states):
            print("Clean failed, object cannot be cleaned")
            return False
        
        if not obj.states[object_states.Stained].get_value():
            print("Clean failed, object is already cleaned")
            return False
        
        has_cleaner=False
        cleaner_soaked=False
        for inventory_obj in self.inventory.values():
            if hasattr(inventory_obj, "states") and object_states.CleaningTool in inventory_obj.states:
                has_cleaner=True
                if hasattr(inventory_obj, "states") and object_states.Soaked in inventory_obj.states and inventory_obj.states[object_states.Soaked].get_value():
                    cleaner_soaked=True
                    break
        if not has_cleaner:
            print("Clean failed, no cleaner in inventory")
            return False
        if not cleaner_soaked:
            print("Clean failed, cleaner is not soaked")
            return False
        obj.states[object_states.Stained].set_value(False)
        print(f"Clean {obj.name} success")
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
        return self.open_close(obj,'left_hand','CLOSE')
    
    
        



        

        
           



    
        