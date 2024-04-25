import gym
import numpy as np
from igibson.action_primitives.starter_semantic_action_primitives import ActionPrimitiveError
from igibson.robots import BaseRobot,BehaviorRobot
from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
import igibson.object_states as object_states
from igibson.simulator import Simulator
from igibson.objects.object_base import BaseObject
from igibson.objects.articulated_object import URDFObject
from collections import defaultdict
from pyquaternion import Quaternion
from .relation_tree import IgibsonRelationTree
from .position_geometry import PositionGeometry
from .relation_tree import TeleportType
class ActionEnv:
    def __init__(
        self,simulator:Simulator,
        scene: InteractiveIndoorScene,
        robot: BehaviorRobot,
        addressable_objects:list,
        using_kinematics=False
    ):
        self.simulator = simulator
        self.scene = scene
        self.robot = robot
        self.addressable_objects = addressable_objects
        self.robot_inventory = {'right_hand':None,'left_hand':None}
        self.relation_tree=IgibsonRelationTree(self.addressable_objects) # to teleport relationship
        self.position_geometry=PositionGeometry(self.robot,using_kinematics)

    ##################### primitive actions #####################

    def navigate_to(self,obj:URDFObject):
        ## pre conditions   
        ## currently do auto navigation, no need for pre conditions
        ## post effects
        if obj.states[object_states.InReachOfRobot].get_value():
            return True
        
        self.position_geometry.set_robot_pos_for_obj(obj)
        for hand,invent_obj in self.robot_inventory.items():
            if invent_obj is not None:
                self.position_geometry.set_in_hand(invent_obj,hand)
                self.teleport_relation(invent_obj)

        if obj.states[object_states.InReachOfRobot].get_value():
            print(f"navigated to {obj.name}, InReachOfRobot: {obj.states[object_states.InReachOfRobot].get_value()}")
            return True
        
    def grasp(self,obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        if self.robot_inventory[hand] is not None:
            print(f"{hand} is already holding {self.robot_inventory[hand].name}")
            return False
        # precondition for fixed body or volume?
        ## post effects
        self.navigate_to_if_needed(obj)
        self.relation_tree.remove_ancestor(obj)
        self.position_geometry.set_in_hand(obj,hand)
        self.robot_inventory[hand]=obj
        self.teleport_relation(obj)
        print(f"Grasp {obj.name} with {hand} successful")
        return True
    
    def release(self,hand:str,obj=None):
        ## pre conditions
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        if obj is not None and self.robot_inventory[hand]!=obj:
            print(f"{obj.name} is not in {hand}")
            return False
        ## post effects
        obj=self.robot_inventory[hand]
        self.robot_inventory[hand]=None
        self.position_geometry.release_obj(obj)
        self.teleport_relation(obj)
        print(f"Release {obj.name} from {hand} successful")
        return True
    
    def place_inside(self,tar_obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(tar_obj)
        except ValueError as e:
            print(e)
            return False
        
        if object_states.Open in tar_obj.states and not tar_obj.states[object_states.Open].get_value():
            print(f"{tar_obj.name} is closed")
            return False
        
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        
        obj_in_hand=self.robot_inventory[hand]
        if obj_in_hand==tar_obj:
            print(f"{tar_obj.name} is already in {hand}")
            return False
        
        if obj_in_hand.bounding_box[0]*obj_in_hand.bounding_box[1]*obj_in_hand.bounding_box[2]> \
        tar_obj.bounding_box[0]* tar_obj.bounding_box[1] * tar_obj.bounding_box[2]:
            print(f"{obj_in_hand.name} is bigger than {tar_obj.name}")
            return False
        
        ## post effects
        self.navigate_to_if_needed(tar_obj)
        self.relation_tree.change_ancestor(obj_in_hand,tar_obj,TeleportType.INSIDE)
        self.position_geometry.set_inside(obj_in_hand,tar_obj)
        self.teleport_relation(obj_in_hand)
        self.robot_inventory[hand]=None
        if obj_in_hand.states[object_states.Inside].get_value(tar_obj):
            print(f"Place inside {obj_in_hand.name} {tar_obj.name} successful")
            return True
        else:
            print(f"Place inside {obj_in_hand.name} {tar_obj.name} unsuccessful")
            return False
        
    def place_ontop(self,obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        
        obj_in_hand=self.robot_inventory[hand]
        if obj_in_hand==obj:
            print(f"{obj.name} is already in {hand}")
            return False
        elif obj in self.robot_inventory.values():
            print(f"Release {obj.name} first to place {obj_in_hand.name} on top of it")
            return False
        
        ## post effects
        self.navigate_to_if_needed(obj)
        self.relation_tree.change_ancestor(obj_in_hand,obj,TeleportType.ONTOP)
        self.position_geometry.set_ontop(obj_in_hand,obj)
        self.teleport_relation(obj_in_hand)
        self.robot_inventory[hand]=None
        if obj_in_hand.states[object_states.OnTop].get_value(obj):
            print(f"Place ontop {obj_in_hand.name} {obj.name} successful")
            return True
        else:
            print(f"Place ontop {obj_in_hand.name} {obj.name} unsuccessful")
            return False
        
    def place_under(self,tar_obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(tar_obj)
        except ValueError as e:
            print(e)
            return False
        
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        
        obj_in_hand=self.robot_inventory[hand]
        if obj_in_hand==tar_obj:
            print(f"{tar_obj.name} is already in {hand}")
            return False
        elif tar_obj in self.robot_inventory.values():
            print(f"Release {tar_obj.name} first to place {obj_in_hand.name} under it")
            return False
        
        ## post effects
        self.navigate_to_if_needed(tar_obj)
        self.relation_tree.remove_ancestor(obj_in_hand)
        self.position_geometry.set_under(obj_in_hand,tar_obj)
        self.teleport_relation(obj_in_hand)
        self.robot_inventory[hand]=None
        if obj_in_hand.states[object_states.Under].get_value(tar_obj):
            print(f"Place under {obj_in_hand.name} {tar_obj.name} successful")
            return True
        else:
            print(f"Place under {obj_in_hand.name} {tar_obj.name} unsuccessful")
            return False
        
    def place_next_to(self,tar_obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(tar_obj)
        except ValueError as e:
            print(e)
            return False
        
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        
        obj_in_hand=self.robot_inventory[hand]
        if obj_in_hand==tar_obj:
            print(f"{tar_obj.name} is already in {hand}")
            return False
        elif tar_obj in self.robot_inventory.values():
            print(f"Release {tar_obj.name} first to place {obj_in_hand.name} next to it")
            return False
        
        ## post effects
        self.navigate_to_if_needed(tar_obj)
        self.relation_tree.remove_ancestor(obj_in_hand)
        self.position_geometry.set_next_to(obj_in_hand,tar_obj)
        self.teleport_relation(obj_in_hand)
        self.robot_inventory[hand]=None
        if obj_in_hand.states[object_states.NextTo].get_value(tar_obj):
            print(f"Place under {obj_in_hand.name} {tar_obj.name} successful")
            return True
        else:
            print(f"Place under {obj_in_hand.name} {tar_obj.name} unsuccessful")
            return False
        
    def pour_inside(self,tar_obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(tar_obj)
        except ValueError as e:
            print(e)
            return False
        
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        
        obj_in_hand=self.robot_inventory[hand]
        if obj_in_hand==tar_obj:
            print(f"{tar_obj.name} is already in {hand}")
            return False
        elif tar_obj in self.robot_inventory.values():
            print(f"Release {tar_obj.name} first to pour contents in {obj_in_hand.name} inside it")
            return False
        
        if len(self.relation_tree.get_node(obj_in_hand).children)==0:
            print(f"{obj_in_hand.name} is empty, nothing to pour inside {tar_obj.name}")
            return False
        
        for obj_inside in self.relation_tree.get_node(obj_in_hand).children.keys():
            if obj_inside.bounding_box[0]*obj_inside.bounding_box[1]*obj_inside.bounding_box[2]> \
            tar_obj.bounding_box[0]* tar_obj.bounding_box[1] * tar_obj.bounding_box[2]:
                print(f"{obj_inside.name} is bigger than {tar_obj.name}, cannot pour inside")
                return False
        
        ## post effects
        self.navigate_to_if_needed(tar_obj)
        for obj_inside in self.relation_tree.get_node(obj_in_hand).children.keys():
            self.relation_tree.change_ancestor(obj_inside,tar_obj,TeleportType.INSIDE)
            self.position_geometry.set_inside(obj_inside,tar_obj)
            self.teleport_relation(obj_inside)
        print(f"Pour inside {obj_in_hand.name} {tar_obj.name} successful")
        return True
    
    def pour_onto(self,tar_obj:URDFObject,hand:str):
        ## pre conditions
        try:
            self.check_interactability(tar_obj)
        except ValueError as e:
            print(e)
            return False
        
        if self.robot_inventory[hand] is None:
            print(f"{hand} is empty")
            return False
        
        obj_in_hand=self.robot_inventory[hand]
        if obj_in_hand==tar_obj:
            print(f"{tar_obj.name} is already in {hand}")
            return False
        elif tar_obj in self.robot_inventory.values():
            print(f"Release {tar_obj.name} first to pour contents in {obj_in_hand.name} onto it")
            return False
        
        if len(self.relation_tree.get_node(obj_in_hand).children)==0:
            print(f"{obj_in_hand.name} is empty, nothing to pour onto {tar_obj.name}")
            return False
        
        ## post effects
        self.navigate_to_if_needed(tar_obj)
        for obj_inside in self.relation_tree.get_node(obj_in_hand).children.keys():
            self.relation_tree.change_ancestor(obj_inside,tar_obj,TeleportType.ONTOP)
            self.position_geometry.set_ontop(obj_inside,tar_obj)
            self.teleport_relation(obj_inside)
        print(f"Pour onto {obj_in_hand.name} {tar_obj.name} successful")
        return True
    
    ##################### high level actions #####################
    def open_or_close(self,obj:URDFObject,open_close:str):
        assert open_close in ['open','close']
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if object_states.Open not in obj.states:
            print(f"{obj.name} cannot be {open_close}ed")
            return False
        
        if obj.states[object_states.Open].get_value()==(open_close=='open'):
            print(f"{obj.name} is already {open_close}ed")
            return False
        
        if None not in self.robot_inventory.values():
            print(f"Both hands full, release one object first to {open_close} the object")
            return False

        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.Open].set_value((open_close=='open'))
        print(f"{open_close.capitalize()} {obj.name} success")
        return True
    
    def toggle_on_off(self,obj:URDFObject,on_off:str):
        assert on_off in ['on','off']
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if object_states.ToggledOn not in obj.states:
            print(f"{obj.name} cannot be toggled {on_off}")
            return False
        
        if obj.states[object_states.ToggledOn].get_value()==(on_off=='on'):
            print(f"{obj.name} is already toggled {on_off}")
            return False
        
        if None not in self.robot_inventory.values():
            print(f"Both hands full, release one object first to toggle {on_off} the object")
            return False

        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.ToggledOn].set_value((on_off=='on'))
        print(f"Toggle{on_off} {obj.name} success")
        return True

    def slice(self,obj):
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if not (hasattr(obj, "states") and object_states.Sliced in obj.states):
            print("Slice failed, object cannot be sliced")
            return False
        
        if obj.states[object_states.Sliced].get_value():
            print("Slice failed, object is already sliced")
            return False
        
        has_slicer=False
        for inventory_obj in self.robot_inventory.values():
            if hasattr(inventory_obj, "states") and object_states.Slicer in inventory_obj.states:
                has_slicer=True
                break
        if not has_slicer:
            print("Slice failed, no slicer in inventory")
            return False
        
        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.Sliced].set_value(True)
        print(f"Slice {obj.name} success")
        return True
    
    def clean_dust(self,obj):
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        
        if not (hasattr(obj, "states") and object_states.Dusty in obj.states):
            print("Clean failed, object cannot be cleaned")
            return False
        
        if not obj.states[object_states.Dusty].get_value():
            print("Clean failed, object is already cleaned")
            return False
        
        has_cleaner=False
        for inventory_obj in self.robot_inventory.values():
            if hasattr(inventory_obj, "states") and object_states.CleaningTool in inventory_obj.states:
                has_cleaner=True
                break
        if not has_cleaner:
            print("Clean failed, no cleaner in inventory")
            return False
        
        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.Dusty].set_value(False)
        print(f"Clean-dust {obj.name} success")
        return True
    
    def clean_stain(self,obj):
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if not (hasattr(obj, "states") and object_states.Stained in obj.states):
            print("Clean failed, object cannot be cleaned")
            return False
        
        if not obj.states[object_states.Stained].get_value():
            print("Clean failed, object is already cleaned")
            return False
        
        has_cleaner=False
        cleaner_soaked=False
        for inventory_obj in self.robot_inventory.values():
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
        
        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.Stained].set_value(False)
        print(f"Clean-stain {obj.name} success")
        return True
    
    def soak_dry(self,obj,soak_or_dry:str):
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if not (hasattr(obj, "states") and object_states.Soaked in obj.states):
            print("Soak failed, object cannot be soaked")
            return False
        
        if obj.states[object_states.Soaked].get_value()==(soak_or_dry=='soak'):
            print(f"{soak_or_dry.capitalize()} failed, object is already {soak_or_dry}ed")
            return False
        
        # has_soaker=False
        # for inventory_obj in self.inventory.values():
        #     if hasattr(inventory_obj, "states") and object_states.Soaker in inventory_obj.states:
        #         has_soaker=True
        #         break
        # if not has_soaker:
        #     print("Soak failed, no soaker in inventory")
        #     return False
        
        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.Soaked].set_value((soak_or_dry=='soak'))
        print(f"{soak_or_dry.capitalize()} {obj.name} success")
        return True
    
    def freeze_unfreeze(self,obj,freeze_or_unfreeze:str):
        assert freeze_or_unfreeze in ['freeze','unfreeze']
        ## pre conditions
        try:
            self.check_interactability(obj)
        except ValueError as e:
            print(e)
            return False
        
        if not (hasattr(obj, "states") and object_states.Frozen in obj.states):
            print("Freeze failed, object cannot be frozen")
            return False
        
        if obj.states[object_states.Frozen].get_value()==(freeze_or_unfreeze=='freeze'):
            print(f"{freeze_or_unfreeze.capitalize()} failed, object is already {freeze_or_unfreeze}ed")
            return False
        
        # has_freezer=False
        # for inventory_obj in self.inventory.values():
        #     if hasattr(inventory_obj, "states") and object_states.Freezer in inventory_obj.states:
        #         has_freezer=True
        #         break
        # if not has_freezer:
        #     print("Freeze failed, no freezer in inventory")
        #     return False
        
        ## post effects
        self.navigate_to_if_needed(obj)
        obj.states[object_states.Frozen].set_value((freeze_or_unfreeze=='freeze'))
        print(f"{freeze_or_unfreeze.capitalize()} {obj.name} success")
        return True
        

    ##################### helper functions #####################
    def navigate_to_if_needed(self,obj:URDFObject):
        if not obj.states[object_states.InReachOfRobot].get_value():
            self.navigate_to(obj)

    def teleport_relation(self,obj1:URDFObject):
        teleport_func={
            TeleportType.INSIDE:self.position_geometry.set_inside,
            TeleportType.ONTOP:self.position_geometry.set_ontop,
        }
        obj_node=self.relation_tree.get_node(obj1)
        def recursive_teleport(node):
            for child in node.children.values():
                teleport_func[child.teleport_type](child.obj,node.obj)
                recursive_teleport(child)
        recursive_teleport(obj_node)

    def teleport_all(self):
        for obj in self.relation_tree.root.children.keys():
            self.teleport_relation(obj)

    def check_interactability(self,obj1):
        # currently just checking if object is inside a closed object or not
        node=self.relation_tree.get_node(obj1)
        node=node.parent
        while node is not self.relation_tree.root:
            obj=node.obj
            if (object_states.Open in obj.states and 
            not obj.states[object_states.Open].get_value() and
            node.teleport_type==TeleportType.INSIDE):
                raise ValueError(f"{obj1.name} is inside closed {obj.name}")
            node=node.parent
    ##################### for behavior task eval #####################
    def navigate(self,obj:URDFObject):
        return self.navigate_to(obj)
    
    def left_grasp(self,obj:URDFObject):
        return self.grasp(obj,'left_hand')
    
    def right_grasp(self,obj:URDFObject):
        return self.grasp(obj,'right_hand')
    
    def left_release(self,obj:URDFObject):
        return self.release('left_hand',obj)
    
    def right_release(self,obj:URDFObject):
        return self.release('right_hand',obj)
    
    def left_place_ontop(self,obj:URDFObject):
        return self.place_ontop(obj,'left_hand')
    
    def right_place_ontop(self,obj:URDFObject):
        return self.place_ontop(obj,'right_hand')
    
    def left_place_inside(self,obj:URDFObject):
        return self.place_inside(obj,'left_hand')
    
    def right_place_inside(self,obj:URDFObject):
        return self.place_inside(obj,'right_hand')
    
    def open(self,obj:URDFObject):
        return self.open_or_close(obj,'open')
    
    def close(self,obj:URDFObject):
        return self.open_or_close(obj,'close')
    
    def left_place_nextto(self,obj:URDFObject):
        return self.place_next_to(obj,'left_hand')
    
    def right_place_nextto(self,obj:URDFObject):
        return self.place_next_to(obj,'right_hand')
    
    def left_transfer_contents_inside(self,obj:URDFObject):
        return self.pour_inside(obj,'left_hand')
    
    def right_transfer_contents_inside(self,obj:URDFObject):
        return self.pour_inside(obj,'right_hand')
    
    def left_transfer_contents_ontop(self,obj:URDFObject):
        return self.pour_onto(obj,'left_hand')
    
    def right_transfer_contents_ontop(self,obj:URDFObject):
        return self.pour_onto(obj,'right_hand')
    
    def soak(self,obj:URDFObject):
        return self.soak_dry(obj,'soak')
    
    def dry(self,obj:URDFObject):
        return self.soak_dry(obj,'dry')
    
    def freeze(self,obj:URDFObject):
        return self.freeze_unfreeze(obj,'freeze')
    
    def unfreeze(self,obj:URDFObject):
        return self.freeze_unfreeze(obj,'unfreeze')
    
    def toggle_on(self,obj:URDFObject):
        return self.toggle_on_off(obj,'on')
    
    def toggle_off(self,obj:URDFObject):
        return self.toggle_on_off(obj,'off')
    
    
    

    
        



        

        
           



    
        