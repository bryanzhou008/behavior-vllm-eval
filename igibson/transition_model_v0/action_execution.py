from .action_utils import grasp_primitive, place_inside_primitive,place_ontop_primitive,release_primitive,\
navigate_to_obj,navigate_if_needed,robot_invenvtory,get_obj_in_hand
from igibson.objects.articulated_object import URDFObject
from igibson import object_states
 
class ActionExecution:
    def __init__(self,scene,robot) -> None:
        self.scene=scene
        self.robot=robot
        self.inventory={'right_hand':None,'left_hand':None}

    def _regrasp_inventory(self):
        # to handle random release
        for hand in self.inventory.keys():
            obj=self.inventory[hand]
            obj_in_hand=get_obj_in_hand(self.scene,self.robot,hand)
            if (obj is not None) and (obj_in_hand is None):
                print(f"{obj.name} is not in inventory, regrasping")
                flag=grasp_primitive(self.scene,self.robot,obj,hand)
                if not flag:
                    print(f"PRIMITIVE: regrasp {obj.name} failed")
                    return False
                else:
                    print(f"PRIMITIVE: regrasp {obj.name} success")
        return True
        

    def navigate_to(self,obj):
        if navigate_to_obj(self.robot,obj):
            print("PRIMITIVE: navigate to {} success".format(obj.name))
        else:
            print("PRIMITIVE: navigate to {} fail".format(obj.name))
    
    def grasp(self,obj,hand):
        self._regrasp_inventory()
        flag=grasp_primitive(self.scene,self.robot,obj,hand)
        if flag:
            self.inventory[hand]=obj
        return flag
    
    def place_on_top(self,obj,hand):
        self._regrasp_inventory()
        obj_in_hand=self.inventory[hand]
        if obj_in_hand is None:
            print(f"PRIMITIVE: PLACE_ONTOP failed, {hand} is empty")
            return False
        flag=place_ontop_primitive(self.scene,self.robot,hand,obj_in_hand,obj)
        if flag:
            self.inventory[hand]=None
        return flag
    
    def place_inside(self,obj,hand):
        self._regrasp_inventory()
        obj_in_hand=self.inventory[hand]
        if obj_in_hand is None:
            print(f"PRIMITIVE: PLACE_INSIDE failed, {hand} is empty")
            return False
        flag=place_inside_primitive(self.scene,self.robot,hand,obj_in_hand,obj)
        if flag:
            self.inventory[hand]=None
        return flag
    
    def release(self,obj,hand):
        self._regrasp_inventory()
        if obj!= self.inventory[hand]:
            print(f"PRIMITIVE: RELEASE Failed, {obj.name} is not in {hand}")
            return False
        flag=release_primitive(self.scene,self.robot,hand,self.inventory[hand])
        if flag:
            self.inventory[hand]=None
        return flag
    
    def right_grasp(self,obj):
        return self.grasp(obj,'right_hand')
    
    def left_grasp(self,obj):
        return self.grasp(obj,'left_hand')
    
    def right_place_ontop(self,obj):
        return self.place_on_top(obj,'right_hand')
    
    def left_place_ontop(self,obj):
        return self.place_on_top(obj,'left_hand')
    
    def right_release(self,obj):
        return self.release(obj,'right_hand')
    
    def left_release(self,obj):
        return self.release(obj,'left_hand')
    
    def right_place_inside(self,obj):
        return self.place_inside(obj,'right_hand')
    
    def left_place_inside(self,obj):
        return self.place_inside(obj,'left_hand')
    
    def open(self,obj):
        navigate_if_needed(self.robot,obj)
        if hasattr(obj, "states") and object_states.Open in obj.states:
            obj.states[object_states.Open].set_value(True, fully=True)
            print(f"PRIMITIVE open {obj.name} success")
            return True
        else:
            print(f"PRIMITIVE open failed, {obj.name} cannot be opened")
    
        return False
    
    def close(self,obj):
        # same as open
        navigate_if_needed(self.robot,obj)
        if hasattr(obj, "states") and object_states.Open in obj.states:
            obj.states[object_states.Open].set_value(False, fully=True)
            print(f"PRIMITIVE close {obj.name} success")
            return True
        else:
            print("PRIMITIVE close failed, cannot be closed")
        
        return False
    
    def clean(self,obj):
        self._regrasp_inventory()
        navigate_if_needed(self.robot,obj)
        if not (hasattr(obj, "states") and object_states.Dusty in obj.states):
            print(f"PRIMITIVE clean failed, object {obj.name} cannot be cleaned")
            return False
        
        if not obj.states[object_states.Dusty].get_value():
            print(f"PRIMITIVE clean failed, object {obj.name} is not dusty")
            return False
        
        inventory =robot_invenvtory(self.scene,self.robot)
        has_cleaning_tool=False
        for inventory_obj in inventory:
            if hasattr(inventory_obj, "states") and object_states.CleaningTool in inventory_obj.states:
                has_cleaning_tool=True
                break

        if not has_cleaning_tool:
            print("PRIMITIVE clean failed, no cleaning tool available")
            return False
        
        obj.states[object_states.Dusty].set_value(False)
        print(f"clean {obj.name} success")
        return True

    def slice(self,obj):
        self._regrasp_inventory()
        navigate_if_needed(self.robot,obj)
        if not (hasattr(obj, "states") and object_states.Sliced in obj.states):
            print("PRIMITIVE slice failed, object cannot be sliced")
            return False
        if obj.states[object_states.Sliced].get_value():
            print("PRIMITIVE slice failed, object is already sliced")
            return False
        inventory =robot_invenvtory(self.scene,self.robot)
        
        has_slicer=False
        for inventory_obj in inventory:
            if hasattr(inventory_obj, "states") and object_states.Slicer in inventory_obj.states:
                has_slicer=True
                break
        if not has_slicer:
            print("PRIMITIVE slice failed, no slicer available")
            return False
        
        obj.states[object_states.Sliced].set_value(True)
        print(f"Slice {obj.name} success")
        return True


    def burn(self,obj):
        if hasattr(obj, "states") and object_states.Burnt in obj.states:
            obj.states[object_states.Burnt].set_value(True)
            return True
        else:
            print("PRIMITIVE burn failed, cannot be burnt")
        return False

    def cook(self,obj):
        if hasattr(obj, "states") and object_states.Cooked in obj.states:
            obj.states[object_states.Cooked].set_value(True)
            return True
        else:
            print("PRIMITIVE cook failed, cannot be cooked")
        return False

    def freeze(self,obj):
        if hasattr(obj, "states") and object_states.Frozen in obj.states:
            obj.states[object_states.Frozen].set_value(True)
            return True
        else:
            print("PRIMITIVE freeze failed, cannot be frozen")
        return False

    def unfreeze(self,obj):
        if hasattr(obj, "states") and object_states.Frozen in obj.states:
            obj.states[object_states.Frozen].set_value(False)
            return True
        else:
            print("PRIMITIVE unfreeze failed, cannot be unfrozen")
        return False

    def soak(self,obj):
        if hasattr(obj, "states") and object_states.Soaked in obj.states:
            obj.states[object_states.Soaked].set_value(True)
            return True
        else:
            print("PRIMITIVE soak failed, cannot be soaked")
        return False

    def dry(self,obj):
        if hasattr(obj, "states") and object_states.Soaked in obj.states:
            obj.states[object_states.Soaked].set_value(False)
            return True
        else:
            print("PRIMITIVE dry failed, cannot be dried")
        return False


    def stain(self,obj):
        if hasattr(obj, "states") and object_states.Stained in obj.states:
            obj.states[object_states.Stained].set_value(True)
            return True
        else:
            print("PRIMITIVE stain failed, cannot be stained")
        return False

    def toggle_on(self,obj):
        if hasattr(obj, "states") and object_states.ToggledOn in obj.states:
            obj.states[object_states.ToggledOn].set_value(True)
            return True
        else:
            print("PRIMITIVE toggle failed, cannot be toggled")
        return False

    def toggle_off(self,obj):
        if hasattr(obj, "states") and object_states.ToggledOn in obj.states:
            obj.states[object_states.ToggledOn].set_value(False)
            return True
        else:
            print("PRIMITIVE toggle failed, cannot be toggled")
        return False


