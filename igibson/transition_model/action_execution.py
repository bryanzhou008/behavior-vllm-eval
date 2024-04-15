from igibson.transition_model.action_utils import grasp, place_inside,place_ontop,release,\
navigate_to_obj,navigate_if_needed,robot_invenvtory
from igibson.objects.articulated_object import URDFObject
from igibson import object_states
import pybullet as p
from enum import IntEnum    
"""
class ActionPrimitives(IntEnum):
    NAVIGATE_TO = 0
    LEFT_GRASP = 1
    RIGHT_GRASP = 2
    LEFT_PLACE_ONTOP = 3
    RIGHT_PLACE_ONTOP = 4
    LEFT_PLACE_INSIDE = 5
    RIGHT_PLACE_INSIDE = 6
    OPEN = 7
    CLOSE = 8
    BURN=9
    COOK=10
    CLEAN=11
    FREEZE=12
    UNFREEZE=13
    SLICE=14
    SOAK=15
    DRY=16
    STAIN=17
    TOGGLE_ON=18
    TOGGLE_OFF=19

"""

def navigate_to(scene,robot,obj):
    if navigate_to_obj(robot,obj):
        print("PRIMITIVE: navigate to {} success".format(obj.name))
    else:
        print("PRIMITIVE: navigate to {} fail".format(obj.name))

def right_grasp(scene,robot,obj):
    return grasp(scene,robot,obj,'right_hand')

def left_grasp(scene,robot,obj):
    return grasp(scene,robot,obj,'left_hand')

def right_place_ontop(scene,robot,obj):
    return place_ontop(scene,robot,obj,'right_hand')

def left_place_ontop(scene,robot,obj):
    return place_ontop(scene,robot,obj,'left_hand')

def right_release(scene,robot,obj):
    return release(scene,robot,obj,'right_hand')

def left_release(scene,robot,obj):
    return release(scene,robot,obj,'left_hand')


def right_place_inside(scene,robot,obj):    
    return place_inside(scene,robot,obj,'right_hand')

def left_place_inside(scene,robot,obj):
    return place_inside(scene,robot,obj,'left_hand')

def open(scene,robot,obj):
    # shall we check if the object is open already / at least one robot hand empty?
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Open in obj.states:
        obj.states[object_states.Open].set_value(True, fully=True)
        print(f"PRIMITIVE open {obj.name} success")
        return True
    else:
        print(f"PRIMITIVE open failed, {obj.name} cannot be opened")

    return False

def close(scene,robot,obj):
    # same as open
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Open in obj.states:
        obj.states[object_states.Open].set_value(False, fully=True)
        print(f"PRIMITIVE close {obj.name} success")
        return True
    else:
        print("PRIMITIVE close failed, cannot be closed")
    
    return False

def clean(scene,robot,obj):
    navigate_if_needed(robot,obj) # automatical or manual?

    if not (hasattr(obj, "states") and object_states.Dusty in obj.states):
        print(f"PRIMITIVE clean failed, object {obj.name} cannot be cleaned")
        return False
    if not obj.states[object_states.Dusty].get_value():
        print(f"PRIMITIVE clean failed, object {obj.name} is not dusty")
        return False
    
    inventory =robot_invenvtory(scene,robot)
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

def slice(scene,robot,obj):
    navigate_if_needed(robot,obj)
    if not (hasattr(obj, "states") and object_states.Sliced in obj.states):
        print("PRIMITIVE slice failed, object cannot be sliced")
        return False
    if obj.states[object_states.Sliced].get_value():
        print("PRIMITIVE slice failed, object is already sliced")
        return False
    inventory =robot_invenvtory(scene,robot)
    
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


def burn(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Burnt in obj.states:
        obj.states[object_states.Burnt].set_value(True)
        return True
    else:
        print("PRIMITIVE burn failed, cannot be burnt")
    return False

def cook(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Cooked in obj.states:
        obj.states[object_states.Cooked].set_value(True)
        return True
    else:
        print("PRIMITIVE cook failed, cannot be cooked")
    return False

def freeze(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Frozen in obj.states:
        obj.states[object_states.Frozen].set_value(True)
        return True
    else:
        print("PRIMITIVE freeze failed, cannot be frozen")
    return False

def unfreeze(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Frozen in obj.states:
        obj.states[object_states.Frozen].set_value(False)
        return True
    else:
        print("PRIMITIVE unfreeze failed, cannot be unfrozen")
    return False

def soak(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Soaked in obj.states:
        obj.states[object_states.Soaked].set_value(True)
        return True
    else:
        print("PRIMITIVE soak failed, cannot be soaked")
    return False

def dry(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Soaked in obj.states:
        obj.states[object_states.Soaked].set_value(False)
        return True
    else:
        print("PRIMITIVE dry failed, cannot be dried")
    return False


def stain(scene,robot,obj):
    if hasattr(obj, "states") and object_states.Stained in obj.states:
        obj.states[object_states.Stained].set_value(True)
        return True
    else:
        print("PRIMITIVE stain failed, cannot be stained")
    return False

def toggle_on(scene,robot,obj):
    if hasattr(obj, "states") and object_states.ToggledOn in obj.states:
        obj.states[object_states.ToggledOn].set_value(True)
        return True
    else:
        print("PRIMITIVE toggle failed, cannot be toggled")
    return False

def toggle_off(scene,robot,obj):
    if hasattr(obj, "states") and object_states.ToggledOn in obj.states:
        obj.states[object_states.ToggledOn].set_value(False)
        return True
    else:
        print("PRIMITIVE toggle failed, cannot be toggled")
    return False


