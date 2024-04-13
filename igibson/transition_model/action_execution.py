from igibson.transition_model.action_utils import get_aabb_volume,  \
grasp_obj, get_obj_in_hand, place_obj,navigate_to_obj,navigate_if_needed
from igibson.objects.articulated_object import URDFObject
from igibson import object_states
import pybullet as p
from igibson.object_states.utils import sample_kinematics

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
    TOGGLE=18

"""

def navigate_to(robot,obj,scene=None,hand=None):
    if navigate_to_obj(robot,obj):
        print("PRIMITIVE: navigate to {} success".format(obj.name))
    else:
        print("PRIMITIVE: navigate to {} fail".format(obj.name))

def grasp(robot,obj,scene,hand):
    obj_in_hand = get_obj_in_hand(scene,robot,hand)
    if obj_in_hand is None:
        if isinstance(obj, URDFObject) and hasattr(obj, "states") and object_states.AABB in obj.states:
            lo, hi = obj.states[object_states.AABB].get_value()
            volume = get_aabb_volume(lo, hi)
            if volume < 0.2 * 0.2 * 0.2 and not obj.main_body_is_fixed:  # we can only grasp small objects
                navigate_if_needed(robot,obj)
                grasp_obj(robot, obj, hand)
                obj_in_hand = get_obj_in_hand(scene,robot,hand)
                print("PRIMITIVE: grasp {} success, obj in hand {}".format(obj.name, obj_in_hand))
            else:
                print("PRIMITIVE: grasp {} fail, too big or fixed".format(obj.name))

    else:
        print("PRIMITIVE: grasp {} fail, hand already holding object".format(obj_in_hand.name))

def place_ontop(robot,obj,scene,hand):
    obj_in_hand = get_obj_in_hand(scene,robot,hand)
    if obj_in_hand is not None and obj_in_hand != obj:
        print("PRIMITIVE:attempt to place {} ontop {}".format(obj_in_hand.name, obj.name))

        if isinstance(obj, URDFObject):
            navigate_if_needed(robot,obj)

            state = p.saveState()
            result = sample_kinematics(
                "onTop",
                obj_in_hand,
                obj,
                True,
                use_ray_casting_method=True,
                max_trials=20,
            )

            if result:
                pos = obj_in_hand.get_position()
                orn = obj_in_hand.get_orientation()
                place_obj(state, pos, orn, hand)
                print("PRIMITIVE: place {} ontop {} success".format(obj_in_hand.name, obj.name))
            else:
                p.removeState(state)
                print("PRIMITIVE: place {} ontop {} fail, sampling fail".format(obj_in_hand.name, obj.name))
        else:
            state = p.saveState()
            result = sample_kinematics(
                "onFloor", obj_in_hand, obj, True, use_ray_casting_method=True, max_trials=20
            )
            if result:
                print("PRIMITIVE: place {} ontop {} success".format(obj_in_hand.name, obj.name))
                pos = obj_in_hand.get_position()
                orn = obj_in_hand.get_orientation()
                place_obj(scene,robot, hand,state, pos, orn)
            else:
                print("PRIMITIVE: place {} ontop {} fail, sampling fail".format(obj_in_hand.name, obj.name))
                p.removeState(state)

def place_inside(robot,obj,scene,hand):
    obj_in_hand = get_obj_in_hand(scene,robot,hand)
    if obj_in_hand is not None and obj_in_hand != obj and isinstance(obj, URDFObject):
        print("PRIMITIVE:attempt to place {} inside {}".format(obj_in_hand.name, obj.name))
        if (
            hasattr(obj, "states")
            and object_states.Open in obj.states
            and obj.states[object_states.Open].get_value()
        ) or (hasattr(obj, "states") and not object_states.Open in obj.states):
            navigate_if_needed(robot,obj)

            state = p.saveState()
            result = sample_kinematics(
                "inside",
                obj_in_hand,
                obj,
                True,
                use_ray_casting_method=True,
                max_trials=20,
            )

            if result:
                pos = obj_in_hand.get_position()
                orn = obj_in_hand.get_orientation()
                place_obj(scene,robot,hand,state, pos, orn)
                print("PRIMITIVE: place {} inside {} success".format(obj_in_hand.name, obj.name))
            else:
                print(
                    "PRIMITIVE: place {} inside {} fail, sampling fail".format(obj_in_hand.name, obj.name)
                )
                p.removeState(state)
        else:
            print("PRIMITIVE: place {} inside {} fail, need open not open".format(obj_in_hand.name, obj.name))

def open(robot,obj,scene=None,hand=None):
    # shall we check if the object is open already / at least one robot hand empty?
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Open in obj.states:
        obj.states[object_states.Open].set_value(True, fully=True)
    else:
        print("PRIMITIVE open failed, cannot be opened")

def close(robot,obj,scene=None,hand=None):
    # same as open
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Open in obj.states:
        obj.states[object_states.Open].set_value(False, fully=True)
    else:
        print("PRIMITIVE close failed, cannot be closed")

def burn(robot,obj,scene=None,hand=None):
    pass

def cook(robot,obj,scene=None,hand=None):
    pass

def clean(robot,obj,scene,hand):
    obj_in_left= get_obj_in_hand(scene,robot,"left_hand")
    obj_in_right= get_obj_in_hand(scene,robot,"right_hand")


def freeze(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Frozen in obj.states:
        obj.states[object_states.Frozen].set_value(True, fully=True)
    else:
        print("PRIMITIVE freeze failed, cannot be frozen")

def unfreeze(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Frozen in obj.states:
        obj.states[object_states.Frozen].set_value(False, fully=True)
    else:
        print("PRIMITIVE unfreeze failed, cannot be unfrozen")

def slice(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Sliced in obj.states:
        obj.states[object_states.Sliced].set_value(True, fully=True)
    else:
        print("PRIMITIVE slice failed, cannot be sliced")

def soak(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Soaked in obj.states:
        obj.states[object_states.Soaked].set_value(True, fully=True)
    else:
        print("PRIMITIVE soak failed, cannot be soaked")

def dry(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Soaked in obj.states:
        obj.states[object_states.Soaked].set_value(False, fully=True)
    else:
        print("PRIMITIVE dry failed, cannot be dried")

def stain(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.Stained in obj.states:
        obj.states[object_states.Stained].set_value(True, fully=True)
    else:
        print("PRIMITIVE stain failed, cannot be stained")

def toggle(robot,obj,scene=None,hand=None):
    navigate_if_needed(robot,obj)
    if hasattr(obj, "states") and object_states.ToggledOn in obj.states:
        obj.states[object_states.ToggledOn].set_value(True, fully=True)
    else:
        print("PRIMITIVE toggle failed, cannot be toggled")
