import pybullet as p
import numpy as np
from igibson.utils.utils import restoreState
from igibson.objects.articulated_object import URDFObject
from igibson.objects.multi_object_wrappers import ObjectMultiplexer
from igibson.object_states.utils import sample_kinematics
from igibson import object_states

def get_aabb_volume(lo, hi):
    dimension = hi - lo
    return dimension[0] * dimension[1] * dimension[2]


def detect_collision(bodyA, object_in_hand=None):
    collision = False
    for body_id in range(p.getNumBodies()):
        if body_id == bodyA or body_id == object_in_hand:
            continue
        closest_points = p.getClosestPoints(bodyA, body_id, distance=0.01)
        if len(closest_points) > 0:
            collision = True
            break
    return collision


def detect_robot_collision(robot):
    left_object_in_hand = robot.parts["left_hand"].object_in_hand
    right_object_in_hand = robot.parts["right_hand"].object_in_hand
    return (
        detect_collision(robot.parts["body"].get_body_id())
        or detect_collision(robot.parts["left_hand"].get_body_id(), left_object_in_hand)
        or detect_collision(robot.parts["right_hand"].get_body_id(), right_object_in_hand)
    )

def reset_and_release_hand(robot, hand):
    robot.set_position_orientation(robot.get_position(), robot.get_orientation())
    for _ in range(100):
        robot.parts[hand].set_close_fraction(0)
        robot.parts[hand].trigger_fraction = 0
        p.stepSimulation()


def grasp_obj(robot, obj, hand):
        obj.set_position(np.array(robot.parts[hand].get_position()))  
        robot.parts[hand].set_close_fraction(1)
        robot.parts[hand].trigger_fraction = 1
        p.stepSimulation()
        obj.set_position(np.array(robot.parts[hand].get_position()))
        robot.parts[hand].handle_assisted_grasping(
            np.zeros(
                28,
            ),
            override_ag_data=(obj.get_body_id(), -1),
        )


def get_obj_in_hand(scene,robot, hand):
    obj_in_hand_id = robot.parts[hand].object_in_hand
    obj_in_hand = scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
    return obj_in_hand

def robot_invenvtory(scene,robot):
    rst=[]
    for hand in ["left_hand", "right_hand"]:
        obj_in_hand = get_obj_in_hand(scene,robot, hand)
        if obj_in_hand is not None:
            rst.append(obj_in_hand)
    return rst 

def place_obj(scene,robot, hand,original_state, target_pos, target_orn):
    obj_in_hand = get_obj_in_hand(scene,robot, hand)

    restoreState(original_state)
    p.removeState(original_state)
    
    reset_and_release_hand(robot,hand)

    robot.parts[hand].force_release_obj()
    obj_in_hand.set_position_orientation(target_pos, target_orn)


def navigate_to_obj(robot, obj):
    # test agent positions around an obj
    # try to place the agent near the object, and rotate it to the object
    valid_position = None  # ((x,y,z),(roll, pitch, yaw))
    original_position = robot.get_position()
    original_orientation = robot.get_orientation()
    if isinstance(obj, URDFObject) or isinstance(obj,ObjectMultiplexer):
        distance_to_try = [0.6, 1.2, 1.8, 2.4]
        obj_pos = obj.get_position()
        for distance in distance_to_try:
            for _ in range(20):
                yaw = np.random.uniform(-np.pi, np.pi)
                pos = [obj_pos[0] + distance * np.cos(yaw), obj_pos[1] + distance * np.sin(yaw), 0.7]
                orn = [0, 0, yaw - np.pi]
                robot.set_position_orientation(pos, p.getQuaternionFromEuler(orn))
                eye_pos = robot.parts["eye"].get_position()
                obj_pos = obj.get_position()
                ray_test_res = p.rayTest(eye_pos, obj_pos)
                blocked = False
                if len(ray_test_res) > 0 and ray_test_res[0][0] != obj.get_body_id():
                    blocked = True

                if not detect_robot_collision(robot) and not blocked:
                    valid_position = (pos, orn)
                    break
            if valid_position is not None:
                break
    else:
        for _ in range(60):
            _, pos = obj.scene.get_random_point_by_room_instance(obj.room_instance)
            yaw = np.random.uniform(-np.pi, np.pi)
            orn = [0, 0, yaw]
            robot.set_position_orientation(pos, p.getQuaternionFromEuler(orn))
            if not detect_robot_collision(robot):
                valid_position = (pos, orn)
                break

    if valid_position is not None:
        target_x = valid_position[0][0]
        target_y = valid_position[0][1]
        x = original_position[0]
        y = original_position[1]
        minx = min(x, target_x) - 1
        miny = min(y, target_y) - 1
        maxx = max(x, target_x) + 1
        maxy = max(y, target_y) + 1

        robot.set_position_orientation(valid_position[0], p.getQuaternionFromEuler(valid_position[1]))
        return True
    else:
        robot.set_position_orientation(original_position, original_orientation)
        return False
    
def navigate_if_needed(robot, obj):
    if obj.states[object_states.InReachOfRobot].get_value():
        return

    for _ in range(10):
        if navigate_to_obj(robot,obj):
            return
        
def grasp(scene,robot,obj,hand):
    obj_in_hand = get_obj_in_hand(scene,robot,hand)
    if obj_in_hand is None:
        if (isinstance(obj, URDFObject) or isinstance(obj,ObjectMultiplexer)) and hasattr(obj, "states") and object_states.AABB in obj.states:
            lo, hi = obj.states[object_states.AABB].get_value()
            volume = get_aabb_volume(lo, hi)
            if volume < 0.5 * 0.5 * 0.5 and not obj.main_body_is_fixed:  # we can only grasp small objects
                navigate_if_needed(robot,obj)
                grasp_obj(robot, obj, hand)
                obj_in_hand = get_obj_in_hand(scene,robot,hand)
                print("PRIMITIVE: grasp {} success, obj in hand {}".format(obj.name, obj_in_hand.name))
                return True
            else:
                print("PRIMITIVE: grasp {} fail, too big or fixed".format(obj.name))
        else:
            print("PRIMITIVE: grasp {} fail, not URDFObject,ObjectMultiplexer or no AABB".format(obj.name))
            return False

    else:
        print("PRIMITIVE: grasp {} fail, hand already holding object".format(obj_in_hand.name))
    
    return False

def place_inside(scene,robot,obj,hand):
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
                return True
            else:
                print(
                    "PRIMITIVE: place {} inside {} fail, sampling fail".format(obj_in_hand.name, obj.name)
                )
                p.removeState(state)
        else:
            print("PRIMITIVE: place {} inside {} fail, need open not open".format(obj_in_hand.name, obj.name))
    else:
        print("PRIMITIVE: place {} inside {} fail, hand empty or holding same object".format(obj_in_hand.name, obj.name))
    return False

def place_ontop(scene,robot,obj,hand):
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
                place_obj(scene,robot,hand,state, pos, orn)
                print("PRIMITIVE: place {} ontop {} success".format(obj_in_hand.name, obj.name))
                return True
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
                return True
            else:
                print("PRIMITIVE: place {} ontop {} fail, sampling fail".format(obj_in_hand.name, obj.name))
                p.removeState(state)
    else:
        print("PRIMITIVE: place {} ontop {} fail, hand empty or holding same object".format(obj_in_hand.name, obj.name))
    return False

def release(scene,robot,obj,hand):
    obj_in_hand = get_obj_in_hand(scene,robot,hand)
    if obj_in_hand is None:
        print("PRIMITIVE: release fail, hand empty")
        return False
    
    if obj_in_hand!=obj:
        print("PRIMITIVE: release {} fail, hand holding wrong object {}".format(obj_in_hand.name, obj.name))
        return False

    placable_objects = []
    reachable_placable_objects = []
    for name in ['cabinet','table','floor']:
        for k in scene.objects_by_category.keys():
            if name in k:
                for obj in scene.objects_by_category[k]:
                    if hasattr(obj, "states") and object_states.InReachOfRobot in obj.states:
                        if obj.states[object_states.InReachOfRobot].get_value():
                            reachable_placable_objects.append(obj)
                    else:
                        placable_objects.append(obj)    
    
    for obj in reachable_placable_objects:
        if place_ontop(scene,robot,obj,hand):
            print("PRIMITIVE: release {} success on {}".format(obj_in_hand.name, obj.name))
            return True
    for obj in placable_objects:
        if place_ontop(scene,robot,obj,hand):
            print("PRIMITIVE: release {} success on {}".format(obj_in_hand.name, obj.name))
            return True
    
    print("PRIMITIVE: release {} fail, no place to release".format(obj_in_hand.name))
    return False