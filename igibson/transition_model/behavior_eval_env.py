from igibson.transition_model.actions import ActionPrimitives
import argparse
import time
import gym.spaces
import numpy as np
import pybullet as p

from igibson import object_states
from igibson.envs.behavior_env import BehaviorEnv
from igibson.external.pybullet_tools.utils import CIRCULAR_LIMITS
from igibson.object_states.on_floor import RoomFloor
from igibson.object_states.utils import sample_kinematics
from igibson.objects.articulated_object import URDFObject
from igibson.robots.behavior_robot import BRBody, BREye, BRHand


class BehaviorEvalEnv(BehaviorEnv):
    """
    iGibson Environment (OpenAI Gym interface)
    """

    def __init__(
        self,
        config_file,
        scene_id=None,
        mode="headless",
        action_timestep=1 / 10.0,
        physics_timestep=1 / 240.0,
        device_idx=0,
        render_to_tensor=False,
        automatic_reset=False,
        seed=0,
        action_filter="mobile_manipulation",
        activity_relevant_objects_only=True,
    ):
        """
        @param config_file: config_file path
        @param scene_id: override scene_id in config file
        @param mode: headless, simple, iggui
        @param action_timestep: environment executes action per action_timestep second
        @param physics_timestep: physics timestep for pybullet
        @param device_idx: which GPU to run the simulation and rendering on
        @param render_to_tensor: whether to render directly to pytorch tensors
        @param automatic_reset: whether to automatic reset after an episode finishes
        @param seed: RNG seed for sampling
        @param action_filter: see BehaviorEnv
        @param activity_relevant_objects_only: Whether the actions should be parameterized by AROs or all scene objs.
        """
        self.activity_relevant_objects_only = activity_relevant_objects_only
        super(BehaviorEvalEnv, self).__init__(
            config_file=config_file,
            scene_id=scene_id,
            mode=mode,
            action_timestep=action_timestep,
            physics_timestep=physics_timestep,
            device_idx=device_idx,
            render_to_tensor=render_to_tensor,
            action_filter=action_filter,
            seed=seed,
            automatic_reset=automatic_reset,
        )

        self.robots[0].initial_z_offset = 0.7

    def load_action_space(self):
        if self.activity_relevant_objects_only:
            self.addressable_objects = [
                item
                for item in self.task.object_scope.values()
                if isinstance(item, URDFObject) or isinstance(item, RoomFloor)
            ]
        else:
            self.addressable_objects = list(
                set(self.task.simulator.scene.objects_by_name.values()) | set(self.task.object_scope.values())
            )

        self.num_objects = len(self.addressable_objects)
        self.action_space = gym.spaces.Discrete(self.num_objects * len(ActionPrimitives))

    def get_body_ids(self, include_self=False):
        ids = []
        for object in self.scene.get_objects():
            if isinstance(object, URDFObject):
                ids.extend(object.body_ids)

        if include_self:
            ids.append(self.robots[0].parts["left_hand"].get_body_id())
            ids.append(self.robots[0].parts["body"].get_body_id())

        return ids


    def step(self, action):
        obj_list_id = int(action) % self.num_objects
        action_primitive = int(action) // self.num_objects

        obj = self.addressable_objects[obj_list_id]
        if not (isinstance(obj, BRBody) or isinstance(obj, BRHand) or isinstance(obj, BREye)):
            if action_primitive == ActionPrimitives.NAVIGATE_TO:
                if self.navigate_to_obj(obj):
                    print("PRIMITIVE: navigate to {} success".format(obj.name))
                else:
                    print("PRIMITIVE: navigate to {} fail".format(obj.name))

            elif action_primitive == ActionPrimitives.RIGHT_GRASP or action_primitive == ActionPrimitives.LEFT_GRASP:
                hand = "right_hand" if action_primitive == ActionPrimitives.RIGHT_GRASP else "left_hand"
                obj_in_hand_id = self.robots[0].parts[hand].object_in_hand
                obj_in_hand = self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
                if obj_in_hand is None:
                    if isinstance(obj, URDFObject) and hasattr(obj, "states") and object_states.AABB in obj.states:
                        lo, hi = obj.states[object_states.AABB].get_value()
                        volume = get_aabb_volume(lo, hi)
                        if volume < 0.2 * 0.2 * 0.2 and not obj.main_body_is_fixed:  # we can only grasp small objects
                            self.navigate_if_needed(obj)
                            self.grasp_obj(obj, hand)
                            obj_in_hand_id = self.robots[0].parts[hand].object_in_hand
                            obj_in_hand = (
                                self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
                            )
                            print("PRIMITIVE: grasp {} success, obj in hand {}".format(obj.name, obj_in_hand))
                        else:
                            print("PRIMITIVE: grasp {} fail, too big or fixed".format(obj.name))
            elif (
                action_primitive == ActionPrimitives.LEFT_PLACE_ONTOP
                or action_primitive == ActionPrimitives.RIGHT_PLACE_ONTOP
            ):
                hand = "right_hand" if action_primitive == ActionPrimitives.RIGHT_PLACE_ONTOP else "left_hand"
                obj_in_hand_id = self.robots[0].parts[hand].object_in_hand
                obj_in_hand = self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
                if obj_in_hand is not None and obj_in_hand != obj:
                    print("PRIMITIVE:attempt to place {} ontop {}".format(obj_in_hand.name, obj.name))

                    if isinstance(obj, URDFObject):
                        self.navigate_if_needed(obj)

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
                            self.place_obj(state, pos, orn, hand)
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
                            self.place_obj(state, pos, orn, hand)
                        else:
                            print("PRIMITIVE: place {} ontop {} fail, sampling fail".format(obj_in_hand.name, obj.name))
                            p.removeState(state)

            elif (
                action_primitive == ActionPrimitives.LEFT_PLACE_INSIDE
                or action_primitive == ActionPrimitives.RIGHT_PLACE_INSIDE
            ):
                hand = "right_hand" if action_primitive == ActionPrimitives.RIGHT_PLACE_INSIDE else "left_hand"
                obj_in_hand_id = self.robots[0].parts[hand].object_in_hand
                obj_in_hand = self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
                if obj_in_hand is not None and obj_in_hand != obj and isinstance(obj, URDFObject):
                    print("PRIMITIVE:attempt to place {} inside {}".format(obj_in_hand.name, obj.name))
                    if (
                        hasattr(obj, "states")
                        and object_states.Open in obj.states
                        and obj.states[object_states.Open].get_value()
                    ) or (hasattr(obj, "states") and not object_states.Open in obj.states):
                        self.navigate_if_needed(obj)

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
                            self.place_obj(state, pos, orn, hand)
                            print("PRIMITIVE: place {} inside {} success".format(obj_in_hand.name, obj.name))
                        else:
                            print(
                                "PRIMITIVE: place {} inside {} fail, sampling fail".format(obj_in_hand.name, obj.name)
                            )
                            p.removeState(state)
                    else:
                        print(
                            "PRIMITIVE: place {} inside {} fail, need open not open".format(obj_in_hand.name, obj.name)
                        )
            elif action_primitive == ActionPrimitives.OPEN:
                self.navigate_if_needed(obj)

                if hasattr(obj, "states") and object_states.Open in obj.states:
                    obj.states[object_states.Open].set_value(True, fully=True)
                else:
                    print("PRIMITIVE open failed, cannot be opened")

            elif action_primitive == ActionPrimitives.CLOSE:
                self.navigate_if_needed(obj)

                if hasattr(obj, "states") and object_states.Open in obj.states:
                    obj.states[object_states.Open].set_value(False)
                else:
                    print("PRIMITIVE close failed, cannot be opened")

        state, reward, done, info = super(BehaviorEvalEnv, self).step(np.zeros(17))
        print("PRIMITIVE satisfied predicates:", info["satisfied_predicates"])
        return state, reward, done, info

    def reset(self):
        obs = super(BehaviorEvalEnv, self).reset()
        for hand in ["left_hand", "right_hand"]:
            self.robots[0].parts[hand].set_close_fraction(0)
            self.robots[0].parts[hand].trigger_fraction = 0
            self.robots[0].parts[hand].force_release_obj()
        return obs


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        "-c",
        default="igibson/examples/configs/behavior.yaml",
        help="which config file to use [default: use yaml files in examples/configs]",
    )
    parser.add_argument(
        "--mode",
        "-m",
        choices=["headless", "gui", "iggui", "pbgui"],
        default="gui",
        help="which mode for simulation (default: headless)",
    )
    args = parser.parse_args()

    env = BehaviorEvalEnv(
        config_file=args.config,
        mode=args.mode,
        action_timestep=1.0 / 300.0,
        physics_timestep=1.0 / 300.0,
    )
    step_time_list = []
    for episode in range(100):
        print("Episode: {}".format(episode))
        start = time.time()
        env.reset()

        for i in range(1000):  # 10 seconds
            action = env.action_space.sample()
            state, reward, done, info = env.step(action)
            print(reward, info)
            if done:
                break
        print("Episode finished after {} timesteps, took {} seconds.".format(env.current_step, time.time() - start))
    env.close()

    




