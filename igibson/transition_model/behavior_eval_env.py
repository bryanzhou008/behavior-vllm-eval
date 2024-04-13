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

from igibson.transition_model.actions import ActionPrimitives
from igibson.transition_model.action_execution import navigate_to, grasp, place_ontop, place_inside, open, close, burn, cook, \
clean, freeze, unfreeze, slice, soak, dry, stain, toggle
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
                navigate_to(self.robots[0], obj)

            elif action_primitive == ActionPrimitives.RIGHT_GRASP or action_primitive == ActionPrimitives.LEFT_GRASP:
                hand = "right_hand" if action_primitive == ActionPrimitives.RIGHT_GRASP else "left_hand"
                grasp(self.robots[0], obj, self.scene, hand)
            elif (
                action_primitive == ActionPrimitives.LEFT_PLACE_ONTOP
                or action_primitive == ActionPrimitives.RIGHT_PLACE_ONTOP
            ):
                hand = "right_hand" if action_primitive == ActionPrimitives.RIGHT_PLACE_ONTOP else "left_hand"
                place_ontop(self.robots[0], obj, self.scene, hand)

            elif (
                action_primitive == ActionPrimitives.LEFT_PLACE_INSIDE
                or action_primitive == ActionPrimitives.RIGHT_PLACE_INSIDE
            ):
                hand = "right_hand" if action_primitive == ActionPrimitives.RIGHT_PLACE_INSIDE else "left_hand"
                place_inside(self.robots[0], obj, self.scene, hand)
                        
            elif action_primitive == ActionPrimitives.OPEN:
                open(self.robots[0], obj)

            elif action_primitive == ActionPrimitives.CLOSE:
                close(self.robots[0], obj)

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

    




