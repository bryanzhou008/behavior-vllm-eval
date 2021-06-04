import argparse
import numpy as np
import time
import tasknet
import types
import gym.spaces
import pybullet as p

from collections import OrderedDict
from gibson2.robots.behavior_robot import BehaviorRobot
from gibson2.envs.behavior_env import BehaviorEnv
from enum import IntEnum
from gibson2.object_states import *
from gibson2.robots.behavior_robot import BREye, BRBody, BRHand
from gibson2.object_states.utils import sample_kinematics
from gibson2.objects.articulated_object import URDFObject
NUM_ACTIONS = 6
class ActionPrimitives(IntEnum):
    NAVIGATE_TO = 0
    GRASP = 1
    PLACE_ONTOP = 2
    PLACE_INSIDE = 3
    OPEN = 4
    CLOSE = 5

def get_aabb_volume(lo, hi):
    dimension = hi - lo
    return dimension[0] * dimension[1] * dimension[2]

def detect_collision(bodyA):
    collision = False
    for body_id in range(p.getNumBodies()):
        if body_id == bodyA:
            continue
        closest_points = p.getClosestPoints(bodyA, body_id, distance=0.01)
        if len(closest_points) > 0:
            collision = True
            break
    return collision

def detect_robot_collision(robot):
    return detect_collision(robot.parts['body'].body_id) or \
           detect_collision(robot.parts['left_hand'].body_id) or \
           detect_collision(robot.parts['right_hand'].body_id)

class BehaviorMPEnv(BehaviorEnv):
    """
    iGibson Environment (OpenAI Gym interface)
    """

    def __init__(
        self,
        config_file,
        scene_id=None,
        mode='headless',
        action_timestep=1 / 10.0,
        physics_timestep=1 / 240.0,
        device_idx=0,
        render_to_tensor=False,
        automatic_reset=False,
        seed=0,
        action_filter='mobile_manipulation'
    ):
        """
        :param config_file: config_file path
        :param scene_id: override scene_id in config file
        :param mode: headless, gui, iggui
        :param action_timestep: environment executes action per action_timestep second
        :param physics_timestep: physics timestep for pybullet
        :param device_idx: which GPU to run the simulation and rendering on
        :param render_to_tensor: whether to render directly to pytorch tensors
        :param automatic_reset: whether to automatic reset after an episode finishes
        """
        super(BehaviorMPEnv, self).__init__(config_file=config_file,
                                            scene_id=scene_id,
                                            mode=mode,
                                            action_timestep=action_timestep,
                                            physics_timestep=physics_timestep,
                                            device_idx=device_idx,
                                            render_to_tensor=render_to_tensor,
                                            action_filter=action_filter,
                                            seed=seed,
                                            automatic_reset=automatic_reset)

        self.obj_in_hand = None

    def load_action_space(self):
        self.task_relevant_objects = [item for item in self.task.object_scope.values() if isinstance(item, URDFObject)]
        self.num_objects = len(self.task_relevant_objects)
        self.action_space = gym.spaces.Discrete(self.num_objects * NUM_ACTIONS)

    def step(self, action):
        obj_list_id = int(action) % self.num_objects
        action_primitive = int(action) // self.num_objects
        obj = self.task_relevant_objects[obj_list_id]
        if not (isinstance(obj, BRBody) or isinstance(obj, BRHand) or isinstance(obj, BREye)):
            if action_primitive == ActionPrimitives.NAVIGATE_TO:
                if self.navigate_to_obj(obj):
                    print('PRIMITIVE: navigate to {} success'.format(obj.name))
                else:
                    print('PRIMITIVE: navigate to {} fail'.format(obj.name))

            elif action_primitive == ActionPrimitives.GRASP:
                if self.obj_in_hand is None:
                    if hasattr(obj, 'states') and AABB in obj.states:
                        lo, hi = obj.states[AABB].get_value()
                        volume = get_aabb_volume(lo, hi)
                        if volume < 0.2 * 0.2 * 0.2 and not obj.main_body_is_fixed: # say we can only grasp small objects
                            if np.linalg.norm(np.array(obj.get_position()) - np.array(self.robots[0].get_position())) < 2:
                                self.obj_in_hand = obj
                                print('PRIMITIVE: grasp {} success'.format(obj.name))
                            else:
                                print('PRIMITIVE: grasp {} fail, too far'.format(obj.name))
                        else:
                            print('PRIMITIVE: grasp {} fail, too big or fixed'.format(obj.name))
            elif action_primitive == ActionPrimitives.PLACE_ONTOP:
                if self.obj_in_hand is not None and self.obj_in_hand != obj:
                    if np.linalg.norm(np.array(obj.get_position()) - np.array(self.robots[0].get_position())) < 2:
                        result = sample_kinematics('onTop', self.obj_in_hand, obj, True, use_ray_casting_method=True,
                                                   max_trials=50)
                        if result:
                            print('PRIMITIVE: place {} ontop {} success'.format(self.obj_in_hand.name, obj.name))
                            self.obj_in_hand = None
                        else:
                            print('PRIMITIVE: place {} ontop {} fail, sampling fail'.format(self.obj_in_hand.name, obj.name))
                    else:
                        print(
                            'PRIMITIVE: place {} ontop {} fail, too far'.format(self.obj_in_hand.name, obj.name))

            elif action_primitive == ActionPrimitives.PLACE_INSIDE:
                if self.obj_in_hand is not None and self.obj_in_hand != obj:
                    if np.linalg.norm(np.array(obj.get_position()) - np.array(self.robots[0].get_position())) < 2:
                        result = sample_kinematics('inside', self.obj_in_hand, obj, True, use_ray_casting_method=True,
                                                   max_trials=50)
                        if result:
                            print('PRIMITIVE: place {} inside {} success'.format(self.obj_in_hand.name, obj.name))
                            self.obj_in_hand = None
                        else:
                            print('PRIMITIVE: place {} inside {} fail, sampling fail'.format(self.obj_in_hand.name, obj.name))
                    else:
                        print('PRIMITIVE: place {} inside {} fail, too far'.format(self.obj_in_hand.name, obj.name))
            elif action_primitive == ActionPrimitives.OPEN:
                if hasattr(obj, 'states') and Open in obj.states:
                    obj.states[Open].set_value(True)
            elif action_primitive == ActionPrimitives.CLOSE:
                if hasattr(obj, 'states') and Open in obj.states:
                    obj.states[Open].set_value(False)

        state, reward, done, info = super(BehaviorMPEnv, self).step(np.zeros(17))
        print("PRIMITIVE satisfied predicates:", info["satisfied_predicates"])
        return state, reward, done, info

    def navigate_to_obj(self, obj):
        # test agent positions around an obj
        # try to place the agent near the object, and rotate it to the object
        distance_to_try = [0.5, 1, 2, 3]
        valid_position = None # ((x,y,z),(roll, pitch, yaw))

        obj_pos = obj.get_position()
        for distance in distance_to_try:
            for _ in range(20):
                # p.restoreState(state_id)
                yaw = np.random.uniform(-np.pi, np.pi)
                pos = [obj_pos[0] + distance * np.cos(yaw), obj_pos[1] + distance * np.sin(yaw), 0.7]
                orn = [0,0,yaw-np.pi]
                self.robots[0].set_position_orientation(pos, p.getQuaternionFromEuler(orn))
                if not detect_robot_collision(self.robots[0]):
                    valid_position = (pos, orn)
                    break
            if valid_position is not None:
                break

        if valid_position is not None:
            self.robots[0].set_position_orientation(valid_position[0], p.getQuaternionFromEuler(valid_position[1]))
            return True
        else:
            return False
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config',
        '-c',
        default = 'gibson2/examples/configs/behavior.yaml',
        help='which config file to use [default: use yaml files in examples/configs]')
    parser.add_argument('--mode',
                        '-m',
                        choices=['headless', 'gui', 'iggui', 'pbgui'],
                        default='gui',
                        help='which mode for simulation (default: headless)')
    args = parser.parse_args()

    env = BehaviorMPEnv(config_file=args.config,
                      mode=args.mode,
                      action_timestep=1.0 / 300.0,
                      physics_timestep=1.0 / 300.0)
    step_time_list = []
    for episode in range(100):
        print('Episode: {}'.format(episode))
        start = time.time()
        env.reset()
        for i in range(1000):  # 10 seconds
            action = env.action_space.sample()
            state, reward, done, info = env.step(action)
            print(reward, info)
            if done:
                break
        print('Episode finished after {} timesteps, took {} seconds.'.format(
            env.current_step, time.time() - start))
    env.close()
