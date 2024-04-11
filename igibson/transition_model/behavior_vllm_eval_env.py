import argparse
import time
from enum import IntEnum

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
from igibson.utils.behavior_robot_planning_utils import dry_run_base_plan, plan_base_motion_br, plan_hand_motion_br
from igibson.utils.utils import restoreState

