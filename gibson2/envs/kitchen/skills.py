import numpy as np
from collections import OrderedDict

import gibson2.envs.kitchen.transform_utils as TU
import gibson2.external.pybullet_tools.transformations as T
from gibson2.envs.kitchen.plan_utils import ConfigurationPath, CartesianPath, configuration_path_to_cartesian_path, \
    compute_grasp_pose, NoPlanException, PreconditionNotSatisfied, move_to_end_of_configuration_path
from gibson2.envs.kitchen.robots import PlannerRobot
from gibson2.envs.kitchen.robots import GRIPPER_CLOSE, GRIPPER_OPEN
import gibson2.external.pybullet_tools.utils as PBU


SKILL_ORIENTATIONS = OrderedDict(
    front=T.quaternion_from_euler(0, 0, 0),
    left=T.quaternion_from_euler(0, 0, np.pi / 2),
    right=T.quaternion_from_euler(0, 0, -np.pi / 2),
    back=T.quaternion_from_euler(0, 0, np.pi),
    top=T.quaternion_from_euler(0, np.pi / 2, 0),
)

SKILL_ORIENTATION_NAMES = list(SKILL_ORIENTATIONS.keys())

ALL_ORIENTATIONS = OrderedDict(SKILL_ORIENTATIONS)

ALL_ORIENTATIONS["backward"] = T.quaternion_from_euler(np.pi / 2, 0, 0)

DRAWER_GRASP_ORN = T.quaternion_from_euler(-np.pi / 2, 0, np.pi)

DEFAULT_JOINT_RESOLUTIONS = (0.1, 0.1, 0.1, np.pi * 0.05, np.pi * 0.05, np.pi * 0.05)


def plan_move_to(
        planner,
        obstacles,
        target_pose,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS
):
    confs = planner.plan_joint_path(
        target_pose=target_pose, obstacles=obstacles, resolutions=joint_resolutions)
    conf_path = ConfigurationPath()
    conf_path.append_segment(confs, gripper_state=GRIPPER_OPEN)
    path = configuration_path_to_cartesian_path(planner, conf_path)
    return path.interpolate(pos_resolution=0.05, orn_resolution=np.pi / 8)


def plan_skill_touch(
        planner,
        obstacles,
        target_pose,
        reach_distance=0.,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS
):
    """
    plan skill for touching an object
    Args:
        planner (PlannerRobot): planner
        obstacles (list, tuple): a list obstacle ids
        target_pose (tuple): pose for grasping the handle
        reach_distance (float): distance for reaching to touch
        joint_resolutions (list, tuple): motion planning joint-space resolution

    Returns:
        path (CartesianPath)
    """
    # approach handle
    approach_pose = PBU.multiply(target_pose, ([-reach_distance, 0, 0], PBU.unit_quat()))
    approach_confs = planner.plan_joint_path(
        target_pose=approach_pose, obstacles=obstacles, resolutions=joint_resolutions)
    conf_path = ConfigurationPath()
    conf_path.append_segment(approach_confs, gripper_state=GRIPPER_CLOSE)
    touch_path = configuration_path_to_cartesian_path(planner, conf_path)
    touch_path.append(target_pose, gripper_state=GRIPPER_CLOSE)
    touch_path.append_pause(5)
    touch_path.append(approach_pose, gripper_state=GRIPPER_OPEN)
    touch_path.append_pause(5)
    return touch_path.interpolate(pos_resolution=0.05, orn_resolution=np.pi/8)


def plan_skill_open_prismatic(
        planner,
        obstacles,
        grasp_pose,
        prismatic_move_distance,
        reach_distance=0.,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS
):
    """
    plan skill for opening articulated object with prismatic joint (e.g., drawers)
    Args:
        planner (PlannerRobot): planner
        obstacles (list, tuple): a list obstacle ids
        grasp_pose (tuple): pose for grasping the handle
        prismatic_move_distance (float): distance for retract to open
        reach_distance (float): distance for reaching to grasp
        joint_resolutions (list, tuple): motion planning joint-space resolution

    Returns:
        path (CartesianPath)
    """
    # approach handle
    approach_pose = PBU.multiply(grasp_pose, ([-reach_distance, 0, 0], PBU.unit_quat()))
    approach_confs = planner.plan_joint_path(
        target_pose=approach_pose, obstacles=obstacles, resolutions=joint_resolutions)
    conf_path = ConfigurationPath()
    conf_path.append_segment(approach_confs, gripper_state=GRIPPER_OPEN)
    grasp_path = configuration_path_to_cartesian_path(planner, conf_path)

    # grasp handle
    grasp_path.append(grasp_pose, gripper_state=GRIPPER_OPEN)
    grasp_path.append_segment([grasp_pose]*5, gripper_state=GRIPPER_CLOSE)
    grasp_path = grasp_path.interpolate(pos_resolution=0.05, orn_resolution=np.pi/8)

    # retract to open
    retract_pose = PBU.multiply(grasp_pose, ([prismatic_move_distance, 0, 0], PBU.unit_quat()))
    retract_path = CartesianPath()
    retract_path.append(grasp_pose, gripper_state=GRIPPER_CLOSE)
    retract_path.append(retract_pose, gripper_state=GRIPPER_CLOSE)
    retract_path.append_pause(num_steps=5)  # pause before release
    retract_path.append_segment([retract_pose] * 2, gripper_state=GRIPPER_OPEN)

    # retract a bit more to make room for the next motion
    retract_more_pose = PBU.multiply(retract_pose, ([-0.10, 0, 0], PBU.unit_quat()))
    retract_path.append(retract_more_pose, gripper_state=GRIPPER_OPEN)
    retract_path = retract_path.interpolate(pos_resolution=0.005, orn_resolution=np.pi/8)  # slowly open
    return grasp_path + retract_path


def joint_plan_skill_open_prismatic(
        planner,
        obstacles,
        grasp_pose,
        prismatic_move_distance,
        reach_distance=0.,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS
):
    """
    plan skill for opening articulated object with prismatic joint (e.g., drawers)
    Args:
        planner (PlannerRobot): planner
        obstacles (list, tuple): a list obstacle ids
        grasp_pose (tuple): pose for grasping the handle
        prismatic_move_distance (float): distance for retract to open
        reach_distance (float): distance for reaching to grasp
        joint_resolutions (list, tuple): motion planning joint-space resolution

    Returns:
        path (CartesianPath)
    """
    # approach handle
    approach_pose = PBU.multiply(grasp_pose, ([-reach_distance, 0, 0], PBU.unit_quat()))
    approach_confs = planner.plan_joint_path(
        target_pose=approach_pose, obstacles=obstacles, resolutions=joint_resolutions)
    conf_path = ConfigurationPath()
    conf_path.append_segment(approach_confs, gripper_state=GRIPPER_OPEN)
    move_to_end_of_configuration_path(planner, conf_path)
    grasp_pose_conf = planner.inverse_kinematics(grasp_pose)
    conf_path.append(grasp_pose_conf, gripper_state=GRIPPER_OPEN)
    conf_path.append_segment([grasp_pose_conf] * 5, gripper_state=GRIPPER_CLOSE)
    move_to_end_of_configuration_path(planner, conf_path)

    # retract to open
    retract_pose = PBU.multiply(grasp_pose, ([prismatic_move_distance, 0, 0], PBU.unit_quat()))
    retract_pose_conf = planner.inverse_kinematics(retract_pose)
    conf_path.append(retract_pose_conf, gripper_state=GRIPPER_CLOSE)
    conf_path.append_pause(5)
    conf_path.append_segment([retract_pose_conf] * 2, gripper_state=GRIPPER_OPEN)

    # retract a bit more to make room for the next motion
    retract_more_pose = PBU.multiply(retract_pose, ([-0.10, 0, 0], PBU.unit_quat()))
    retract_more_pose_conf = planner.inverse_kinematics(retract_more_pose)
    conf_path.append(retract_more_pose_conf, gripper_state=GRIPPER_OPEN)
    return conf_path.interpolate(resolutions=[np.pi / 32] * 6)
    # return conf_path


def plan_skill_grasp(
        planner,
        obstacles,
        grasp_pose,
        reach_distance=0.,
        lift_height=0.,
        grasp_speed=0.05,
        lift_speed=0.05,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
):
    """
    plan skill for opening articulated object with prismatic joint (e.g., drawers)
    Args:
        planner (PlannerRobot): planner
        obstacles (list, tuple): a list obstacle ids
        grasp_pose (tuple): pose for grasping the object
        reach_distance (float): distance for reaching to grasp
        lift_height (float): distance to lift after grasping
        joint_resolutions (list, tuple): motion planning joint-space resolution

    Returns:
        path (CartesianPath)
    """

    reach_pose = PBU.multiply(grasp_pose, ([-reach_distance, 0, 0], PBU.unit_quat()))

    approach_confs = planner.plan_joint_path(
        target_pose=reach_pose, obstacles=obstacles, resolutions=joint_resolutions)
    conf_path = ConfigurationPath()
    conf_path.append_segment(approach_confs, gripper_state=GRIPPER_OPEN)

    grasp_path = configuration_path_to_cartesian_path(planner, conf_path)
    grasp_path.append(grasp_pose, gripper_state=GRIPPER_OPEN)
    grasp_path.append_segment([grasp_pose] * 5, gripper_state=GRIPPER_CLOSE)
    grasp_path = grasp_path.interpolate(pos_resolution=grasp_speed, orn_resolution=np.pi / 8)

    lift_path = CartesianPath()
    lift_path.append(grasp_pose, gripper_state=GRIPPER_CLOSE)
    lift_pose = (grasp_pose[0][:2] + (grasp_pose[0][2] + lift_height,), grasp_pose[1])
    lift_path.append(lift_pose, gripper_state=GRIPPER_CLOSE)
    lift_path = lift_path.interpolate(pos_resolution=lift_speed, orn_resolution=np.pi / 8)

    return grasp_path + lift_path


def plan_skill_place(
        planner,
        obstacles,
        object_target_pose,
        holding,
        retract_distance=0.,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS
):
    """
    plan skill for placing an object at a target pose
    Args:
        planner (PlannerRobot): planner
        obstacles (list, tuple): a list obstacle ids
        object_target_pose (tuple): target pose for placing the object
        holding (int): object id of the object in hand
        retract_distance (float): how far should the gripper retract after placing
        joint_resolutions (list, tuple): motion planning joint-space resolution

    Returns:
        path (CartesianPath)
    """
    grasp_pose = PBU.multiply(PBU.invert(planner.ref_robot.get_eef_position_orientation()), PBU.get_pose(holding))
    target_place_pose = PBU.end_effector_from_body(object_target_pose, grasp_pose)
    # reach_place_pose = PBU.multiply(target_place_pose, ([-0.03, 0, 0], PBU.unit_quat()))

    confs = planner.plan_joint_path(
        target_pose=target_place_pose, obstacles=obstacles, resolutions=joint_resolutions, attachment_ids=(holding,))
    conf_path = ConfigurationPath()
    conf_path.append_segment(confs, gripper_state=GRIPPER_CLOSE)

    place_path = configuration_path_to_cartesian_path(planner, conf_path)
    place_path.append_pause(3)
    place_path.append(target_place_pose, GRIPPER_OPEN)
    place_path.append_pause(3)
    # place_path.append(target_place_pose, GRIPPER_OPEN)
    # place_path.append_pause(2)

    if retract_distance > 0:
        retract_pose = PBU.multiply(target_place_pose, ([-retract_distance, 0, 0], PBU.unit_quat()))
        place_path.append(target_place_pose, GRIPPER_OPEN)
        place_path.append(retract_pose, GRIPPER_OPEN)

    return place_path.interpolate(pos_resolution=0.05, orn_resolution=np.pi / 8)


def plan_skill_pour(
        planner,
        obstacles,
        object_target_pose,
        holding,
        pour_angle_speed=np.pi / 64,
        joint_resolutions=DEFAULT_JOINT_RESOLUTIONS
):
    grasp_pose = PBU.multiply(PBU.invert(planner.ref_robot.get_eef_position_orientation()), PBU.get_pose(holding))
    object_orn = PBU.get_pose(holding)[1]
    target_pose = PBU.end_effector_from_body((object_target_pose[0], object_orn), grasp_pose)
    target_pour_pose = PBU.end_effector_from_body(object_target_pose, grasp_pose)

    confs = planner.plan_joint_path(
        target_pose=target_pose, obstacles=obstacles, resolutions=joint_resolutions, attachment_ids=(holding,))

    conf_path = ConfigurationPath()
    conf_path.append_segment(confs, gripper_state=GRIPPER_CLOSE)

    approach_path = configuration_path_to_cartesian_path(planner, conf_path)
    approach_path = approach_path.interpolate(pos_resolution=0.025, orn_resolution=np.pi / 4)

    pour_path = CartesianPath()
    pour_path.append(target_pose, gripper_state=GRIPPER_CLOSE)
    pour_path.append(target_pour_pose, gripper_state=GRIPPER_CLOSE)
    pour_path = pour_path.interpolate(pos_resolution=0.025, orn_resolution=pour_angle_speed)
    return approach_path + pour_path


class SkillParams(object):
    def __init__(self, low=None, high=None, size=None):
        self._low = None
        self._high = None

    def sample(self, mode=None, low=None, high=None, choices=None, sampler_fn=None):
        raise NotImplementedError

    @property
    def low(self):
        return self._low.copy()

    @property
    def high(self):
        return self._high.copy()

    @property
    def default(self):
        return (self._high - self._low) / 2 + self._low

    @property
    def sample_shape(self):
        return self._low.shape


class SkillParamsContinuous(SkillParams):
    def __init__(self, low=None, high=None, size=None):
        super(SkillParamsContinuous, self).__init__()
        if size is not None:
            self._low = np.zeros(size)  # place holder
        if low is not None:
            self._low = np.array(low)
        assert self._low is not None
        if high is None:
            high = self._low
        self._high = np.array(high)
        assert self._low.shape == self._high.shape
        assert np.all(self._high >= self._low)

    def sample(self, mode='uniform', low=None, high=None, choices=None, sampler_fn=None):
        assert mode in ['uniform', 'normal']
        if low is not None:
            low = np.array(low)
            assert low.shape == self.sample_shape
        else:
            low = self.low
        if high is not None:
            high = np.array(high)
            assert high.shape == self.sample_shape
        else:
            high = self.high
        sample = None
        if sampler_fn is not None:
            sample = sampler_fn()
            assert sample.shape == self.sample_shape
            assert np.all(np.bitwise_and(sample >= self._low, sample <= self._high))
        elif choices is not None:
            for c in choices:
                assert np.array(c).shape == self.sample_shape
            sample = np.array(choices[np.random.randint(low=0, high=len(choices))])
        elif mode == 'uniform':
            sample = np.random.rand(*self.sample_shape) * (high - low) + low
        elif mode == 'normal':
            mu = (high - low) / 2 + low
            sigma = (high - low) / 2
            sample = np.random.randn(*self.sample_shape) * sigma + mu
            sample = np.clip(sample, low, high)
        return sample


class SkillParamsDiscrete(SkillParams):
    def __init__(self, low=None, high=None, size=None):
        super(SkillParamsDiscrete, self).__init__()
        assert low is None
        assert high is None
        assert size is not None
        self._low = np.zeros(size)
        self._high = np.ones(size)

    @property
    def default(self):
        return self.low

    def sample(self, mode=None, low=None, high=None, choices=None, sampler_fn=None):
        if sampler_fn is not None:
            sample_ind = sampler_fn()
            assert 0 <= sample_ind < self.sample_shape[0]
        elif choices is not None:
            choices = np.array(choices)
            assert len(choices.shape) == 1
            assert np.all(np.bitwise_and(choices >= 0, choices < self.sample_shape[0]))  # ensure samples are in-range
            sample_ind = np.random.choice(choices)
        else:
            sample_ind = np.random.randint(low=0, high=self.sample_shape[0])
        sample = np.zeros(self.sample_shape)
        sample[sample_ind] = 1
        return sample


class Skill(object):
    def __init__(
            self,
            params=None,
            name='skill',
            requires_holding=False,
            acquires_holding=False,
            releases_holding=False,
            requires_not_holding=False,
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            verbose=False,
            precondition_fn=None,
    ):
        self.params = params
        if params is None:
            self.params = self.get_default_params()  # dummy
        assert isinstance(self.params, OrderedDict)
        self.planner = None
        self.obstacles = None
        self.env = None
        self.joint_resolutions = joint_resolutions
        self.requires_holding = requires_holding
        self.acquires_holding = acquires_holding
        self.releases_holding = releases_holding
        self.requires_not_holding = requires_not_holding
        if acquires_holding:
            assert self.requires_not_holding
        assert not (self.requires_holding and self.requires_not_holding)
        self.verbose = verbose
        self.precondition_fn = precondition_fn
        self._name = name

    def get_default_params(self):
        return OrderedDict(dummy=SkillParamsContinuous(low=np.zeros(1)))

    @property
    def action_dimension(self):
        return int(np.sum(p.sample_shape[0] for p in self.params.values()))

    @property
    def name(self):
        return self._name

    @property
    def parameters(self):
        return self.params

    @property
    def parameter_ranges(self):
        low = np.concatenate([p.low for p in self.params.values()])
        high = np.concatenate([p.high for p in self.params.values()])
        return low, high

    def skill_params_to_string(self, params, target_object_id):
        """
        Convert a parameter array and target_object_id to human-readable string
        Args:
            params (np.ndarray): serialized skill params
            target_object_id (int): object id of the skill
        Returns:
            msg: str
        """
        assert len(params) == self.action_dimension
        return "{}({})".format(self.name, self.env.objects.body_id_to_name(target_object_id))

    def serialize_skill_param_dict(self, param_dict):
        """
        Serialize a dictionary of parameters to a numpy array

        Args:
            param_dict (dict): named parameters

        Returns:
            params: np.ndarray
        """
        return np.concatenate([param_dict[k] for k in self.params], axis=0)

    def deserialize_skill_param_array(self, params):
        """
        De-serialize a params array to a dictionary of params (assuming ordered by keys of @self.params)

        Args:
            params (np.ndarray): named parameters

        Returns:
            params: OrderedDict
        """
        assert params.shape[0] == self.action_dimension
        params = params.copy()
        param_dict = OrderedDict()
        ind = 0
        for k in self.params:
            param_dict[k] = params[ind: ind + self.params[k].sample_shape[0]]
            ind += self.params[k].sample_shape[0]
        return param_dict

    def plan(self, params, **kwargs):
        raise NotImplementedError

    def precondition_satisfied(self, target_object_id):
        if self.precondition_fn is not None:
            return self.precondition_fn(target_object_id)
        else:
            return True

    def get_skill_params(self, **kwargs):
        raise NotImplementedError

    def get_serialized_skill_params(self, **kwargs):
        param_dict = self.get_skill_params(**kwargs)
        for p in self.params:
            assert p in param_dict
            assert self.params[p].sample_shape == param_dict[p].shape
        return self.serialize_skill_param_dict(param_dict)

    def sample_skill_params(self, **kwargs):
        param_dict = OrderedDict()
        for k in kwargs:
            assert k in list(self.params.keys()), \
                "{} is not a valid skill param, choices are: {}".format(k, self.params.keys())
        for p in self.params:
            sample_kwargs = kwargs.get(p, dict())
            param_dict[p] = self.params[p].sample(**sample_kwargs)
        return param_dict

    def get_default_skill_params(self):
        param_dict = OrderedDict()
        for p in self.params:
            param_dict[p] = self.params[p].default
        return param_dict

    def sample_serialized_skill_params(self, **kwargs):
        param_dict = self.sample_skill_params(**kwargs)
        return self.serialize_skill_param_dict(param_dict)


class GraspDistOrn(Skill):
    def __init__(
            self,
            params=None,
            name="grasp_dist_orn",
            reach_distance=0.05,
            lift_height=0.1,
            lift_speed=0.05,
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            verbose=False,
            precondition_fn=None
    ):
        super(GraspDistOrn, self).__init__(
            params=params,
            name=name,
            acquires_holding=True,
            requires_holding=False,
            releases_holding=False,
            requires_not_holding=True,
            joint_resolutions=joint_resolutions,
            verbose=verbose,
            precondition_fn=precondition_fn
        )
        self.lift_height = lift_height
        self.lift_speed = lift_speed
        self.reach_distance = reach_distance

    def get_default_params(self):
        return OrderedDict(
            grasp_distance=SkillParamsContinuous(size=1),
            grasp_orn=SkillParamsContinuous(size=3)
        )

    def plan(self, params, target_object_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)
        grasp_orn_quat = TU.axisangle2quat(*TU.vec2axisangle(params["grasp_orn"]))
        grasp_pose = compute_grasp_pose(
            PBU.get_pose(target_object_id), grasp_orientation=grasp_orn_quat, grasp_distance=params["grasp_distance"])
        traj= plan_skill_grasp(
            planner=self.planner,
            obstacles=self.obstacles,
            grasp_pose=grasp_pose,
            joint_resolutions=self.joint_resolutions,
            lift_height=self.lift_height,
            lift_speed=self.lift_speed,
            reach_distance=self.reach_distance
        )
        return traj

    def get_skill_params(self, grasp_orn, grasp_distance):
        params = dict(
            grasp_distance=grasp_distance,
            grasp_orn=TU.axisangle2vec(*TU.quat2axisangle(grasp_orn))
        )
        return params


class GraspDistDiscreteOrn(GraspDistOrn):
    def get_default_params(self):
        return OrderedDict(
            grasp_distance=SkillParamsContinuous(size=1),
            grasp_orn=SkillParamsDiscrete(size=len(SKILL_ORIENTATIONS))
        )

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        pose_idx = int(np.argmax(params["grasp_orn"]))
        with np.printoptions(precision=4, suppress=True):
            param_str = "dist={}, orn={}".format(params["grasp_distance"], SKILL_ORIENTATION_NAMES[pose_idx])
        return msg + " " + param_str

    def plan(self, params, target_object_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)
        pose_idx = int(np.argmax(params["grasp_orn"]))
        orn = SKILL_ORIENTATIONS[SKILL_ORIENTATION_NAMES[pose_idx]]
        grasp_pose = compute_grasp_pose(
            PBU.get_pose(target_object_id), grasp_orientation=orn, grasp_distance=params["grasp_distance"])

        traj= plan_skill_grasp(
            planner=self.planner,
            obstacles=self.obstacles,
            grasp_pose=grasp_pose,
            joint_resolutions=self.joint_resolutions,
            lift_height=self.lift_height,
            lift_speed=self.lift_speed,
            reach_distance=self.reach_distance
        )
        return traj

    def get_skill_params(self, grasp_orn_name, grasp_distance):
        orn_index = SKILL_ORIENTATION_NAMES.index(grasp_orn_name)
        orn = np.zeros(len(SKILL_ORIENTATION_NAMES))
        orn[orn_index] = 1
        params = dict(
            grasp_distance=grasp_distance,
            grasp_orn=orn
        )
        return params


class PlacePosOrn(Skill):
    def __init__(
            self,
            params=None,
            name="place_pos_orn",
            retract_distance=0.1,
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            num_pause_steps=0,
            verbose=False,
            precondition_fn=None
    ):
        super(PlacePosOrn, self).__init__(
            params=params,
            name=name,
            requires_holding=True,
            releases_holding=True,
            acquires_holding=False,
            requires_not_holding=False,
            joint_resolutions=joint_resolutions,
            verbose=verbose,
            precondition_fn=precondition_fn
        )
        self.retract_distance = retract_distance
        self.num_pause_steps = num_pause_steps

    def get_default_params(self):
        return OrderedDict(
            place_pos=SkillParamsContinuous(size=3),
            place_orn=SkillParamsContinuous(size=3)
        )

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        with np.printoptions(precision=4, suppress=True):
            param_str = "pos={}, orn={}".format(params["place_pos"], params["place_orn"])
        return msg + " " + param_str

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)
        target_pos = np.array(PBU.get_pose(target_object_id)[0])
        target_pos[2] = PBU.stable_z(holding_id, target_object_id)
        params["place_pos"][2] = max(params["place_pos"][2], 0)  # z clipping
        target_pos += params["place_pos"]
        orn = TU.axisangle2quat(*TU.vec2axisangle(params["place_orn"]))
        place_pose = (target_pos, orn)

        traj = plan_skill_place(
            self.planner,
            obstacles=self.obstacles,
            object_target_pose=place_pose,
            holding=holding_id,
            retract_distance=self.retract_distance,
            joint_resolutions=self.joint_resolutions
        )
        traj.append_pause(self.num_pause_steps)
        return traj

    def get_skill_params(self, place_pos, place_orn):
        params = dict(
            place_pos=place_pos,
            place_orn=TU.axisangle2vec(*TU.quat2axisangle(place_orn))
        )
        return params


class PlacePosDiscreteOrn(PlacePosOrn):
    def get_default_params(self):
        return OrderedDict(
            place_pos=SkillParamsContinuous(size=3),
            place_orn=SkillParamsDiscrete(size=len(SKILL_ORIENTATIONS))
        )

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        orn_idx = int(np.argmax(params["place_orn"]))
        with np.printoptions(precision=4, suppress=True):
            param_str = "pos={}, orn={}".format(params["place_pos"], SKILL_ORIENTATION_NAMES[orn_idx])
        return msg + " " + param_str

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)

        target_pos = np.array(PBU.get_pose(target_object_id)[0])
        target_pos[2] = PBU.stable_z(holding_id, target_object_id)
        params["place_pos"][2] = max(params["place_pos"][2], 0)  # z clipping
        target_pos += params["place_pos"]

        orn_idx = int(np.argmax(params["place_orn"]))
        orn = SKILL_ORIENTATIONS[SKILL_ORIENTATION_NAMES[orn_idx]]
        place_pose = (target_pos, orn)

        traj = plan_skill_place(
            self.planner,
            obstacles=self.obstacles,
            object_target_pose=place_pose,
            holding=holding_id,
            retract_distance=self.retract_distance,
            joint_resolutions=self.joint_resolutions
        )
        traj.append_pause(self.num_pause_steps)
        return traj

    def get_skill_params(self, place_pos, place_orn_name):
        orn_index = SKILL_ORIENTATION_NAMES.index(place_orn_name)
        orn = np.zeros(len(SKILL_ORIENTATION_NAMES))
        orn[orn_index] = 1
        params = dict(
            place_pos=place_pos,
            place_orn=orn
        )
        return params


class PlacePosYawOrn(PlacePosOrn):
    def get_default_params(self):
        return OrderedDict(
            place_pos=SkillParamsContinuous(size=3),
            place_orn=SkillParamsContinuous(size=1)
        )

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        with np.printoptions(precision=4, suppress=True):
            param_str = "pos={}, orn={}".format(params["place_pos"], params["place_orn"])
        return msg + " " + param_str

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)

        target_pos = np.array(PBU.get_pose(target_object_id)[0])
        target_pos[2] = PBU.stable_z(holding_id, target_object_id)
        params["place_pos"][2] = max(params["place_pos"][2], 0)  # z clipping
        target_pos += params["place_pos"]

        yaw = params["place_orn"]
        orn = T.quaternion_about_axis(yaw, axis=(0, 0, 1))

        place_pose = (target_pos, orn)

        traj = plan_skill_place(
            self.planner,
            obstacles=self.obstacles,
            object_target_pose=place_pose,
            holding=holding_id,
            retract_distance=self.retract_distance,
            joint_resolutions=self.joint_resolutions
        )
        traj.append_pause(self.num_pause_steps)
        return traj


class PourPosOrn(Skill):
    def __init__(
            self,
            params=None,
            name="pour_pos_orn",
            pour_angle_speed=np.pi / 64,
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            num_pause_steps=30,
            verbose=False,
            precondition_fn=None
    ):
        super(PourPosOrn, self).__init__(
            params=params,
            name=name,
            requires_holding=True,
            acquires_holding=False,
            releases_holding=False,
            requires_not_holding=False,
            joint_resolutions=joint_resolutions,
            verbose=verbose,
            precondition_fn=precondition_fn
        )
        self.pour_angle_speed = pour_angle_speed
        self.num_pause_steps = num_pause_steps

    def get_default_params(self):
        return OrderedDict(
            pour_pos=SkillParamsContinuous(size=3),
            pour_orn=SkillParamsContinuous(size=3)
        )

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)

        target_pos = np.array(PBU.get_pose(target_object_id)[0])
        target_pos += params["pour_pos"]

        orn = TU.axisangle2quat(*TU.vec2axisangle(params["pour_orn"]))
        pour_pose = (target_pos, orn)

        traj = plan_skill_pour(
            self.planner,
            obstacles=self.obstacles,
            object_target_pose=pour_pose,
            holding=holding_id,
            joint_resolutions=self.joint_resolutions,
            pour_angle_speed=self.pour_angle_speed
        )
        traj.append_pause(self.num_pause_steps)
        return traj

    def get_skill_params(self, pour_pos, pour_orn):
        params = dict(
            pour_pos=pour_pos,
            pour_orn=TU.axisangle2vec(*TU.quat2axisangle(pour_orn))
        )
        return params


class PourPosAngle(PourPosOrn):
    def get_default_params(self):
        return OrderedDict(
            pour_pos=SkillParamsContinuous(size=3),
            pour_angle=SkillParamsContinuous(size=1)
        )

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        with np.printoptions(precision=4, suppress=True):
            param_str = "pos={}, angle={}".format(params["pour_pos"], -params["pour_angle"])
        return msg + " " + param_str

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)

        target_pos = np.array(PBU.get_pose(target_object_id)[0])
        pour_pos = target_pos + params["pour_pos"]

        cur_orn = np.array(PBU.get_pose(holding_id)[1])
        angle = -params["pour_angle"]
        xy_vec = target_pos[:2] - pour_pos[:2]
        perp_xy_vec = np.array([xy_vec[1], -xy_vec[0], 0])

        rot = T.quaternion_about_axis(angle, perp_xy_vec)
        orn = T.quaternion_multiply(rot, cur_orn)

        pour_pose = (pour_pos, orn)

        traj = plan_skill_pour(
            self.planner,
            obstacles=self.obstacles,
            object_target_pose=pour_pose,
            holding=holding_id,
            joint_resolutions=self.joint_resolutions,
            pour_angle_speed=self.pour_angle_speed
        )
        traj.append_pause(self.num_pause_steps)
        return traj

    def get_skill_params(self, pour_pos, pour_angle):
        params = OrderedDict(
            pour_pos=pour_pos,
            pour_angle=pour_angle
        )
        return params


class OperatePrismaticPosDistance(Skill):
    def __init__(
            self,
            params=None,
            name="operate_prismatic_pos_distance",
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            num_pause_steps=0,
            verbose=False,
            precondition_fn=None
    ):
        super(OperatePrismaticPosDistance, self).__init__(
            params=params,
            name=name,
            requires_holding=False,
            acquires_holding=False,
            releases_holding=False,
            requires_not_holding=True,
            joint_resolutions=joint_resolutions,
            verbose=verbose,
            precondition_fn=precondition_fn
        )
        self.num_pause_steps = num_pause_steps

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        with np.printoptions(precision=4, suppress=True):
            param_str = "pos={}, move_distance={}".format(params["grasp_pos"], params["prismatic_move_distance"])
        return msg + " " + param_str

    def get_default_params(self):
        return OrderedDict(
            grasp_pos=SkillParamsContinuous(size=3),
            prismatic_move_distance=SkillParamsContinuous(size=1)
        )

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)
        delta_pos = params["grasp_pos"]
        distance = params["prismatic_move_distance"]

        target_pos = np.array(PBU.get_pose(target_object_id)[0])
        target_pos += delta_pos

        grasp_pose = (target_pos, DRAWER_GRASP_ORN)

        traj = plan_skill_open_prismatic(
            self.planner,
            obstacles=self.obstacles,
            grasp_pose=grasp_pose,
            prismatic_move_distance=distance,
            reach_distance=0.05,
            joint_resolutions=self.joint_resolutions,
        )
        traj.append_pause(self.num_pause_steps)
        return traj

    def get_skill_params(self, grasp_pos, prismatic_move_distance):
        params = OrderedDict(
            grasp_pos=grasp_pos,
            prismatic_move_distance=prismatic_move_distance
        )
        return params


class TouchPosition(Skill):
    def __init__(
            self,
            params=None,
            name="touch_position",
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            num_pause_steps=0,
            verbose=False,
            precondition_fn=None
    ):
        super(TouchPosition, self).__init__(
            params=params,
            name=name,
            requires_holding=False,
            acquires_holding=False,
            releases_holding=False,
            requires_not_holding=True,
            joint_resolutions=joint_resolutions,
            verbose=verbose,
            precondition_fn=precondition_fn
        )
        self.num_pause_steps = num_pause_steps

    def skill_params_to_string(self, params, target_object_id):
        msg = Skill.skill_params_to_string(self, params, target_object_id)
        params = self.deserialize_skill_param_array(params)
        with np.printoptions(precision=4, suppress=True):
            param_str = "pos={}".format(params["touch_pos"])
        return msg + " " + param_str

    def get_default_params(self):
        return OrderedDict(
            touch_pos=SkillParamsContinuous(size=3)
        )

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        params = self.deserialize_skill_param_array(params)
        delta_pos = params["touch_pos"]

        target_pose = PBU.get_pose(target_object_id)
        target_pose = PBU.multiply(target_pose, (delta_pos, ALL_ORIENTATIONS["top"]))

        traj = plan_skill_touch(
            self.planner,
            obstacles=self.obstacles,
            target_pose=target_pose,
            reach_distance=0.1,
            joint_resolutions=self.joint_resolutions,
        )
        traj.append_pause(self.num_pause_steps)
        return traj

    def get_skill_params(self, touch_pos):
        params = OrderedDict(
            touch_pos=touch_pos,
        )
        return params


class ConditionSkill(Skill):
    def __init__(
            self,
            name="condition",
            joint_resolutions=DEFAULT_JOINT_RESOLUTIONS,
            verbose=False,
            precondition_fn=None
    ):
        super(ConditionSkill, self).__init__(
            name=name,
            requires_holding=False,
            acquires_holding=False,
            releases_holding=False,
            requires_not_holding=False,
            joint_resolutions=joint_resolutions,
            verbose=verbose,
            precondition_fn=precondition_fn
        )

    def plan(self, params, target_object_id=None, holding_id=None):
        assert len(params) == self.action_dimension
        return CartesianPath(arm_path=[], gripper_path=[])

    def get_skill_params(self, **kwargs):
        return OrderedDict(
            dummy=np.zeros(1)
        )


class SkillLibrary(object):
    def __init__(self, env, planner, obstacles, skills, verbose=False):
        self.planner = planner
        self.obstacles = obstacles
        self.env = env
        self._skills = skills
        assert len(set(self.skill_names)) == len(self.skill_names)  # make sure all skill names are unique
        for s in self.skills:
            s.planner = planner
            s.obstacles = obstacles
            s.env = env
            s.verbose = verbose
        self._holding = None

    def sub_library(self, names):
        skills = [s for s in self.skills if s.name in names]
        return self.__class__(env=self.env, planner=self.planner, obstacles=self.obstacles, skills=skills)

    @property
    def skills(self):
        return self._skills

    def reset(self):
        self._holding = None

    @property
    def skill_names(self):
        return [s.name for s in self._skills]

    def __len__(self):
        return len(self._skills)

    def name_to_skill_index(self, name):
        return self.skill_names.index(name)

    def skill_index_to_array(self, skill_index):
        arr = np.zeros(len(self))
        arr[skill_index] = 1
        return arr

    @property
    def holding(self):
        return self._holding

    @property
    def action_dimension(self):
        return int(len(self.skills) + np.sum([s.action_dimension for s in self.skills]))

    def skill_params_to_string(self, params, target_object_id):
        skill_index, skill_params = self._parse_serialized_skill_params(params)
        return self.skills[skill_index].skill_params_to_string(skill_params, target_object_id=target_object_id)

    def _parse_serialized_skill_params(self, all_params):
        assert len(all_params) == self.action_dimension
        skill_index = int(np.argmax(all_params[:len(self.skills)]))
        assert skill_index < len(self.skills)
        ind = 0
        params = all_params[len(self.skills):]
        for i, s in enumerate(self.skills):
            if i == skill_index:
                return skill_index, params[ind:ind + s.action_dimension]
            else:
                ind += s.action_dimension
        return None

    def _parse_skill_param_dict(self, all_params):
        """
        parse skill parameters dictionary

        Args:
            all_params (dict): dict that maps "skill_name | param_name" to params

        Returns:
            skill_index: int
            skill_params: dict
        """
        skill_index = int(all_params["skill_index"].argmax())
        skill = self.skills[skill_index]
        skill_params = OrderedDict()
        for k in all_params:
            skill_name, param_name = k.split('|')
            if skill_name == skill.name:
                assert param_name in skill.parameters
                skill_params[param_name] = all_params[k]
        return skill_index, skill_params

    def get_serialized_skill_params(self, skill_name, **kwargs):
        params = np.zeros(self.action_dimension)
        skill_index = self.name_to_skill_index(skill_name)
        skill_params = self.skills[skill_index].get_serialized_skill_params(**kwargs)
        params[skill_index] = 1

        ind = 0
        for i, s in enumerate(self.skills):
            if i == skill_index:
                params[len(self.skills) + ind: len(self.skills) + ind + s.action_dimension] = skill_params
            else:
                ind += s.action_dimension

        return params

    def sample_serialized_skill_params(self, skill_name, **kwargs):
        params = np.zeros(self.action_dimension)
        skill_index = self.name_to_skill_index(skill_name)
        skill_params = self.skills[skill_index].sample_serialized_skill_params(**kwargs)
        params[skill_index] = 1

        ind = 0
        for i, s in enumerate(self.skills):
            if i == skill_index:
                params[len(self.skills) + ind: len(self.skills) + ind + s.action_dimension] = skill_params
            else:
                ind += s.action_dimension

        return params

    def flatten_skill_param_key(self, skill_name, skill_params):
        new_params = OrderedDict()
        for p in skill_params:
            new_key = "{}|{}".format(skill_name, p)
            new_params[new_key] = skill_params[p]
        return new_params

    def get_skill_params(self, skill_name, **kwargs):
        param_dict = OrderedDict()
        for skill in self.skills:
            if skill.name == skill_name:
                param_dict.update(self.flatten_skill_param_key(skill.name, skill.get_skill_params(**kwargs)))
            else:
                param_dict.update(self.flatten_skill_param_key(skill.name, skill.get_default_skill_params()))  # dummy
        skill_index = self.skill_names.index(skill_name)
        skill_index_arr = np.zeros(len(self.skills))
        skill_index_arr[skill_index] = 1
        param_dict["skill_index"] = skill_index_arr
        return param_dict

    def sample_skill_params(self, skill_name, **kwargs):
        param_dict = OrderedDict()
        for skill in self.skills:
            if skill.name == skill_name:
                param_dict.update(self.flatten_skill_param_key(skill.name, skill.sample_skill_params(**kwargs)))
            else:
                param_dict.update(self.flatten_skill_param_key(skill.name, skill.get_default_skill_params()))  # dummy
        skill_index = self.skill_names.index(skill_name)
        skill_index_arr = np.zeros(len(self.skills))
        skill_index_arr[skill_index] = 1
        param_dict["skill_index"] = skill_index_arr
        return param_dict

    def deserialize_skill_params(self, all_params):
        """
        Deserialize a skill param array to a dictionary of skill params
        Args:
            all_params (np.ndarray): all skill params

        Returns:
            an OrderedDict that maps "skill_name | param_name" to params
        """
        assert all_params.shape[0] == self.action_dimension
        param_dict = OrderedDict()

        skill_index = int(np.argmax(all_params[:len(self.skills)]))
        assert skill_index < len(self.skills)
        ind = 0
        params = all_params[len(self.skills):]
        for i, skill in enumerate(self.skills):
            skill_params = skill.deserialize_skill_param_array(params[ind: ind + skill.action_dimension])
            param_dict.update(self.flatten_skill_param_key(skill.name, skill_params))
            ind += skill.action_dimension
        skill_index_arr = np.zeros(len(self.skills))
        skill_index_arr[skill_index] = 1
        param_dict["skill_index"] = skill_index_arr
        return param_dict

    def get_skill_param_dict_metadata(self, param_dict):
        """
        Get masks for skill param dict

        Args:
            param_dict (dict): an OrderedDict that maps "skill_name | param_name" to params

        Returns:
            an OrderedDict that maps "skill_name | param_name | mask / type " to an integer
        """
        meta = OrderedDict()
        skill_index = int(param_dict["skill_index"].argmax())
        target_skill_name = self.skills[skill_index].name
        for p, v in param_dict.items():
            if p == "skill_index":
                continue
            skill_name, param_name = p.split("|")
            meta["{}|mask".format(p)] = np.ones(1) if skill_name == target_skill_name else np.zeros(1)
            skill = self.skills[self.skill_names.index(skill_name)]
            skill_is_cont = isinstance(skill.parameters[param_name], SkillParamsContinuous)
            meta["{}|type".format(p)] = np.ones(1) if skill_is_cont else np.zeros(1)
        return meta

    def get_serialized_skill_param_mask(self, all_params):
        """
        Get a binary mask for the valid skill parameters (useful for network training).

        Args:
            all_params (np.ndarray): all parameters, including the prepended skill index

        Returns:
            masks, excluding dimension for skill index

        """
        assert all_params.shape[0] == self.action_dimension
        masks = np.zeros(self.action_dimension - len(self.skills))
        skill_index = int(np.argmax(all_params[:len(self.skills)]))
        assert skill_index < len(self.skills)
        ind = 0
        for i, s in enumerate(self.skills):
            if i == skill_index:
                masks[ind: ind + s.action_dimension] = 1
            ind += s.action_dimension
        return masks

    def plan(self, params, target_object_id):
        if isinstance(params, dict):
            skill_index, skill_params = self._parse_skill_param_dict(params)
            skill_params = self.skills[skill_index].serialize_skill_param_dict(skill_params)
        else:
            skill_index, skill_params = self._parse_serialized_skill_params(params)
        skill = self.skills[skill_index]
        if not skill.precondition_satisfied(target_object_id=target_object_id):
            raise PreconditionNotSatisfied("Precondition for skill '{}' is not satisfied".format(skill.name))

        if skill.requires_holding:
            if self._holding is None:
                raise NoPlanException("Robot is not holding anything but is trying to run {}".format(skill.name))
            if self._holding == target_object_id:
                raise NoPlanException("Applying skill {} to the object that is being held".format(skill.name))
            traj = skill.plan(skill_params, target_object_id=target_object_id, holding_id=self._holding)
            if skill.releases_holding:
                self._holding = None
        elif skill.acquires_holding:
            if self._holding is not None:
                raise NoPlanException("Robot is holding something but is trying to run {}".format(skill.name))
            traj = skill.plan(skill_params, target_object_id=target_object_id)
            self._holding = target_object_id
        else:
            if skill.requires_not_holding and self._holding is not None:
                raise NoPlanException("Robot is holding something but is trying to run {}".format(skill.name))
            traj = skill.plan(skill_params, target_object_id=target_object_id)
        return traj
