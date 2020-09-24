from copy import deepcopy
import numpy as np

import gibson2.external.pybullet_tools.transformations as T
import pybullet as p
from gibson2.core.physics.interactive_objects import InteractiveObj, YCBObject, Object, VisualMarker
import gibson2.external.pybullet_tools.utils as PBU
import gibson2.envs.kitchen.env_utils as EU


class Faucet(Object):
    def __init__(
            self,
            num_beads=20,
            dispense_freq=1,
            dispense_height=0.3,
            base_color=(0.75, 0.75, 0.75, 1),
            beads_color=(0, 0, 1, 1),
            beads_size=0.015
    ):
        self._dispense_freq = dispense_freq
        self._beads = []
        self._next_bead_index = 0
        self._n_step_since = 0
        self._base_color = base_color
        self._beads_color = beads_color
        self._beads_size = beads_size
        self._num_beads = num_beads
        self._dispense_position = np.array([0, 0, dispense_height])
        super(Faucet, self).__init__()

    @property
    def beads(self):
        return deepcopy(self._beads)

    def load(self):
        self.body_id = PBU.create_box(0.15, 0.15, 0.01, mass=100, color=self._base_color)
        self._beads = [PBU.create_sphere(
            self._beads_size, mass=PBU.STATIC_MASS, color=self._beads_color
        ) for _ in range(self._num_beads)]
        self.loaded = True

    def reset(self):
        self._next_bead_index = 0
        for i, b in enumerate(self._beads):
            p.resetBasePositionAndOrientation(b, self.get_position() + np.array([0, 0, 10 + b * 0.1]), PBU.unit_quat())
            p.changeDynamics(b, -1, mass=PBU.STATIC_MASS)

        self._n_step_since = 0

    def _try_dispense(self, task_objs):
        if self._next_bead_index == self._num_beads:
            return False
        bid = self._beads[self._next_bead_index]
        prev_pose = PBU.get_pose(bid)
        PBU.set_pose(bid, (self.get_position() + self._dispense_position, PBU.unit_quat()))
        for oid in [o.body_id for o in task_objs] + self._beads:
            if oid != bid and PBU.body_collision(oid, bid):
                PBU.set_pose(bid, prev_pose)
                return False
        p.changeDynamics(bid, -1, mass=0.3)
        self._next_bead_index += 1
        return True

    def step(self, task_objs, gripper=None):
        should_dispense = False
        for o in task_objs:
            if o.body_id == self.body_id:
                continue
            center_place = PBU.is_center_stable(o.body_id, self.body_id, above_epsilon=0.01, below_epsilon=0.02)
            in_contact = PBU.body_collision(self.body_id, o.body_id)
            should_dispense = should_dispense or (center_place and in_contact)
        if should_dispense and self._n_step_since >= self._dispense_freq:
            self._try_dispense(task_objs)
            self._n_step_since = 0
        else:
            self._n_step_since += 1


class CoffeeMachine(Faucet):
    def __init__(
            self,
            filename,
            beans_set,
            num_beans_trigger=5,
            num_coffee_beads=20,
            dispense_freq=1,
            dispense_position=(0, 0, 0),
            platform_position=(0, 0, 0),
            beads_color=(1, 1, 1, 1),
            beads_size=0.01,
            button_pose=((0, 0, 0), PBU.unit_quat()),
            scale=1.
    ):

        super(CoffeeMachine, self).__init__(
            num_beads=num_coffee_beads,
            dispense_freq=dispense_freq,
            beads_color=beads_color,
            beads_size=beads_size
        )
        self._file_path = filename
        self._beans_set = beans_set
        self._num_beans_trigger = num_beans_trigger
        self._dispense_position = np.array(dispense_position)
        self._platform_position = np.array(platform_position)
        self._scale = scale
        self._button_pose = button_pose
        self._should_dispense = False
        self.platform = None
        self.button = None

    def reset(self):
        super(CoffeeMachine, self).reset()
        self._sync_parts()
        self._should_dispense = False

    def _sync_parts(self):
        p.resetBasePositionAndOrientation(
            self.platform.body_id, self._platform_position + np.array(self.get_position()), PBU.unit_quat())
        PBU.multiply(self._button_pose, self.get_position_orientation())
        p.resetBasePositionAndOrientation(
            self.button.body_id, *PBU.multiply(self.get_position_orientation(), self._button_pose)
        )

    def load(self):
        self.body_id = p.loadURDF(
            self._file_path, globalScaling=self._scale, flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        self.platform = VisualMarker(visual_shape=p.GEOM_BOX, half_extents=[0.05, 0.05, 0.005], rgba_color=(0.8, 0.8, 0.8, 0.1))
        self.platform.load()
        # self.button = VisualMarker(visual_shape=p.GEOM_CYLINDER, radius=0.03, length=0.008, rgba_color=(0, 1, 0, 1))
        self.button = Cylinder(radius=0.03, height=0.008, color=(0, 1, 0, 1), mass=PBU.STATIC_MASS)
        self.button.load()
        self._beads = [PBU.create_sphere(
            self._beads_size, mass=PBU.STATIC_MASS, color=self._beads_color
        ) for _ in range(self._num_beads)]
        self.loaded = True

    def step(self, task_objs, gripper=None):
        self._sync_parts()
        beans = EU.objects_center_in_container(self._beans_set, self.body_id)

        # needs to be in the funnel
        # beans = [bid for bid in beans if PBU.get_pose(bid)[0][2] > self.get_position()[2] - 0.05]

        # start dispensing when button is pressed
        self._should_dispense = self._should_dispense or PBU.body_collision(gripper.body_id, self.button.body_id)
        # stop when no more beans in the machine
        self._should_dispense = self._should_dispense and len(beans) > 0

        if self._should_dispense and self._n_step_since >= self._dispense_freq:
            if self._try_dispense(task_objs):
                p.resetBasePositionAndOrientation(
                    beans[0], np.array([0, 0, 10 + self._next_bead_index * 0.1]), PBU.unit_quat())
                p.changeDynamics(beans[0], -1, mass=PBU.STATIC_MASS)
                self._n_step_since = 0
        else:
            self._n_step_since += 1


class Box(Object):
    def __init__(self, color=(0, 1, 0, 1), size=(0.15, 0.15, 0.01), mass=100):
        super(Box, self).__init__()
        self._color = color
        self._size = size
        self._mass = mass

    def load(self):
        self.body_id = PBU.create_box(*self._size, mass=self._mass, color=self._color)
        self.loaded = True


class Cylinder(Object):
    def __init__(self, color=(0, 1, 0, 1), radius=0.1, height=0.1, mass=100):
        super(Cylinder, self).__init__()
        self._color = color
        self._radius = radius
        self._height = height
        self._mass = mass

    def load(self):
        self.body_id = PBU.create_cylinder(radius=self._radius, height=self._height, mass=self._mass, color=self._color)
        self.loaded = True


class MessyPlate(Box):
    def __init__(self, color=(0, 1, 0, 1), size=(0.15, 0.15, 0.01), mass=100, num_stuff=5, stuff_size=(0.01, 0.01, 0.01)):
        super(MessyPlate, self).__init__(color=color, size=size, mass=mass)
        self.num_stuff = num_stuff
        self.stuff_size = stuff_size
        self._stuff = []

    @property
    def stuff(self):
        return deepcopy(self._stuff)

    def load(self):
        super(MessyPlate, self).load()
        for i in range(self.num_stuff):
            color = np.random.random(4)
            color[3] = 1
            self._stuff.append(PBU.create_box(*self.stuff_size, mass=0.01, color=color))

    def reset(self):
        for bid in self._stuff:
            PBU.sample_placement(top_body=bid, bottom_body=self.body_id)


class Tube(Object):
    def __init__(self, color=(0, 1, 0, 1), size=(0.5, 0.1, 0.1), width=0.01, mass=1.):
        super(Tube, self).__init__()
        self._color = color
        self._size = size
        self._width = width
        self._mass = mass

    def load(self):
        self.loaded = True
        l, w, h = self._size

        bottom_col, bottom_vir = PBU.create_shape(
            PBU.get_box_geometry(l, w, self._width), color=self._color)

        left_col, left_vir = PBU.create_shape(
            PBU.get_box_geometry(l, self._width, h), color=self._color)

        right_col, right_vir = PBU.create_shape(
            PBU.get_box_geometry(l, self._width, h), color=self._color)

        top_col, top_vir = PBU.create_shape(
            PBU.get_box_geometry(l, w, self._width), color=self._color)

        masses = [self._mass / 4] * 3
        col_indices = (left_col, right_col, top_col)
        vir_indices = (left_vir, right_vir, top_vir)
        positions = [
            (0, -w / 2, h / 2),
            (0, w / 2, h / 2),
            (0, 0, h),
        ]
        orns = [PBU.unit_quat()] * 3

        self.body_id = p.createMultiBody(
            baseMass=self._mass / 4,
            baseCollisionShapeIndex=bottom_col,
            baseVisualShapeIndex=bottom_vir,
            basePosition=PBU.unit_point(),
            baseOrientation=PBU.unit_quat(),
            baseInertialFramePosition=PBU.unit_point(),
            baseInertialFrameOrientation=PBU.unit_quat(),
            linkMasses=masses,
            linkCollisionShapeIndices=col_indices,
            linkVisualShapeIndices=vir_indices,
            linkPositions=positions,
            linkOrientations=orns,
            linkInertialFramePositions=[PBU.unit_point()] * 3,
            linkInertialFrameOrientations=orns,
            linkParentIndices=[0, 0, 0],
            linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED],
            linkJointAxis=[PBU.unit_point()] * 3
        )


class Hook(Object):
    def __init__(self, width, length1, length2, color=(0, 1, 0, 1)):
        super(Hook, self).__init__()
        self._width = width
        self._length1 = length1
        self._length2 = length2
        self._color = color

    def load(self):
        self.loaded = True

        collision_id1, visual_id1 = PBU.create_shape(
            PBU.get_box_geometry(self._length1, self._width, self._width), color=self._color)
        collision_id2, visual_id2 = PBU.create_shape(
            PBU.get_box_geometry(self._length2, self._width, self._width), color=self._color)
        self.body_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=collision_id1,
            baseVisualShapeIndex=visual_id1,
            basePosition=PBU.unit_point(),
            baseOrientation=PBU.unit_quat(),
            baseInertialFramePosition=PBU.unit_point(),
            baseInertialFrameOrientation=PBU.unit_quat(),
            linkMasses=(0.5,),
            linkCollisionShapeIndices=[collision_id2],
            linkVisualShapeIndices=[visual_id2],
            linkPositions=[(-self._length1 / 2 + self._width / 2, self._length2 / 2 - self._width / 2, 0)],
            linkOrientations=[T.quaternion_from_euler(0, 0, np.pi / 2)],
            linkInertialFramePositions=[(0, 0, 0)],
            linkInertialFrameOrientations=[PBU.unit_quat()],
            linkParentIndices=[0],
            linkJointTypes=[p.JOINT_FIXED],
            linkJointAxis=[[0, 0, 0]]
        )