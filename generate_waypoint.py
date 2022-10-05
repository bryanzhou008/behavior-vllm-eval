import os
import cv2
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from scipy.spatial import distance
import pdb

import numpy as np
from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
from igibson.simulator import Simulator

IMAGE_HEIGHT = 720
IMAGE_WIDTH = 1024
MAX_NUM_FRAMES = 10000
FRAME_BATCH_SIZE = 5


class GenerateDataset(object):
    def __init__(self):
        self.sim = Simulator(
            image_height=IMAGE_HEIGHT,
            image_width=IMAGE_WIDTH,
            mode='headless',
        )

        scene = InteractiveIndoorScene(scene_id="Ihlen_1_int")
        self.sim.import_scene(scene)
        self.sim.scene.open_all_doors()
        self.floor = self.sim.scene.get_random_floor()
        self.check_points = []

        for room_instance in self.sim.scene.room_ins_name_to_ins_id:
            lower, upper = self.sim.scene.get_aabb_by_room_instance(room_instance) #Axis Aligned Bounding Box
            x_cord, y_cord, z_cord = (upper - lower)/2 + lower
            self.check_points.append((x_cord, y_cord))

    def prepare_spline_functions(self, shortest_path):
        self.spline_functions = []
        path_length = len(shortest_path)
        self.spline_functions.append(CubicSpline(
            range(path_length), shortest_path[:, 0], bc_type='clamped'))
        self.spline_functions.append(CubicSpline(
            range(path_length), shortest_path[:, 1], bc_type='clamped'))

    def get_interpolated_steps(self, step):
        curr_x, curr_y = self.spline_functions[0](step), self.spline_functions[1](step)
        next_x, next_y = self.spline_functions[0](step + 1), self.spline_functions[1](step + 1)
        dist_to_next_step = distance.euclidean((curr_x, curr_y), (next_x, next_y))
        path_length = int(100 * dist_to_next_step)
        interpolated_points = []
        for i in range(path_length - 1):
            curr_step = step + (1.0/path_length*i)
            interpolated_points.append([self.spline_functions[0](
                curr_step), self.spline_functions[1](curr_step)])
        return np.array(interpolated_points)

    def generate(self):
        # source, target, camera_up
        # TODO: Generate New Waypoints
        check_points = self.check_points
        shortest_path = []
        avg_distance = 0

        for i in range(1, len(check_points)):
            steps = self.sim.scene.get_shortest_path(self.floor, check_points[i-1][:2], check_points[i][:2], True)[0]
            for j in range(len(steps)-1):
                step = steps[j]
                shortest_path.append(step)
                # next_step = steps[j+1]
                # temp_dist = distance.euclidean(step, next_step)
                # dist_threshold = 0.147
                # avg_distance += dist_threshold
                # if temp_dist > dist_threshold:
                #     interpolation_num = int (temp_dist // dist_threshold)
                #     x_diff = (next_step[0] - step[0]) / interpolation_num
                #     y_diff = (next_step[1] - step[1]) / interpolation_num
                #     for k in range(interpolation_num):
                #         shortest_path.append([step[0] + k*x_diff, step[1] + k*y_diff])
                # else:
                #     shortest_path.append(step)
        shortest_path = np.array(shortest_path)
        print(avg_distance/len(shortest_path))
        self.prepare_spline_functions(shortest_path)

        steps = []
        for i in range(len(shortest_path)):
            for step in self.get_interpolated_steps(i):
                steps.append(step)
        steps = np.array(steps)
        # plt.plot(steps[:, 0], steps[:, 1], color ='tab:blue',  marker='o')
        # plt.xlabel("x")
        # plt.ylabel("y")
        # plt.show()
    
        for i in range(len(steps)-20):
            step = steps[i]
            # pdb.set_trace()
            next_step = np.average(steps[i+1:i+5], axis=0)

            x, y, z = step[0], step[1], 0.8
            tar_x, tar_y, tar_z = next_step[0], next_step[1], 0.8

            self.sim.renderer.set_camera(
                [x, y, z], [tar_x, tar_y, tar_z], [0, 0, 1])
            frames = self.sim.renderer.render(modes=("rgb", "3d"))

            # Render 3d points as depth map
            depth = np.linalg.norm(frames[1][:, :, :3], axis=2)
            depth /= depth.max()
            frames[1][:, :, :3] = depth[..., None]

            self.sim.step()
            cv2.imshow("test", cv2.cvtColor(frames[0], cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)


    def disconnect_simulator(self):
        self.sim.disconnect()

path = os.path.join(os.getcwd(), "data.hdf5")
dataset_generator = GenerateDataset()
dataset_generator.generate()
dataset_generator.disconnect_simulator()
