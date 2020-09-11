import cv2
import sys
import os
import numpy as np
from gibson2.core.render.mesh_renderer.mesh_renderer_cpu import MeshRenderer
from gibson2.core.render.profiler import Profiler
from gibson2.utils.assets_utils import get_model_path
import gibson2
import time

def main():
    global _mouse_ix, _mouse_iy, down, view_direction

    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        model_path = os.path.join(get_model_path('Rs'), 'mesh_z_up.obj')

    for interval in [3,2,1,0.5,0.4,0.3]:
        renderer = MeshRenderer(width=512, height=512, optimize=False)
        renderer.load_object(model_path)
        renderer.add_instance(0)

        renderer.load_object(os.path.join(gibson2.assets_path, 'models/ycb/002_master_chef_can/textured_simple.obj'))
        for i in np.arange(-2,2,interval):
            for j in np.arange(-2,2,interval):
                renderer.add_instance(1)
                renderer.instances[-1].set_position([i,j,0.5])



        renderer.load_object(os.path.join(gibson2.assets_path, 'models/ycb/003_cracker_box/textured_simple.obj'))
        for i in np.arange(-2,2,interval):
            for j in np.arange(-2,2,interval):
                renderer.add_instance(2)
                renderer.instances[-1].set_position([i,j,0.8])

        if renderer.optimize:
            renderer.optimize_vertex_and_texture()

        print(renderer.visual_objects, renderer.instances)
        print(renderer.materials_mapping, renderer.mesh_materials)
        camera_pose = np.array([0, 0, 1.2])
        view_direction = np.array([1, 0, 0])
        renderer.set_camera(camera_pose, camera_pose + view_direction, [0, 0, 1])
        renderer.set_fov(90)

        # px = 0
        # py = 0

        # _mouse_ix, _mouse_iy = -1, -1
        # down = False

        # def change_dir(event, x, y, flags, param):
        #     global _mouse_ix, _mouse_iy, down, view_direction
        #     if event == cv2.EVENT_LBUTTONDOWN:
        #         _mouse_ix, _mouse_iy = x, y
        #         down = True
        #     if event == cv2.EVENT_MOUSEMOVE:
        #         if down:
        #             dx = (x - _mouse_ix) / 100.0
        #             dy = (y - _mouse_iy) / 100.0
        #             _mouse_ix = x
        #             _mouse_iy = y
        #             r1 = np.array([[np.cos(dy), 0, np.sin(dy)], [0, 1, 0], [-np.sin(dy), 0, np.cos(dy)]])
        #             r2 = np.array([[np.cos(-dx), -np.sin(-dx), 0], [np.sin(-dx), np.cos(-dx), 0], [0, 0, 1]])
        #             view_direction = r1.dot(r2).dot(view_direction)
        #     elif event == cv2.EVENT_LBUTTONUP:
        #         down = False

        # cv2.namedWindow('test')
        # cv2.setMouseCallback('test', change_dir)

        # while True:
        #     with Profiler('Render'):
        #         frame = renderer.render(modes=('rgb', 'normal', '3d'))
        #     cv2.imshow('test', cv2.cvtColor(np.concatenate(frame, axis=1), cv2.COLOR_RGB2BGR))
        #     q = cv2.waitKey(1)
        #     if q == ord('w'):
        #         px += 0.05
        #     elif q == ord('s'):
        #         px -= 0.05
        #     elif q == ord('a'):
        #         py += 0.05
        #     elif q == ord('d'):
        #         py -= 0.05
        #     elif q == ord('q'):
        #         break
        #     camera_pose = np.array([px, py, 1.2])
        #     renderer.set_camera(camera_pose, camera_pose + view_direction, [0, 0, 1])

        start = time.time()

        for i in range(100):
            frame = renderer.render(modes=('rgb'))

        elapsed = time.time() - start
        print("num objects {} fps {}".format(len(renderer.instances), 100/elapsed))

        renderer.release()


if __name__ == '__main__':
    main()