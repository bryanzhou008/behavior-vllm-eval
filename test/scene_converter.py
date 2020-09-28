#!/usr/bin/env python

from gibson2.utils.utils import parse_config
import os
import gibson2

from gibson2.utils.assets_utils import get_ig_scene_path, get_ig_category_path

import argparse
import xml.etree.ElementTree as ET
import json
import numpy as np
import math


config = parse_config(os.path.join(gibson2.root_path, '../test/test.yaml'))
missing_models = set([
    'exercise_equipment',
    'curtain',
    'garbage_compressor',
    'staircase',
    'closet',
    'ac_engine',
    'water_hearter',
])


def convert_scene(scene_name, select_best=False):

    scene_file = get_ig_scene_path(
        scene_name) + "/" + scene_name + "_orig.urdf"
    scene_tree = ET.parse(scene_file)
    bbox_dir = os.path.join(get_ig_scene_path(scene_name), "bbox")
    os.makedirs(bbox_dir, exist_ok=True)

    print(scene_file)

    with open(get_ig_scene_path(scene_name) + '/misc/all_objs.json', 'r') as all_objs_file:
        all_objs = json.load(all_objs_file)

        total = 0
        categories = []
        for c, obj in enumerate(all_objs):
            obj_category = obj["category"].lower()
            # print('obj_category:', obj_category)
            if obj_category in missing_models:
                print("We don't have yet models of ", obj_category)
                continue

            total += 1

            link_name = obj_category + "_" + str(c)

            if obj_category not in categories:
                categories += [obj_category]

            edge_x = obj['edge_x']
            bbox_x = np.linalg.norm(edge_x)
            edge_y = obj['edge_y']
            bbox_y = np.linalg.norm(edge_y)
            z_bbox_coords = obj['z']
            bbox_z = (z_bbox_coords[1] - z_bbox_coords[0]) * 0.99
            print(bbox_x, bbox_y, bbox_z)

            # select the best object model that matches the aspect ratio
            # of each bounding box
            if select_best:
                cat_dir = get_ig_category_path(obj_category)
                objs = os.listdir(cat_dir)
                obj_id_to_scale_rsd = []
                for obj_id in objs:
                    obj_dir = os.path.join(cat_dir, obj_id)
                    bbox_json = os.path.join(obj_dir, 'misc', 'metadata.json')
                    with open(bbox_json, 'r') as fp:
                        bbox_data = json.load(fp)
                    obj_lenx, obj_leny, obj_lenz = bbox_data['bbox_size'] 

                    # all_objs.json and bbox.json have xy axis flipped
                    scale_x, scale_y, scale_z = \
                        bbox_y / obj_lenx, bbox_x / obj_leny, bbox_z / obj_lenz
                    scale_vector = np.asarray([scale_x, scale_y, scale_z])
                    scale_rsd = np.std(scale_vector) / np.mean(scale_vector)
                    obj_id_to_scale_rsd.append((scale_rsd, obj_id))

                obj_id_to_scale_rsd.sort(key=lambda x: x[0])
                _, obj_id = obj_id_to_scale_rsd[0]
                link_el = ET.SubElement(scene_tree.getroot(), 'link',
                                        dict([("name", link_name),
                                              ("category", obj_category),
                                              ("model", obj_id)]))
            else:
                link_el = ET.SubElement(scene_tree.getroot(), 'link',
                                        dict([("name", link_name),
                                              ("category", obj_category),
                                              ("model", "random")]))
                if obj['instance'] is not None:
                    link_el.set("random_group", str(obj['instance']))

            # Ugly hack: Apparently the image had x-y swapped so we need to swap them also here
            link_el.set("bounding_box", "{0:f} {1:f} {2:f}".format(
                bbox_y, bbox_x, bbox_z))

            joint_name = "j_" + link_name

            joint_el = ET.SubElement(
                scene_tree.getroot(), 'joint', dict([("name", joint_name)]))

            if obj['is_fixed'] == True:
                joint_el.set("type", "fixed")
            else:
                joint_el.set("type", "floating")
            # joint_el.set("type", "fixed")

            xy_coord = obj['center']
            z_coord = (z_bbox_coords[1] + z_bbox_coords[0])/2.0

            yaw = -obj['theta'] + math.pi / 2.

            bbox_file = os.path.join(bbox_dir, "{}.obj".format(c))
            # edge_x= np.asarray(obj['edge_x'])
            # edge_y= np.asarray(obj['edge_y'])
            # z= obj['z']
            # gen_cube_obj(center, edge_x, edge_y, z, bbox_file, is_color=True)
            write_obj(*gen_rotated_obj(obj['center'], bbox_x, bbox_y,
                                       z_bbox_coords, obj['theta']), bbox_file)

            # Ugly hack: Apparently the image had x-y swapped so we need to swap them also here
            origin = \
                ET.SubElement(joint_el, 'origin',
                              dict([("xyz", "{0:f} {1:f} {2:f}".format(xy_coord[0], xy_coord[1], z_coord)),
                                    ('rpy', "0 0 {0:f}".format(yaw))]))

            child = ET.SubElement(
                joint_el, 'child', dict([("link", link_name)]))
            # We connect the embedded URDFs to the world
            parent = ET.SubElement(
                joint_el, 'parent', dict([("link", "world")]))
            # print(total)

    scene_file_out = get_ig_scene_path(scene_name) + "/" + scene_name + ".urdf"
    scene_tree.write(scene_file_out)
    print('all categories:', categories)


def gen_rotated_obj(center, scale_x, scale_y, z, theta):
    rot = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])
    vertices = []
    a, b, c, d = get_coords([1, 0], [0, 1], [0, 0])
    for x, y in [a, b, d, c]:
        vertices.append((x, y, z[1]))
    for x, y in [a, b, d, c]:
        vertices.append((x, y, z[0]))
    vertices = np.array(vertices)
    vertices *= [scale_x, scale_y, 1.]
    vertices = vertices.dot(rot)
    vertices += [*center, 0]
    faces = [(1, 2, 3), (2, 4, 3), (1, 3, 5),
             (3, 7, 5), (1, 5, 2), (5, 6, 2),
             (2, 6, 4), (6, 8, 4), (4, 8, 7),
             (7, 3, 4), (6, 7, 8), (6, 5, 7), ]
    faces = [(*f, -1) for f in faces]
    return vertices, faces


def gen_cube_obj_raw(center, edge_x, edge_y, z):
    vertices = []
    a, b, c, d = get_coords(edge_x, edge_y, center)
    for x, y in [a, b, d, c]:
        vertices.append((x, y, z[1]))
    for x, y in [a, b, d, c]:
        vertices.append((x, y, z[0]))
    faces = [(1, 2, 3), (2, 4, 3), (1, 3, 5),
             (3, 7, 5), (1, 5, 2), (5, 6, 2),
             (2, 6, 4), (6, 8, 4), (4, 8, 7),
             (7, 3, 4), (6, 7, 8), (6, 5, 7), ]
    faces = [(*f, -1) for f in faces]
    return vertices, faces


def write_obj(vertices, faces, file_path, is_color=True):
    c = np.random.rand(3)
    with open(file_path, 'w') as fp:
        for v in vertices:
            if is_color:
                v1, v2, v3 = v
                fp.write('v {} {} {} {} {} {}\n'.format(v1, v2, v3, *c))
            else:
                v1, v2, v3 = v
                fp.write('v {} {} {}\n'.format(v1, v2, v3))
        for f in faces:
            fp.write('f {} {} {}\n'.format(*f[:-1]))


def get_coords(edge_x, edge_y, center):
    '''
    Return the vertices of the bounding box, in order of BL,BR,TR,TL
    '''
    x = np.array(edge_x) / 2.
    y = np.array(edge_y) / 2.
    return np.array([center - x - y,
                     center + x - y,
                     center + x + y,
                     center - x + y])


def main():
    parser = argparse.ArgumentParser(
        description='Convert from old json annotation into new urdf models.')
    parser.add_argument('scene_names', metavar='s', type=str,
                        nargs='+', help='The name of the scene to process')
    parser.add_argument('--select_best', dest='select_best',
                        action='store_true')

    args = parser.parse_args()
    for scene_name in args.scene_names:
        convert_scene(scene_name, select_best=args.select_best)


if __name__ == "__main__":
    main()
