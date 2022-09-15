'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 15:40:19
LastEditTime: 2022-09-16 00:30:47
Description: 
'''
from dis import dis
import logging
logging.basicConfig(
                level=logging.DEBUG,
                filename='log/bucket_pose.log',
                filemode='a',
                format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s')
from utils.categorize import CAT
import sapien.core as sapien
import numpy as np
from PIL import Image, ImageColor
import open3d as o3d
from sapien.utils.viewer import Viewer
from transforms3d.euler import mat2euler
from glob import glob
import random
import json
import os
from utils.d3_util import actor_to_open3d_mesh, add_line_set_to_renderer
from icecream import ic, install
install()
ic.configureOutput(includeContext=True, contextAbsPath=True)

box_id = [
    40,
    41,
    45,
    # 52,
    55,
    58,
    60,
    63,
    64,
    # 65,
    67,
    68,
    73,
    78,
    # 86,
    89,
    # 91,
    94,
    96,
    99,
    100,
    # 101,
    102,
    117,
    122,
    124,
    128,
    # 130,
    135,
    141,
    142,
    # 148,
    149,
    153,
    164,
    165,
    167,
    169,
    174,
    178,
    184,
    186,
    # 189,
    194,
    195,
    196,
    198,
    201,
    # 202,
    204,
    206,
    208,
    209,
    210,
    # 212,
]

trashcan_id = [
    213,
    219,
    224,
    225,
    227,
    229,
    230,
    232,
    234,
    237,
    244,
    245,
    246,
    247,
    249,
    250,
    # 254,  # noisy mesh, bbox too big
    256,
    257,
    258,
    260,
    263,
    270,
]

def render(idx, cat):

    urdf_path = f'processed/{cat}/{idx}/motion_sapien_v2.urdf'
    ic(urdf_path)
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    # urdf_path = f'data/{id}/motion_.urdf'
    # load as a kinematic articulation
    asset = loader.load_kinematic(urdf_path)
    assert asset, 'URDF not loaded.'

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

    near, far = 0.1, 100
    width, height = 640, 480
    camera_mount_actor = scene.create_actor_builder().build_kinematic()
    camera = scene.add_mounted_camera(
        name="camera",
        actor=camera_mount_actor,
        pose=sapien.Pose(),  # relative to the mounted actor
        width=width,
        height=height,
        fovy=np.deg2rad(55),
        near=near,
        far=far,
    )

    # Compute the camera pose by specifying forward(x), left(y) and up(z)
    cam_pos = np.array([-.15, -.15, .2])
    forward = -cam_pos / np.linalg.norm(cam_pos)
    left = np.cross([0, 0, 1], forward)
    left = left / np.linalg.norm(left)
    up = np.cross(forward, left)
    mat44 = np.eye(4)
    mat44[:3, :3] = np.stack([forward, left, up], axis=1)
    mat44[:3, 3] = cam_pos
    camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(mat44))

    scene.step()  # make everything set
    scene.update_render()
    camera.take_picture()

    # add_line_set_to_renderer(scene=scene, renderer=renderer, position=bbox_points, connection=bbox_lines)

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    # We show how to set the viewer according to the pose of a camera
    # opengl camera -> sapien world
    model_matrix = camera.get_model_matrix()
    # sapien camera -> sapien world
    # You can also infer it from the camera pose
    model_matrix = model_matrix[:, [2, 0, 1, 3]] * np.array([-1, -1, 1, 1])
    # The rotation of the viewer camera is represented as [roll(x), pitch(-y), yaw(-z)]
    rpy = mat2euler(model_matrix[:3, :3]) * np.array([1, -1, -1])
    viewer.set_camera_xyz(*model_matrix[0:3, 3])
    viewer.set_camera_rpy(*rpy)
    if os.path.isfile(f'processed/{cat}/{idx}/init_pose.json'):
        with open(f'processed/{cat}/{idx}/init_pose.json', 'r') as f:
            data = json.load(f)
        asset.set_qpos(data['qpos'])
        bbox_points, bbox_lines = np.asarray(data['bbox_points']), np.asarray(data['bbox_lines'])
        add_line_set_to_renderer(scene=scene, renderer=renderer, position=bbox_points, connection=bbox_lines)
        print('reset pose from json file')
    else:
        logging.info(f'{idx} init pose not found')
    while not viewer.closed:
        viewer.update_coordinate_axes_scale(0)
        viewer.set_fovy(np.deg2rad(55))
        scene.step()
        scene.update_render()
        viewer.render()
        rgba = viewer.window.get_float_texture('Color')
        rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
        rgba_pil = Image.fromarray(rgba_img)
        rgba_pil.save(f'./img/{cat}_{idx}.png')
        viewer.close()



if __name__ == '__main__':
    for idx in trashcan_id:
        render(idx,cat='trashcan')