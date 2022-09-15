'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 15:40:19
LastEditTime: 2022-09-15 18:37:20
Description: 
'''
import logging
logging.basicConfig(
                level=logging.DEBUG,
                filename='log/drawer_info.log',
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

# box 1
def open_drawer(idx):

    urdf_path = f'processed/drawer/{idx}/motion_sapien.urdf'
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

    # ic(asset.get_joints())
    joints = asset.get_joints()
    for i, joint in enumerate(joints):
        if joint.type == 'revolute':
            # ic(joint.get_limits().max())
            # ic(asset.get_qpos())
            asset.set_qpos([joint.get_limits().max()])

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
    cam_pos = np.array([-.2, -.2, .3])
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

    while not viewer.closed:
        if viewer.window.key_down('p'):  # Press 'p' to take the screenshot
            rgba = viewer.window.get_float_texture('Color')
            rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
            rgba_pil = Image.fromarray(rgba_img)
            rgba_pil.save(f'./log/{idx}.png')


        elif viewer.window.key_down('up'):
            qpos = asset.get_qpos()
            qpos[0] += 0.01
            asset.set_qpos(qpos)
        elif viewer.window.key_down('down'):
            qpos = asset.get_qpos()
            qpos[0] -= 0.01
            asset.set_qpos(qpos)
        elif viewer.window.key_down('l'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0, 0, 0.01, 0]))
            asset.set_root_pose(root_pose)
        elif viewer.window.shift and viewer.window.key_down('l'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0, 0, -0.01, 0]))
            asset.set_root_pose(root_pose)
        elif viewer.window.key_down('j'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0, 0.01, 0, 0]))
            asset.set_root_pose(root_pose)
        elif viewer.window.shift and viewer.window.key_down('j'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0, -0.01, 0, 0]))
            asset.set_root_pose(root_pose)
        elif viewer.window.key_down('k'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0.01, 0, 0, 0]))
            asset.set_root_pose(root_pose)
        elif viewer.window.shift and viewer.window.key_down('k'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([-0.01, 0, 0, 0]))
            asset.set_root_pose(root_pose)
        elif viewer.window.key_down('i'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0, 0, 0, 0.01]))
            asset.set_root_pose(root_pose)
        elif viewer.window.shift and viewer.window.key_down('i'):
            root_pose = asset.get_root_pose()
            root_pose.set_q(root_pose.q + np.array([0, 0, 0, -0.01]))
            asset.set_root_pose(root_pose)
        elif viewer.window.key_down('1'):
            logging.info(f'{idx} 1')
            viewer.close()
        elif viewer.window.key_down('escape'):
            logging.error(f'{idx} 0')
            viewer.close()
        elif viewer.window.key_down('2'):
            logging.info(f'{idx} 2')
            viewer.close()
        elif viewer.window.key_down('3'):
            logging.info(f'{idx} 3')
            viewer.close()
        elif viewer.window.key_down('4'):
            logging.info(f'{idx} 4')
            viewer.close()
        elif viewer.window.key_down('5'):
            logging.info(f'{idx} 5')
            viewer.close()
        elif viewer.window.key_down('6'):
            logging.info(f'{idx} 6')
            viewer.close()
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == '__main__':
    for idx in range(274, 315):
        open_drawer(idx)