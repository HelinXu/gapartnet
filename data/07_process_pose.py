'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 15:40:19
LastEditTime: 2022-09-05 18:09:05
Description: 
'''
import logging
logging.basicConfig(
                level=logging.DEBUG,
                filename='log/process_pose.log',
                filemode='a',
                format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s')
from categorize import CAT
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
from icecream import ic, install
install()
ic.configureOutput(includeContext=True, contextAbsPath=True)

# box 1
def box1(idx):
    urdf_path = f'processed/box/{idx}/motion_sapien.urdf'
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
    asset.set_root_pose(sapien.Pose(CAT['box'][1]['root_p'],
                                    CAT['box'][1]['root_q']))
    # ic(asset.get_joints())
    if asset.get_qpos().shape[0] != 1:
        logging.error(f'box1 {idx} qpos shape error: {asset.get_qpos()}')
    else:
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
        elif viewer.window.key_down('r'):
            # reset from json file
            if os.path.isfile(f'processed/box/{idx}/init_pose.json'):
                with open(f'processed/box/{idx}/init_pose.json', 'r') as f:
                    data = json.load(f)
                print('reset pose from json file')
                asset.set_qpos(data['qpos'])
                asset.set_root_pose(sapien.Pose(data['root_p'], data['root_q']))
        elif viewer.window.key_down('up'):
            asset.set_qpos([asset.get_qpos()[0] + 0.01])
        elif viewer.window.key_down('down'):
            asset.set_qpos([asset.get_qpos()[0] - 0.01])
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
        elif viewer.window.key_down('enter'):
            # write the pose to json file
            with open(f'processed/box/{idx}/init_pose.json', 'w') as f:
                pose_dict = {
                    'root_p': asset.get_root_pose().p.tolist(),
                    'root_q': asset.get_root_pose().q.tolist(),
                    'qpos': asset.get_qpos().tolist()
                }
                json.dump(pose_dict, f, indent=4)
            viewer.close()
        elif viewer.window.key_down('escape'):
            logging.error(f'box1 {idx} did not save the pose')
            viewer.close()
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == '__main__':
    for i in range(len(CAT['box'][1]['ids'])):
        logging.info(CAT['box'][1]['ids'][i])
        box1(CAT['box'][1]['ids'][i])
        logging.info(CAT['box'][1]['ids'][i], 'done')