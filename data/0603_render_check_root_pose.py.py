"""Camera.

Concepts:
    - Create and mount cameras
    - Render RGB images, point clouds, segmentation masks
"""

import logging
logging.basicConfig(level=logging.INFO, filename='./v2/root_pose_check.log')
import sapien.core as sapien
import numpy as np
from PIL import Image, ImageColor
import open3d as o3d
from sapien.utils.viewer import Viewer
from transforms3d.euler import mat2euler
from glob import glob
import os


def render(data_root='../../', save_root='.',
        category_name='box', object_id='3f8719c9-0e3e-11ed-ac75-ec2e98c7e246'
        ):
    # if os.path.isfile(f'{save_root}/{category_name}/{object_id}_seg1.png'):
    #     return
    urdf_path = os.path.join(data_root, category_name, object_id, 'motion_sapien_v2.urdf')

    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    # load as a kinematic articulation
    asset = loader.load_kinematic(urdf_path)
    assert asset, 'URDF not loaded.'


    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

    # ---------------------------------------------------------------------------- #
    # Camera
    # ---------------------------------------------------------------------------- #
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

    print('Intrinsic matrix\n', camera.get_camera_matrix())

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

    # ---------------------------------------------------------------------------- #
    # RGBA
    # ---------------------------------------------------------------------------- #
    rgba = camera.get_float_texture('Color')  # [H, W, 4]
    # An alias is also provided
    # rgba = camera.get_color_rgba()  # [H, W, 4]
    rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
    rgba_pil = Image.fromarray(rgba_img)
    rgba_pil.save(f'{save_root}/{category_name}/{object_id}_color.png')

    # ---------------------------------------------------------------------------- #
    # XYZ position in the camera space
    # ---------------------------------------------------------------------------- #
    # Each pixel is (x, y, z, is_valid) in camera space (OpenGL/Blender)
    position = camera.get_float_texture('Position')  # [H, W, 4]

    # OpenGL/Blender: y up and -z forward
    points_opengl = position[..., :3][position[..., 3] > 0]
    points_color = rgba[position[..., 3] > 0][..., :3]
    # Model matrix is the transformation from OpenGL camera space to SAPIEN world space
    # camera.get_model_matrix() must be called after scene.update_render()!
    model_matrix = camera.get_model_matrix()
    points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]

    # SAPIEN CAMERA: z up and x forward
    # points_camera = points_opengl[..., [2, 0, 1]] * [-1, -1, 1]

    # Depth
    depth = -position[..., 2]
    depth_image = (depth * 1000.0).astype(np.uint16)
    depth_pil = Image.fromarray(depth_image)
    depth_pil.save(f'{save_root}/{category_name}/{object_id}_depth.png')

    # ---------------------------------------------------------------------------- #
    # Segmentation labels
    # ---------------------------------------------------------------------------- #
    # Each pixel is (visual_id, actor_id/link_id, 0, 0)
    # visual_id is the unique id of each visual shape
    seg_labels = camera.get_uint32_texture('Segmentation')  # [H, W, 4]
    colormap = sorted(set(ImageColor.colormap.values()))
    color_palette = np.array([ImageColor.getrgb(color) for color in colormap],
                             dtype=np.uint8)
    label0_image = seg_labels[..., 0].astype(np.uint8)  # mesh-level
    label1_image = seg_labels[..., 1].astype(np.uint8)  # actor-level
    # Or you can use aliases below
    # label0_image = camera.get_visual_segmentation()
    # label1_image = camera.get_actor_segmentation()
    label0_pil = Image.fromarray(color_palette[label0_image])
    label0_pil.save(f'{save_root}/{category_name}/{object_id}_seg0.png')
    label1_pil = Image.fromarray(color_palette[label1_image])
    label1_pil.save(f'{save_root}/{category_name}/{object_id}_seg1.png')


if __name__ == '__main__':
    data_root = 'processed'
    save_root = 'v2'
    for f in glob(f'{data_root}/*/*/motion_sapien_v2.urdf'):
        print(f)
        _f = f.split('/')
        category_name = _f[-3]
        object_id = _f[-2]
        render(data_root=data_root, save_root=save_root, category_name=category_name, object_id=object_id)
        logging.info(f'success {object_id}')



        



