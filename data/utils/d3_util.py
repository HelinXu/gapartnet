'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 18:14:53
LastEditTime: 2022-09-05 18:29:58
Description: 
'''
import os
from typing import List, Optional, Union

import cv2
import numpy as np
import open3d as o3d
from sapien import core as sapien
from sapien.core.pysapien import renderer as R

Vector3dVector = o3d.utility.Vector3dVector
Vector3iVector = o3d.utility.Vector3iVector


def merge_o3d_meshes(meshes: List[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    if len(meshes) < 1:
        raise RuntimeError(f"Need at least one mesh to merge.")
    if len(meshes) == 1:
        return meshes[0]
    else:
        combined_mesh = meshes[0]
        for mesh in meshes[1:]:
            mesh = mesh
            combined_mesh += mesh
        return combined_mesh


def render_body_to_open3d_mesh(render_body: sapien.RenderBody, use_texture=True) -> o3d.geometry.TriangleMesh:
    body_pose = render_body.local_pose.to_transformation_matrix()
    body_type = render_body.type
    if body_type == "mesh":
        meshes = []
        scale = render_body.scale
        for render_shape in render_body.get_render_shapes():
            mesh = render_shape.mesh
            material = render_shape.material

            # TODO: consider other texture type other than diffuse
            has_material = len(material.diffuse_texture_filename) > 0

            vertices = mesh.vertices
            indices = np.reshape(mesh.indices, [-1, 3]).astype(np.int32)
            normals = mesh.normals

            triangle_mesh = o3d.geometry.TriangleMesh(Vector3dVector(vertices * scale[None, :]),
                                                      Vector3iVector(indices))
            triangle_mesh.vertex_normals = Vector3dVector(normals)
            if has_material and use_texture:
                img = cv2.imread(material.diffuse_texture_filename)
                triangle_mesh.textures = o3d.geometry.Image(img)
            else:
                vertex_color = material.base_color[:3]
                triangle_mesh.vertex_colors = Vector3dVector(np.tile(vertex_color, (vertices.shape[0], 1)))
            meshes.append(triangle_mesh)
        render_body_mesh = merge_o3d_meshes(meshes)
        render_body_mesh.transform(body_pose)
    else:
        assert len(render_body.get_render_shapes()) == 1
        material = render_body.get_render_shapes()[0].material
        has_material = len(material.diffuse_texture_filename) > 0
        if body_type == "box":
            half_size = render_body.half_lengths
            render_body_mesh = o3d.geometry.TriangleMesh.create_box(*(half_size * 2))
        elif body_type == "capsule":
            radius = render_body.radius
            half_length = render_body.half_length
            vertices = render_body.get_render_shapes()[0].mesh.vertices
            normals = render_body.get_render_shapes()[0].mesh.normals
            indices = np.reshape(render_body.get_render_shapes()[0].mesh.indices, [-1, 3]).astype(np.int32)
            render_body_mesh = o3d.geometry.TriangleMesh(Vector3dVector(vertices),
                                                         Vector3iVector(indices))
            render_body_mesh.vertex_normals = Vector3dVector(normals)
        elif body_type == "sphere":
            radius = render_body.radius
            render_body_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        else:
            raise NotImplementedError

        # Texture or color
        if has_material and use_texture:
            raise NotImplementedError(f"Only mesh type supports texture in SAPIEN.")
        else:
            vertex_color = material.base_color[:3]
            num_v = len(render_body_mesh.vertices)
            render_body_mesh.vertex_colors = Vector3dVector(np.tile(vertex_color, (num_v, 1)))

        # Pose
        render_body_mesh.transform(body_pose)

    return render_body_mesh

def actor_to_open3d_mesh(actor: sapien.ActorBase, use_collision_mesh=False, use_texture=False, use_actor_pose=False):
    meshes = []
    if not use_collision_mesh:
        for render_body in actor.get_visual_bodies():
            meshes.append(render_body_to_open3d_mesh(render_body, use_texture))
    else:
        for collision_render_body in actor.get_collision_visual_bodies():
            meshes.append(render_body_to_open3d_mesh(collision_render_body, use_texture))

    if len(meshes) > 0:
        mesh = merge_o3d_meshes(meshes)
        if use_actor_pose:
            mesh.transform(actor.get_pose().to_transformation_matrix())
    else:
        mesh = None

    return mesh