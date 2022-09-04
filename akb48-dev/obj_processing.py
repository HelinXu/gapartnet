'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-04 21:17:42
LastEditTime: 2022-09-04 22:00:21
Description: 
'''
import open3d as o3d
from glob import glob

def process_obj(objfile):
    with open(objfile, "r") as f:
        lines = f.readlines()
        outputs = lines.copy()
        for line in lines:
            if line.startswith("f "):
                # print(line)
                l = line.strip().split(' ')
                newline = f"\nf {l[1]} {l[3]} {l[2]}"
                outputs.append(newline)

    # save outputs
    with open(objfile, "w") as f:
        f.writelines(outputs)

for objfile in glob('data/*/part_meshes/*.obj'):
    print(objfile)
    process_obj(objfile)



# # load obj file
# mesh = o3d.io.read_triangle_mesh("data/1/raw2.obj")

# # # crate triangle mesh
# a= mesh.compute_vertex_normals()
# # # a = mesh.compute_convex_hull()

# # visualize
# o3d.visualization.draw_geometries([a])
