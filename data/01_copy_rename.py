'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 01:42:26
LastEditTime: 2022-09-05 01:49:11
Description: 
'''
import os
from glob import glob

for idx, f in enumerate(glob('../../*/*/motion_unity.urdf')):
    print(f)
    print(f[:-18])
    # copy the folder
    os.system(f"cp -r {f[:-18]} processed/{f.split('/')[2]}/{idx}")
