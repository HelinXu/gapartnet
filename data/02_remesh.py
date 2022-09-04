'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-04 21:17:42
LastEditTime: 2022-09-05 01:52:19
Description: 
'''
from glob import glob
from tqdm import tqdm

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

for objfile in tqdm(glob('processed/*/*/part_meshes/*.obj')):
    print(objfile)
    process_obj(objfile)
