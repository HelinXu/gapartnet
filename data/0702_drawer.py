'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-15 22:42:06
LastEditTime: 2022-09-15 23:16:52
Description: 


'''

from email.quoprimime import body_check
import logging
logging.basicConfig(level=logging.DEBUG, filename='./v2/bucket.log')
import sapien.core as sapien
import numpy as np
from PIL import Image, ImageColor
import open3d as o3d
from sapien.utils.viewer import Viewer
from transforms3d.euler import mat2euler
from glob import glob
import os
import six
from lxml import etree as ET
from utils.categorize import CAT
from icecream import ic, install
install()
ic.configureOutput(includeContext=True, contextAbsPath=True)


def load_urdf(file_obj):
    """Load a URDF from a file.

    Parameters
    ----------
    file_obj : str or file-like object
        The file to load the URDF from. Should be the path to the
        ``.urdf`` XML file. Any paths in the URDF should be specified
        as relative paths to the ``.urdf`` file instead of as ROS
        resources.
    """
    if isinstance(file_obj, six.string_types):
        if os.path.isfile(file_obj):
            parser = ET.XMLParser(remove_comments=True,
                                    remove_blank_text=True)
            tree = ET.parse(file_obj, parser=parser)
            path, _ = os.path.split(file_obj)
        else:
            raise ValueError('{} is not a file'.format(file_obj))
    else:
        parser = ET.XMLParser(remove_comments=True, remove_blank_text=True)
        tree = ET.parse(file_obj, parser=parser)
        path, _ = os.path.split(file_obj.name)

    node = tree.getroot()
    return (node, path)


if __name__ == '__main__':
    count = 0
    for idx in CAT['drawer'][1]["ids"] + CAT['drawer'][2]["ids"] + CAT['drawer'][3]["ids"] + CAT['drawer'][4]["ids"] + CAT['drawer'][5]["ids"] + CAT['drawer'][6]["ids"]:
        cat = 'drawer'
        file = f'processed/{cat}/{idx}/motion_sapien_v2.urdf'
        # check if there are 3 links
        node, path = load_urdf(file)
        links = node.findall('link')
        print(len(links), idx)
        # check if there is root link
        root_link = node.find('link[@name="root"]')
        assert root_link is not None, f'{idx} has no root link'
        # check if there is base link
        base_link = node.find('link[@name="base_link"]')
        assert base_link is not None, f'{idx} has no base link'
        # check if there is link_1
        link_1 = node.find('link[@name="link1"]')
        assert link_1 is not None, f'{idx} has no link1'
        if len(links) == 4:
            # check if there is link_2
            link_2 = node.find('link[@name="link2"]')
            assert link_2 is not None, f'{idx} has no link2'
            with open(f'processed/{cat}/{idx}/semantics.txt', 'w') as f:
                f.write("root fixed root\nbase_link static body\nlink1 slider drawer\nlink2 slider drawer")
            count += 1
        
        elif len(links) == 5:
            # check if there is link_2
            link_2 = node.find('link[@name="link2"]')
            assert link_2 is not None, f'{idx} has no link2'
            # check if there is link_3
            link_3 = node.find('link[@name="link3"]')
            assert link_3 is not None, f'{idx} has no link3'
            with open(f'processed/{cat}/{idx}/semantics.txt', 'w') as f:
                f.write("root fixed root\nbase_link static body\nlink1 slider drawer\nlink2 slider drawer\nlink3 slider drawer")
            count += 1
        else:
            assert len(links) == 3, f'{idx} has more than 3 links'
            with open(f'processed/{cat}/{idx}/semantics.txt', 'w') as f:
                f.write("root fixed root\nbase_link static body\nlink1 slider drawer")
            count += 1
    
    print("total drawer objects: ", count)