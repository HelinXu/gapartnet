'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-15 16:16:55
LastEditTime: 2022-09-15 17:39:32
Description: 
'''
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
    # box1
    for i in range(len(CAT['box'][1]['ids'])):
        cat = 'box'
        idx = CAT[cat][1]["ids"][i]
        file = f'processed/{cat}/{idx}/motion_sapien.urdf'
        node, path = load_urdf(file)
        ic(file)
        ic(node)
        # exit()
        # add a new link
        new_link = ET.Element('link', name='root')
        node.append(new_link)
        # add a new joint
        new_joint = ET.Element('joint', name='root_joint', type='fixed')
        new_joint.append(ET.Element('parent', link='root'))
        new_joint.append(ET.Element('child', link='base_link'))
        new_joint.append(ET.Element('origin', xyz='0 0 0', rpy='0 0 3.14159265359'))
        node.append(new_joint)
        # save the etree
        with open(file.replace('sapien.urdf', 'sapien_v2.urdf'), 'wb') as f:
            f.write(ET.tostring(node, pretty_print=True))
        