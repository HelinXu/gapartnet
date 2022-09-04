'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 00:36:41
LastEditTime: 2022-09-05 01:13:36
Description: 
'''
import urdfpy
from glob import glob
import os
import six
from lxml import etree as ET
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


for f in glob('data/*/motion_unity.urdf'):
    print(f)
    node, path = load_urdf(f)
    print(node)
    # print the etree
    for element in node.iter():
        if element.tag == 'origin':
            ic(element.attrib)
            xyz = element.attrib['xyz'].split(' ')
            ic(xyz)
            ic(f'{float(xyz[1])} {-float(xyz[2])} {float(xyz[0])}')
            element.attrib['xyz'] = f'{float(xyz[1])} {-float(xyz[2])} {float(xyz[0])}'
            element.attrib['t'] = 'v1'
        elif element.tag == 'axis':
            ic(element.attrib)
            xyz = element.attrib['xyz'].split(' ')
            ic(xyz)
            ic(f'{float(xyz[1])} {float(xyz[2])} {float(xyz[0])}')
            element.attrib['xyz'] = f'{float(xyz[1])} {float(xyz[2])} {float(xyz[0])}'
            element.attrib['t'] = 'v1'


    # save the etree
    with open(f.replace('unity.urdf', '.urdf'), 'wb') as f:
        f.write(ET.tostring(node, pretty_print=True))


