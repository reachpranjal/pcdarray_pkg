#!/usr/bin/env python3

"""
Description: This script performs conversion of PointCloud2 data into numpy array.
Author: Pranjal Paul
Date Created: August 14, 2023
Date Modified: August 29, 2023
Version: 1.0.0
Python Version: 3.10
Dependencies: numpy
License: MIT License
"""

import numpy as np
from sensor_msgs.msg import PointField

# Ref: https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html

__all__ = ['PC2_Array']

__datatypes = [
    np.dtype('int8'), 
    np.dtype('uint8'),
    np.dtype('int16'), 
    np.dtype('uint16'),
    np.dtype('int32'), 
    np.dtype('uint32'),
    np.dtype('float32'), 
    np.dtype('float64'),
]

__type_dict = {
    getattr(PointField, dtype.name.upper()): dtype for dtype in __datatypes
}

__type_size = {
    ftype: np.dtype(dtype).itemsize for ftype, dtype in __type_dict.items()
}


def __get_X3d(array):
    points = np.zeros((array.shape[0], 3), dtype=np.float32)
    points[:, 0] = array['x']
    points[:, 1] = array['y']
    points[:, 2] = array['z']
    return points


def PC2_Array(pc2_msg):
    dtypes = []    
    offset = 0
    for field in pc2_msg.fields: 
        '''Each field contains (name, offset, datatype, count) attribs'''
        dtype = __type_dict[field.datatype]
        dtypes.append((field.name, dtype))
        offset += __type_size[field.datatype]

    arr = np.frombuffer(pc2_msg.data, dtypes).reshape((pc2_msg.width,))
    return __get_X3d(arr)