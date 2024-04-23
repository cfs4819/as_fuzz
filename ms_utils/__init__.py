import glob
import json
import shutil
import os
import logging
import random
import time
import math
from typing import List

# from config import APOLLO_ROOT, RECORDS_DIR, STREAM_LOGGING_LEVEL
import carla


def calc_relative_loc(ref_tf: carla.Transform,
                      offset_f: float,
                      offset_r: float,
                      offset_h=0.0) -> carla.Location:
    ref_loc = ref_tf.location
    ref_f_vec = ref_tf.get_forward_vector()
    ref_r_vec = ref_tf.get_right_vector()
    ref_u_vec = ref_tf.get_up_vector()
    target_x = ref_loc.x + offset_f*ref_f_vec.x + \
        offset_r*ref_r_vec.x + offset_h*ref_u_vec.x
    target_y = ref_loc.y + offset_f*ref_f_vec.y + \
        offset_r*ref_r_vec.y + offset_h*ref_u_vec.y
    target_z = ref_loc.z + offset_f*ref_f_vec.z + \
        offset_r*ref_r_vec.z + offset_h*ref_u_vec.z
    return carla.Location(target_x, target_y, target_z)

def calc_relative_loc_dict(ref_tf: carla.Transform,
                      offset_f: float,
                      offset_r: float,
                      offset_h=0.0) -> carla.Location:
    ref_loc = ref_tf.location
    ref_f_vec = ref_tf.get_forward_vector()
    ref_r_vec = ref_tf.get_right_vector()
    ref_u_vec = ref_tf.get_up_vector()
    target_x = ref_loc.x + offset_f*ref_f_vec.x + \
        offset_r*ref_r_vec.x + offset_h*ref_u_vec.x
    target_y = ref_loc.y + offset_f*ref_f_vec.y + \
        offset_r*ref_r_vec.y + offset_h*ref_u_vec.y
    target_z = ref_loc.z + offset_f*ref_f_vec.z + \
        offset_r*ref_r_vec.z + offset_h*ref_u_vec.z
    return {'x': target_x, 'y': target_y, 'z': target_z}

def rotate_point(x, y, angle_degrees):
    """
    Rotate a point counterclockwise by a given angle around the origin.
    The angle should be given in degrees.
    """
    angle_radians = math.radians(angle_degrees)
    cos_angle = math.cos(angle_radians)
    sin_angle = math.sin(angle_radians)
    x_new = x * cos_angle - y * sin_angle
    y_new = x * sin_angle + y * cos_angle
    return x_new, y_new