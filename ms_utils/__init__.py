import math
from typing import List
import numpy as np
# import copy
# from config import APOLLO_ROOT, RECORDS_DIR, STREAM_LOGGING_LEVEL
import carla


def calc_relative_loc(ref_tf: carla.Transform,
                      offset_f: float,
                      offset_r: float,
                      offset_h=0.0) -> carla.Location:
    result_loc = carla.Location(x=offset_f, y=offset_r, z=offset_h)
    ref_tf.transform(result_loc)
    return result_loc
    # ref_loc = ref_tf.location
    # ref_f_vec = ref_tf.get_forward_vector()
    # ref_r_vec = ref_tf.get_right_vector()
    # ref_u_vec = ref_tf.get_up_vector()
    # target_x = ref_loc.x + offset_f * ref_f_vec.x + \
    #     offset_r * ref_r_vec.x + offset_h * ref_u_vec.x
    # target_y = ref_loc.y + offset_f * ref_f_vec.y + \
    #     offset_r * ref_r_vec.y + offset_h * ref_u_vec.y
    # target_z = ref_loc.z + offset_f * ref_f_vec.z + \
    #     offset_r * ref_r_vec.z + offset_h * ref_u_vec.z
    # return carla.Location(target_x, target_y, target_z)


def calc_relative_loc_dict(ref_tf: carla.Transform,
                           offset_f: float,
                           offset_r: float,
                           offset_h=0.0) -> carla.Location:
    result_loc = carla.Location(x=offset_f, y=offset_r, z=offset_h)
    ref_tf.transform(result_loc)
    return {'x': result_loc.x, 'y': result_loc.y, 'z': result_loc.z}
    # ref_loc = ref_tf.location
    # ref_f_vec = ref_tf.get_forward_vector()
    # ref_r_vec = ref_tf.get_right_vector()
    # ref_u_vec = ref_tf.get_up_vector()
    # target_x = ref_loc.x + offset_f * ref_f_vec.x + \
    #     offset_r * ref_r_vec.x + offset_h * ref_u_vec.x
    # target_y = ref_loc.y + offset_f * ref_f_vec.y + \
    #     offset_r * ref_r_vec.y + offset_h * ref_u_vec.y
    # target_z = ref_loc.z + offset_f * ref_f_vec.z + \
    #     offset_r * ref_r_vec.z + offset_h * ref_u_vec.z
    # return {'x': target_x, 'y': target_y, 'z': target_z}


def inverse_transform_point(local_transform: carla.Transform, point_loc: carla.Location) -> carla.Location:
    world_2_local = np.array(local_transform.get_inverse_matrix())
    points = np.array(
        [point_loc.x, point_loc.y, point_loc.z, 1])
    points_camera = np.dot(world_2_local, points)
    return carla.Location(points_camera[0], points_camera[1], points_camera[2])


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
