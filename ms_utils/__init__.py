import math
from typing import List
import numpy as np
# import copy
# from config import APOLLO_ROOT, RECORDS_DIR, STREAM_LOGGING_LEVEL
import carla
import random

from shapely.geometry import Polygon

def freeze_and_set_green_all_tls(carla_world, logger):
    traffic_lights = carla_world.get_actors().filter('traffic.traffic_light')
    for tl in traffic_lights:
        tl.set_state(carla.TrafficLightState.Green)
        tl.freeze(True)
    logger.info('[Simulator] freeze and set green all tls')


def select_valid_dest(ego_vehicle, carla_map, min_radius=100, max_radius=150) -> carla.Transform:
    '''
        Select a destination outside the specified radius from current position
    '''
    ego_curr_point = ego_vehicle.get_transform()
    valid_destination = False
    sps = carla_map.get_spawn_points()
    while not valid_destination:
        des_transform = random.choice(sps)
        des_wp = carla_map.get_waypoint(des_transform.location,
                                        project_to_road=False)
        distance = ego_curr_point.location.distance(des_transform.location)
        if distance < min_radius or distance > max_radius:
            continue
        if des_wp.is_junction:
            continue
        valid_destination = True
    return des_transform


def detect_front_vehicle_obstacle(world, ego_vehicle, max_distance=10):
    """
    Check if there is a vehicle in front of the ego vehicle blocking its path when the ego is stationary.

    Args:
        ego_vehicle (carla.Vehicle): The ego vehicle object.
        world (carla.World): Carla world object to access other actors and map.
        max_distance (float): Maximum detection distance in front of the ego vehicle.
        up_angle_th (float): Upper angle threshold for detection range.
        low_angle_th (float): Lower angle threshold for detection range.

    Returns:
        tuple: (bool, carla.Vehicle or None)
            - bool: True if a vehicle is blocking the path, False otherwise.
            - carla.Vehicle: The blocking vehicle object, or None if no vehicle is detected.
    """

    def get_route_polygon():
        """
        Generate a polygon representing the area in front of the ego vehicle's route.
        """
        route_bb = []
        extent_y = ego_vehicle.bounding_box.extent.y
        r_ext = extent_y
        l_ext = -extent_y
        r_vec = ego_transform.get_right_vector()
        p1 = ego_location + \
            carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
        p2 = ego_location + \
            carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
        route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

        for wp, _ in local_planner.get_plan():
            if ego_location.distance(wp.transform.location) > max_distance:
                break
            r_vec = wp.transform.get_right_vector()
            p1 = wp.transform.location + \
                carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
            p2 = wp.transform.location + \
                carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
            route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

        # Ensure the polygon has enough points to form a valid shape
        if len(route_bb) < 3:
            return None

        return Polygon(route_bb)

    # Check if ego vehicle is stationary
    velocity = ego_vehicle.get_velocity()
    if velocity.x != 0 or velocity.y != 0 or velocity.z != 0:
        return False, None  # Ego vehicle is not stationary

    # Get global vehicle list
    vehicle_list = world.get_actors().filter("*vehicle*")
    ego_transform = ego_vehicle.get_transform()
    ego_location = ego_transform.location
    ego_front_transform = ego_transform
    ego_front_transform.location += carla.Location(
        ego_vehicle.bounding_box.extent.x * ego_transform.get_forward_vector()
    )
    local_planner = world.get_map()

    # Get route bounding polygon
    route_polygon = get_route_polygon()
    if not route_polygon:
        return False, None

    for target_vehicle in vehicle_list:
        if target_vehicle.id == ego_vehicle.id:
            continue

        target_transform = target_vehicle.get_transform()
        if target_transform.location.distance(ego_location) > max_distance:
            continue

        # Check if the target vehicle blocks the route polygon
        target_bb = target_vehicle.bounding_box
        target_vertices = target_bb.get_world_vertices(target_transform)
        target_polygon = Polygon([[v.x, v.y, v.z]
                                  for v in target_vertices])

        if route_polygon.intersects(target_polygon):
            return True, target_vehicle

    return False, None


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


def get_crosswalk_list(locs: List[carla.Location]):
    crosswalk_list = []
    i = 0
    while i < len(locs):
        j = i + 1
        this_crosswalk = [locs[i]]
        while locs[i] != locs[j]:
            this_crosswalk.append(locs[j])
            j += 1
            if j == len(locs):
                break
        crosswalk_list.append(this_crosswalk)
        i = j + 1
    return crosswalk_list


def is_point_in_crosswalk(point: carla.Location, crosswalk: List[carla.Location]):
    x, y = point.x, point.y
    n = len(crosswalk)
    inside = False

    p1x, p1y = crosswalk[0].x, crosswalk[0].y
    for i in range(n + 1):
        p2x, p2y = crosswalk[i % n].x, crosswalk[i % n].y
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def is_point_in_any_crosswalk(point: carla.Location, crosswalk_list: List[List[carla.Location]]):
    return any(is_point_in_crosswalk(point, crosswalk) for crosswalk in crosswalk_list)


def predict_collision(actor1: carla.ActorSnapshot,
                      actor2: carla.ActorSnapshot,
                      prediction_time=1,
                      time_step=0.05,
                      collision_distance=2.0):
    if actor1 is None or actor2 is None:
        return False, None
    # Initial conditions
    position1 = actor1.get_transform().location
    velocity1 = actor1.get_velocity()
    acceleration1 = actor1.get_acceleration()

    position2 = actor2.get_transform().location
    velocity2 = actor2.get_velocity()
    acceleration2 = actor2.get_acceleration()

    for t in np.arange(0, prediction_time, time_step):
        # Update positions
        future_pos1 = carla.Location(
            x=position1.x + (velocity1.x * t) + 0.5
            * acceleration1.x * t ** 2,
            y=position1.y + (velocity1.y * t) + 0.5
            * acceleration1.y * t ** 2,
            z=position1.z + (velocity1.z * t) + 0.5
            * acceleration1.z * t ** 2
        )

        future_pos2 = carla.Location(
            x=position2.x + (velocity2.x * t) + 0.5
            * acceleration2.x * t ** 2,
            y=position2.y + (velocity2.y * t) + 0.5
            * acceleration2.y * t ** 2,
            z=position2.z + (velocity2.z * t) + 0.5
            * acceleration2.z * t ** 2
        )

        # Calculate distance between future positions
        dist = math.sqrt((future_pos1.x - future_pos2.x) ** 2
                         + (future_pos1.y - future_pos2.y) ** 2
                         + (future_pos1.z - future_pos2.z) ** 2)

        # Check for potential collision
        if dist < collision_distance:
            return True, t

    return False, None
