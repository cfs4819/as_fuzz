import math
import os
import sys
import random
import time

import numpy as np
import matplotlib.pyplot as plt
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, parent_dir)
from ms_utils.apollo_routing_listener import ApolloRoutingListener


# def set_carla_api_path():
#     # print('carla 0914 neednot be installed in this version ')
#     # return
#     try:
#         api_path = "../PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg"
#     except IndexError:
#         print("Couldn't set Carla API path.")
#         exit(-1)
#
#     if api_path not in sys.path:
#         sys.path.append(api_path)
#         print(f"API: {api_path}")
#
#
# set_carla_api_path()

import carla


class Env:
    def __init__(self, world, bounds, obstacles, map, waypoints, target_waypoint, resolution=1.0, safety_distance=1.0):
        """
        Initialize the environment for Carla simulation.

        :param world: Carla world object
        :param bounds: (x_min, y_min, x_max, y_max), bounding box of the environment
        :param obstacles: List of Carla Actor objects representing obstacles
        :param map: Carla map object
        :param waypoints: List of Carla waypoint objects
        :param target_waypoint: Target waypoint for planning
        :param resolution: Grid resolution (size of each grid cell in meters)
        :param safety_distance: Safety distance to expand around obstacles
        """
        self.world = world
        self.bounds = bounds
        self.obstacles = obstacles
        self.map = map
        self.map_waypoints = map.generate_waypoints(distance=resolution)
        self.waypoints = waypoints
        self.target_waypoint = target_waypoint
        self.resolution = resolution
        self.safety_distance = safety_distance

        self.lane_grid = set()  # Grid cells representing lanes
        self.obs = set()  # Grid cells representing obstacles (expanded by safety distance)

        self.update_lanes()  # Initialize lane grid
        self.update_obs()  # Initialize obstacle grid

    def update_lanes(self):
        """
        Compute the drivable lane grid based on waypoints and lane width.
        """
        self.lane_grid.clear()

        for waypoint in self.waypoints:
            # Get lane width and direction
            lane_width = waypoint.lane_width
            transform = waypoint.transform
            location = transform.location
            yaw = math.radians(transform.rotation.yaw)

            # Calculate lane boundary offsets
            dx = math.cos(yaw) * (lane_width / 2.0)
            dy = math.sin(yaw) * (lane_width / 2.0)

            # Compute boundary points
            left_x, left_y = location.x - dx, location.y - dy
            right_x, right_y = location.x + dx, location.y + dy

            # Add all cells between left and right boundaries
            for x in np.arange(left_x, right_x, self.resolution):
                for y in np.arange(left_y, right_y, self.resolution):
                    self.lane_grid.add((int(x), int(y)))
            lane = waypoint.lane_type
            if lane == carla.LaneType.Driving:
                self.add_lane_cells(waypoint)
            # Add lanes to the left and right if available
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                self.add_lane_cells(left_lane)

            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                self.add_lane_cells(right_lane)

    def add_lane_cells(self, waypoint):
        """
        Add grid cells for an entire lane based on a given waypoint, including all points
        on the same road and lane ID, both before and after the given waypoint.
        """
        target_road_id = waypoint.road_id  # Get the road ID of the given waypoint
        target_lane_id = waypoint.lane_id  # Get the lane ID of the given waypoint
        print(f"Processing Lane: Road ID {target_road_id}, Lane ID {target_lane_id}")
        map_waypoints = self.map_waypoints
        # Filter all waypoints that belong to the same road and lane ID
        lane_waypoints = [
            wp for wp in map_waypoints
            if wp.road_id == target_road_id and wp.lane_id == target_lane_id
        ]

        if not lane_waypoints:
            print(f"[WARNING] No waypoints found for Road ID {target_road_id}, Lane ID {target_lane_id}.")
            return

        temp_lane_grid = set()  # Temporary storage for the points in this lane

        for current_waypoint in lane_waypoints:
            transform = current_waypoint.transform
            location = transform.location
            lane_width = current_waypoint.lane_width
            yaw = math.radians(transform.rotation.yaw)

            # Lane length for this segment (estimated based on resolution)
            lane_length = self.resolution * 3  # Assuming each lane segment covers ~3 grid cells

            # Calculate rotation matrix for the rectangle
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # Calculate lane dimensions in local coordinates
            half_width = lane_width / 2.0
            half_length = lane_length / 2.0

            # Generate grid points in the local coordinate system
            grid_points_count = 0
            for x_local in np.arange(-half_length, half_length, self.resolution):
                for y_local in np.arange(-half_width, half_width, self.resolution):
                    # Transform local coordinates to global coordinates
                    x_global = location.x + x_local * cos_yaw - y_local * sin_yaw
                    y_global = location.y + x_local * sin_yaw + y_local * cos_yaw

                    # Add the global point to the temporary lane grid
                    temp_lane_grid.add((int(x_global), int(y_global)))
                    grid_points_count += 1

            # Debug information
            print(f"Processed Waypoint at ({location.x:.2f}, {location.y:.2f}):")
            print(f"  Grid Points Processed in this Rectangle: {grid_points_count}")
            print(f"  Temporary Lane Grid Total Size: {len(temp_lane_grid)}\n")

        # Update the main lane grid with the points from this lane
        self.lane_grid.update(temp_lane_grid)
        print(f"Added {len(temp_lane_grid)} points to the main lane grid.")

    def update_obs(self):
        """
        Update obstacle occupancy grid based on obstacle positions and safety distance.
        """
        self.obs.clear()

        for obstacle in self.obstacles:
            bounding_box = obstacle.bounding_box
            location = obstacle.get_location()

            # Expand obstacle bounding box by the safety distance
            obs_cells = self.bounding_box_to_grid(bounding_box, location, self.safety_distance)
            self.obs.update(obs_cells)

    def bounding_box_to_grid(self, bounding_box, location, safety_distance):
        """
        Convert a bounding box to occupied grid cells, considering a spherical safety distance.

        :param bounding_box: Carla bounding box object
        :param location: Location of the bounding box
        :param safety_distance: Distance to expand around the bounding box (spherical expansion)
        :return: Set of occupied grid cells
        """
        obs_cells = set()
        box_extent = bounding_box.extent

        # Calculate the effective radius of the expanded bounding box
        effective_radius = max(box_extent.x, box_extent.y) + safety_distance

        # Define the bounding box limits for grid generation
        x_min = location.x - effective_radius
        x_max = location.x + effective_radius
        y_min = location.y - effective_radius
        y_max = location.y + effective_radius

        # Generate grid points within the bounding box limits
        for x in np.arange(x_min, x_max, self.resolution):
            for y in np.arange(y_min, y_max, self.resolution):
                # Calculate distance from the center of the bounding box to the grid point
                distance = ((x - location.x) ** 2 + (y - location.y) ** 2) ** 0.5
                # Check if the grid point lies within the spherical region
                if distance <= effective_radius:
                    obs_cells.add((int(x), int(y)))

        return obs_cells

    def is_occupied(self, x, y):
        """
        Check if a grid cell is occupied (either by an obstacle or outside the lane grid).

        :param x: X-coordinate of the grid cell
        :param y: Y-coordinate of the grid cell
        :return: True if occupied, False otherwise
        """
        return (x, y) in self.obs or (x, y) not in self.lane_grid

    def to_grid(self, location):
        """
        Convert a Carla location to grid coordinates.

        :param location: Carla location object
        :return: (x, y) grid coordinates
        """
        return int(location.x / self.resolution), int(location.y / self.resolution)

    @staticmethod
    def visualize_grid(lane_grid, obs_grid, ego_location, target_location):
        """
        Visualize the grid showing lanes and obstacles.
        """
        plt.figure(figsize=(10, 10))
        lane_x, lane_y = zip(*lane_grid) if lane_grid else ([], [])
        obs_x, obs_y = zip(*obs_grid) if obs_grid else ([], [])

        plt.scatter(lane_x, lane_y, c='blue', s=5, label='Lanes')
        plt.scatter(obs_x, obs_y, c='red', s=5, label='Obstacles')
        plt.scatter([ego_location[0]], [ego_location[1]], c='green', s=100, label='Ego Vehicle')
        plt.scatter([target_location[0]], [target_location[1]], c='orange', s=100, label='Target')

        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Grid Visualization')
        plt.grid()
        plt.show()


def main():
    from cyber.python.cyber_py3 import cyber
    from loguru import logger
    """
    Main function to test the Env class functionality with Carla.
    """
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Get map and waypoints
    carla_map = world.get_map()
    waypoints = carla_map.generate_waypoints(distance=2.0)


    # Get obstacles
    obstacles = [actor for actor in world.get_actors() if 'vehicle.' in actor.type_id]

    # Find Ego vehicle
    ego_vehicle = None
    for vehicle in world.get_actors().filter('vehicle.*'):
        if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
            ego_vehicle = vehicle
            if any(ego_vehicle.id == obs.id for obs in obstacles):  # Compare by ID
                obstacles = [obs for obs in obstacles if obs.id != ego_vehicle.id]  # Remove by ID
            break

    if not ego_vehicle:
        print("[ERROR] No vehicle.lincoln.mkz_2017 found as ego vehicle.")
        return
    cyber.init()

    logger.info("Apollo Cyber RT initialized.")
    apollo_listener = ApolloRoutingListener(carla_world=world, ego_vehicle=ego_vehicle, debug=True)
    apollo_listener.start()
    while not apollo_listener.routing_wps:
        print("Waiting for routing response...")
        time.sleep(0.5)

    routing_waypoints = apollo_listener.routing_wps

    print(f"Received {len(routing_waypoints)} waypoints from routing.")
    # Get ego vehicle location
    ego_location = ego_vehicle.get_location()
    ego_bounding_box = ego_vehicle.bounding_box
    ego_waypoints = []
    vehicle_transform = ego_vehicle.get_transform()
    # Use bounding box corners to calculate lane occupation
    bbox_vertices = ego_bounding_box.get_world_vertices(vehicle_transform)

    for vertex in bbox_vertices:
        ego_waypoints.append(carla_map.get_waypoint(vertex, project_to_road=True, lane_type=carla.LaneType.Driving))
    ego_grid = (int(ego_location.x), int(ego_location.y))

    map_waypoints = carla_map.generate_waypoints(distance=2.0)
    # Randomly generate a target location
    # Assuming map_waypoints is a list of waypoints
    # Randomly generate a target location from driving lanes
    driving_waypoints = [wp for wp in map_waypoints if wp.lane_type == carla.LaneType.Driving]

    if not driving_waypoints:
        raise ValueError("No driving waypoints found in the map.")

    target_waypoint = random.choice(driving_waypoints)
    target_location = target_waypoint.transform.location
    target_grid = (int(target_location.x), int(target_location.y))

    # Set bounds
    x_min, y_min, x_max, y_max = ego_location.x - 100, ego_location.y - 100, ego_location.x + 100, ego_location.y + 100
    bounds = (x_min, y_min, x_max, y_max)

    # Initialize environment
    env = Env(world, bounds, obstacles, carla_map, routing_waypoints, target_waypoint, resolution=1.0, safety_distance=0.5)

    # Check if ego and target are on lane
    # if ego_grid in env.lane_grid:
    #     print("[PASS] Ego vehicle is correctly on the lane grid.")
    # else:
    #     print("[FAIL] Ego vehicle is not on the lane grid.")
    #
    # if target_grid in env.lane_grid:
    #     print("[PASS] Target location is correctly on the lane grid.")
    # else:
    #     print("[FAIL] Target location is not on the lane grid.")

    # Visualize the grid
    Env.visualize_grid(env.lane_grid, env.obs, ego_grid, target_grid)


if __name__ == "__main__":
    main()
