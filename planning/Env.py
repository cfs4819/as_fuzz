import math
import os
import pdb
import sys
import random
import time
import signal

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

        x_min, y_min, x_max, y_max = bounds
        self.x_range = (x_min, x_max)
        self.y_range = (y_min, y_max)
        self.motions = [(-resolution, 0), (-resolution, resolution), (0, resolution), (resolution, resolution),
                        (resolution, 0), (resolution, -resolution), (0, -resolution), (-resolution, -resolution)]

        self.lane_grid = set()  # Grid cells representing lanes
        self.safe_grid_lane = set()  # Grid cells for safety distance
        self.safe_grid_obs = set()  # Grid cells for obstacle safety distance

        self.obs = set()  # Grid cells representing obstacles (expanded by safety distance)

        self.update_lanes()  # Initialize lane grid
        self.update_obs()  # Initialize obstacle grid

    def update_lanes(self):
        """
        Compute the drivable lane grid based on waypoints and lane width.
        """
        self.lane_grid.clear()

        for waypoint in self.waypoints:
            self.add_lane_cells(waypoint)
            # Recursively add lanes to the left and right
            self.recursively_add_lanes(waypoint, direction="left")
            self.recursively_add_lanes(waypoint, direction="right")

    def add_lane_cells(self, waypoint):
        """
        Add grid cells for an entire lane based on a given waypoint, including all points
        on the same road and lane ID. Marks the edges of the lane as safe grid cells if conditions are met.
        """
        target_road_id = waypoint.road_id  # Get the road ID of the given waypoint
        target_lane_id = waypoint.lane_id  # Get the lane ID of the given waypoint
        map_waypoints = self.map_waypoints

        # Filter all waypoints that belong to the same road and lane ID
        lane_waypoints = [
            wp for wp in map_waypoints
            if wp.road_id == target_road_id and wp.lane_id == target_lane_id
        ]

        if not lane_waypoints:
            print(f"[WARNING] No waypoints found for Road ID {target_road_id}, Lane ID {target_lane_id}.")
            return

        temp_lane_grid = set()  # Temporary storage for lane grid points
        temp_safe_grid = set()  # Temporary storage for safety grid points

        for waypoint in lane_waypoints:
            transform = waypoint.transform
            location = transform.location
            lane_width = waypoint.lane_width
            yaw = math.radians(transform.rotation.yaw)

            # Check lane change ability and determine where to add safe grid
            lane_change = waypoint.lane_change

            # Get the left and right lanes
            left_lane = waypoint.get_left_lane()
            right_lane = waypoint.get_right_lane()

            # Check for ID jump for left and right lanes
            left_id_jump = (left_lane is not None and left_lane.lane_id * waypoint.lane_id <= 0) or (left_lane is not None and abs(left_lane.lane_id - waypoint.lane_id) > 1)
            right_id_jump = (right_lane is not None and right_lane.lane_id * waypoint.lane_id <= 0) or (right_lane is not None and abs(right_lane.lane_id - waypoint.lane_id) > 1)

            # Determine whether to add safe grid on the left
            add_left_safe_grid = (
                (lane_change in [carla.LaneChange.NONE, carla.LaneChange.Right])  # Left change not allowed
                or left_lane is None  # No left lane
                or left_id_jump  # ID jump detected
                or (left_lane.lane_type != carla.LaneType.Driving)  # Left lane not drivable
            )

            # Determine whether to add safe grid on the right
            add_right_safe_grid = (
                (lane_change in [carla.LaneChange.NONE, carla.LaneChange.Left])  # Right change not allowed
                or right_lane is None  # No right lane
                or right_id_jump  # ID jump detected
                or (right_lane.lane_type != carla.LaneType.Driving)  # Right lane not drivable
            )

            # Calculate half-width and rotation matrix
            half_width = lane_width / 2.0
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # Extend the lane grid and mark edges as safe grid
            for y_offset in np.arange(-half_width - self.safety_distance, half_width + self.safety_distance,
                                      self.resolution):
                # Determine if the current offset is on a safe edge
                is_left_edge = y_offset < -half_width
                is_right_edge = y_offset > half_width
                is_safe_edge = (is_left_edge and add_left_safe_grid) or (is_right_edge and add_right_safe_grid)

                for x_offset in np.arange(-self.resolution * 3, self.resolution * 3,
                                          self.resolution):  # Lane segment length
                    # Transform local coordinates to global
                    x_global = location.x + x_offset * cos_yaw - y_offset * sin_yaw
                    y_global = location.y + x_offset * sin_yaw + y_offset * cos_yaw
                    grid_point = to_grid(carla.Location(x=x_global, y=y_global), resolution=self.resolution)

                    # Add to lane grid
                    temp_lane_grid.add(grid_point)
                    if is_safe_edge:
                        temp_safe_grid.add(grid_point)

        # Update the main lane grid and safe grid
        self.lane_grid.update(temp_lane_grid)
        self.safe_grid_lane.update(temp_safe_grid)

    def update_obs(self):
        """
        Update obstacle occupancy grid based on obstacle positions and safety distance.
        """
        self.obs.clear()
        self.safe_grid_obs.clear()
        for obstacle in self.obstacles:
            bounding_box = obstacle.bounding_box
            location = obstacle.get_location()

            # Expand obstacle bounding box by the safety distance
            obs_cells = bounding_box_to_grid(bounding_box, location, self.safety_distance)
            self.obs.update(obs_cells)
            safe_cells = expand_grid_with_safety_distance(obs_cells, self.safety_distance // self.resolution)
            self.safe_grid_obs.update(safe_cells)

    def is_occupied(self, x, y):
        """
        Check if a grid cell is occupied (either by an obstacle or outside the lane grid).

        :param x: X-coordinate of the grid cell
        :param y: Y-coordinate of the grid cell
        :return: True if occupied, False otherwise
        """
        return (x, y) in self.obs or (x, y) in self.safe_grid_lane or (x, y) in self.safe_grid_obs or (
            x, y) not in self.lane_grid
        # return (x, y) in self.obs or (x, y) not in self.lane_grid

    def recursively_add_lanes(self, waypoint, direction="left"):
        """
        Recursively add lanes to the left or right until there are no more valid lanes,
        or the lane ID changes abruptly, or lane change is not allowed.

        :param waypoint: The starting waypoint
        :param direction: Direction to check ("left" or "right")
        """
        if direction == "left":
            get_lane_func = waypoint.get_left_lane
            valid_lane_changes = [carla.LaneChange.Left, carla.LaneChange.Both]
        elif direction == "right":
            get_lane_func = waypoint.get_right_lane
            valid_lane_changes = [carla.LaneChange.Right, carla.LaneChange.Both]
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

        current_lane_id = waypoint.lane_id

        while True:
            next_lane = get_lane_func()
            if not next_lane:
                print(f"[INFO] No more lanes to the {direction}.")
                break

            # Stop if lane change is not allowed
            if next_lane.lane_change not in valid_lane_changes:
                print(f"[INFO] Lane change not allowed to the {direction}.")
                break

            # Stop if lane ID changes abruptly
            if abs(next_lane.lane_id - current_lane_id) > 1:
                print(f"[INFO] Lane ID jump detected to the {direction}: {next_lane.lane_id}")
                break

            # Add the lane to the grid
            self.add_lane_cells(next_lane)
            print(f"[INFO] Added lane {next_lane.lane_id} to the {direction}.")

            # Update the current waypoint and lane ID
            current_lane_id = next_lane.lane_id


def save_grid(lane_grid, obs_grid, ego_location, target_location, filename="grid_visualization.png"):
    """
    Save the grid visualization showing lanes and obstacles to a file.

    :param lane_grid: Set of grid points representing lanes.
    :param obs_grid: Set of grid points representing obstacles.
    :param ego_location: Tuple (x, y) of the ego vehicle's location.
    :param target_location: Tuple (x, y) of the target's location.
    :param filename: Filename to save the visualization (default: "grid_visualization.png").
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

    # Save the figure to the specified file
    plt.savefig(filename)
    print(f"Grid visualization saved to {filename}.")
    plt.close()  # Close the plot to free up memory


# def main():
#     from cyber.python.cyber_py3 import cyber
#     from loguru import logger
#     """
#     Main function to continuously update and save grid visualization with Carla.
#     """
#     client = carla.Client('localhost', 4000)
#     client.set_timeout(10.0)
#     world = client.get_world()
#
#     # Get map and initial waypoints
#     carla_map = world.get_map()
#
#     # Initialize Apollo Cyber RT
#     cyber.init()
#     logger.info("Apollo Cyber RT initialized.")
#
#     # Find Ego vehicle with retry mechanism
#     max_retries = 5
#     retry_interval = 2  # seconds
#     resolution = 0.5
#     ego_vehicle = None
#
#     for attempt in range(max_retries):
#         print(f"[INFO] Attempting to find ego vehicle (Attempt {attempt + 1}/{max_retries})...")
#         for vehicle in world.get_actors().filter('vehicle.*'):
#             if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
#                 ego_vehicle = vehicle
#                 print("[INFO] Ego vehicle found.")
#                 break
#         if ego_vehicle:
#             break
#         else:
#             print(f"[WARNING] Ego vehicle not found. Retrying in {retry_interval} seconds...")
#             time.sleep(retry_interval)
#
#     if not ego_vehicle:
#         print("[ERROR] Failed to find vehicle.lincoln.mkz_2017 after multiple attempts.")
#         return
#
#     # Initialize Apollo Routing Listener
#     apollo_listener = ApolloRoutingListener(carla_world=world, ego_vehicle=ego_vehicle, debug=True)
#     apollo_listener.start("routing_test_node")
#
#     def signal_handler(sig, frame):
#         """Handle Ctrl+C to gracefully exit."""
#         print("\n[INFO] Ctrl+C detected. Shutting down...")
#         apollo_listener.stop()
#         cyber.shutdown()
#         sys.exit(0)
#
#     # Register the signal handler
#     signal.signal(signal.SIGINT, signal_handler)
#
#     print("Waiting for routing response...")
#     while not apollo_listener.routing_wps:
#         time.sleep(0.5)
#
#     print("Routing response received. Starting visualization loop...")
#
#     while True:
#         try:
#             # Check if ego vehicle still exists
#             if ego_vehicle is None or ego_vehicle not in world.get_actors():
#                 print("[WARNING] Ego vehicle is missing. Attempting to reacquire...")
#                 ego_vehicle = None
#                 for vehicle in world.get_actors().filter('vehicle.*'):
#                     if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
#                         ego_vehicle = vehicle
#                         print("[INFO] Ego vehicle reacquired.")
#                         break
#                 if ego_vehicle is None:
#                     print("[ERROR] Ego vehicle could not be reacquired. Exiting loop.")
#                     break
#
#             # Get ego vehicle location
#             retry_attempts = 3
#             for attempt in range(retry_attempts):
#                 ego_location = ego_vehicle.get_location()
#                 if ego_location.x != 0 or ego_location.y != 0:
#                     break
#                 print(f"[WARNING] Ego vehicle location returned (0, 0). Retrying... ({attempt + 1}/{retry_attempts})")
#                 time.sleep(0.5)
#             else:
#                 print("[ERROR] Ego vehicle location invalid after retries. Exiting loop.")
#                 break
#
#             ego_bounding_box = ego_vehicle.bounding_box
#             vehicle_transform = ego_vehicle.get_transform()
#
#             # Update routing waypoints
#             routing_waypoints = apollo_listener.routing_wps
#
#             # Update obstacles
#             obstacles = [
#                 actor for actor in world.get_actors()
#                 if (('vehicle' in actor.type_id) and actor.id != ego_vehicle.id)
#             ]
#
#             # Use bounding box corners to calculate lane occupation
#             bbox_vertices = ego_bounding_box.get_world_vertices(vehicle_transform)
#             ego_waypoints = []
#             for vertex in bbox_vertices:
#                 ego_waypoints.append(
#                     carla_map.get_waypoint(vertex, project_to_road=True, lane_type=carla.LaneType.Driving))
#             ego_grid = to_grid(ego_location, resolution=resolution)
#             # Update target location
#             if routing_waypoints and routing_waypoints[-1]:
#                 target_waypoint = routing_waypoints[-1][-1]
#                 target_location = target_waypoint.transform.location
#                 target_grid = to_grid(target_location, resolution=resolution)
#             else:
#                 print("[ERROR] No valid target waypoint found. Exiting loop.")
#                 break
#
#             # Update waypoints for the environment
#             waypoints = [waypoints_e[0] for waypoints_e in routing_waypoints if waypoints_e[0]]
#
#
#
#             # Set the expanded bounds
#             bounds = (x_min, y_min, x_max, y_max)
#             # Initialize environment
#             env = Env(world, bounds, obstacles, carla_map, waypoints, target_waypoint, resolution=resolution,
#                       safety_distance=0.5)
#
#             # Save the grid visualization, overwriting the same file
#             filename = "grid_visualization.png"
#             save_grid(env.lane_grid, env.obs, ego_grid, target_grid, filename=filename)
#             print(f"[INFO] Saved updated grid visualization to {filename}")
#
#             time.sleep(1)  # Adjust the update interval as needed
#
#         except Exception as e:
#             print(f"[ERROR] Exception occurred during main loop: {e}")
#             break


def bounding_box_to_grid(bounding_box, location, resolution):
    """
    Convert a bounding box to grid cells.

    :param bounding_box: Carla bounding box object
    :param location: Location of the bounding box
    :param resolution: Grid resolution (size of each grid cell in meters)
    :return: Set of grid cells representing the bounding box
    """
    grid_cells = set()
    box_extent = bounding_box.extent

    # Define the bounding box limits
    x_min = location.x - box_extent.x
    x_max = location.x + box_extent.x
    y_min = location.y - box_extent.y
    y_max = location.y + box_extent.y

    # Generate grid points within the bounding box limits
    for x in np.arange(x_min, x_max, resolution):
        for y in np.arange(y_min, y_max, resolution):
            grid_cells.add(to_grid(carla.Location(x=x, y=y), resolution))

    return grid_cells


def expand_grid_with_safety_distance(bounding_box_grid, safety_grid):
    """
    Generate grid cells representing the safety distance around each point in the bounding box grid.

    :param bounding_box_grid: Set of grid cells from the bounding box
    :param safety_grid: Safety grid to expand around each grid cell
    :return: Set of grid cells representing only the safety distance
    """
    expanded_cells = set()

    # Iterate through each grid cell in the bounding box
    for cell in bounding_box_grid:
        x_center, y_center = cell  # Center of the current grid cell

        # Calculate the limits for expansion around this grid cell
        x_min = x_center - safety_grid
        x_max = x_center + safety_grid
        y_min = y_center - safety_grid
        y_max = y_center + safety_grid

        # Generate additional grid points within the safety distance
        for x in range(int(x_min), int(x_max) + 1):
            for y in range(int(y_min), int(y_max) + 1):
                distance = ((x - x_center) ** 2 + (y - y_center) ** 2) ** 0.5
                if distance <= safety_grid:
                    # Ensure the expanded cell is not part of the original bounding box grid
                    if (x, y) not in bounding_box_grid:
                        expanded_cells.add((x, y))

    return expanded_cells


def to_grid(location, resolution=1.0):
    """
    Convert a Carla location to grid coordinates.

    :param location: Carla location object
    :param resolution: Grid resolution (size of each grid cell in meters)
    :return: (x, y) grid coordinates
    """
    return int(location.x / resolution), int(location.y / resolution)


def to_carla(grid_point, resolution=1.0):
    """Convert grid coordinates to Carla coordinates."""
    return grid_point[0] * resolution, grid_point[1] * resolution


if __name__ == "__main__":
    from cyber.python.cyber_py3 import cyber

    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Exiting...")
        cyber.shutdown()
        sys.exit(0)
