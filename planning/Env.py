import math
import os
import pdb
import sys

import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, parent_dir)

import carla


class Env:
    def __init__(self, world, bounds, obstacles, map, waypoints, resolution, safety_distance):
        """
        Initialize the environment for Carla simulation.

        :param world: Carla world object
        :param bounds: (x_min, y_min, x_max, y_max), bounding box of the environment
        :param obstacles: List of Carla Actor objects representing obstacles
        :param map: Carla map object
        :param waypoints: List of Carla waypoint objects
        :param resolution: Grid resolution (size of each grid cell in meters)
        :param safety_distance: Safety distance to expand around obstacles
        """
        self.world = world
        self.bounds = bounds
        self.obstacles = obstacles
        self.map = map
        self.map_waypoints = map.generate_waypoints(distance=resolution)
        self.waypoints = waypoints
        self.resolution = resolution
        self.safety_distance = safety_distance

        x_min, y_min, x_max, y_max = bounds
        self.x_range = (x_min, x_max)
        self.y_range = (y_min, y_max)
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

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
            # Compute inner lane grid and safe boundary grid

        inner_lane_grid, safe_boundary_grid = compute_inner_safe_grid(self.lane_grid, self.resolution,
                                                                      self.safety_distance)

        # Update the safe grid and lane grid
        self.safe_grid_lane = safe_boundary_grid
        self.lane_grid = inner_lane_grid

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

        # Generate lane points (entire lane)
        lane_points = generate_lane_points(lane_waypoints, self.resolution)

        # Add all lane points to the grid
        temp_lane_grid = set()
        for point in lane_points:
            grid_point = to_grid(point, self.resolution)
            temp_lane_grid.add(grid_point)
        # Update the main lane grid and safe grid
        self.lane_grid.update(temp_lane_grid)

    def update_obs(self):
        """
        Update obstacle occupancy grid based on obstacle positions and safety distance.
        """
        self.obs.clear()
        self.safe_grid_obs.clear()
        for obstacle in self.obstacles:

            # Expand obstacle bounding box by the safety distance
            obs_cells = bounding_box_to_grid(obstacle, self.resolution)
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

        lane = waypoint

        while True:
            if not lane:
                # print(f"[INFO] No more lanes to the {direction}.")
                break

            # Stop if lane change is not allowed
            if lane.lane_change not in valid_lane_changes:
                # print(f"[INFO] Lane change not allowed to the {direction}.")
                break
            lane = get_lane_func()
            if not lane:
                break
            # Add the lane to the grid
            self.add_lane_cells(lane)


def bounding_box_to_grid(obstacle, resolution):
    """
    Convert a bounding box to grid cells while considering the obstacle's position and orientation.
    :param obstacle: Carla Actor object representing the obstacle
    :param resolution: Grid resolution (size of each grid cell in meters)
    :return: Set of grid cells representing the bounding box
    """
    bounding_box = obstacle.bounding_box
    location = obstacle.get_location()
    rotation = obstacle.get_transform().rotation  # Get the obstacle's rotation
    grid_cells = set()
    box_extent = bounding_box.extent

    # Precompute sine and cosine of yaw for rotation
    yaw = math.radians(rotation.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Generate corner points of the bounding box in local space
    corners = [
        carla.Location(x=-box_extent.x, y=-box_extent.y),
        carla.Location(x=-box_extent.x, y=box_extent.y),
        carla.Location(x=box_extent.x, y=-box_extent.y),
        carla.Location(x=box_extent.x, y=box_extent.y)
    ]

    # Transform the corners to global space
    global_corners = []
    for corner in corners:
        # Apply rotation
        rotated_x = corner.x * cos_yaw - corner.y * sin_yaw
        rotated_y = corner.x * sin_yaw + corner.y * cos_yaw

        # Translate to the obstacle's global position
        global_x = rotated_x + location.x
        global_y = rotated_y + location.y

        global_corners.append((global_x, global_y))

    # Determine the bounding box limits in global space
    x_min = min(c[0] for c in global_corners)
    x_max = max(c[0] for c in global_corners)
    y_min = min(c[1] for c in global_corners)
    y_max = max(c[1] for c in global_corners)

    # Generate grid points within the transformed bounding box limits
    for x in np.arange(x_min, x_max, resolution):
        for y in np.arange(y_min, y_max, resolution):
            # Transform the grid point back to local space to check if it is inside the bounding box
            local_x = (x - location.x) * cos_yaw + (y - location.y) * sin_yaw
            local_y = -(x - location.x) * sin_yaw + (y - location.y) * cos_yaw

            if -box_extent.x <= local_x <= box_extent.x and -box_extent.y <= local_y <= box_extent.y:
                grid_cells.add(to_grid(carla.Location(x=x, y=y), resolution))

    return grid_cells


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def generate_lane_points(lane_waypoints, resolution):
    """
    Generate all points within a lane based on the center points, lane direction, and width.

    :param lane_waypoints: List of waypoints representing the center of the lane.
    :param resolution: The resolution (grid size) to discretize the lane points.
    :return: A list of (x, y) tuples representing all points within the lane.
    """
    lane_points = []

    for waypoint in lane_waypoints:
        # Get lane width and yaw (direction)
        half_width = waypoint.lane_width / 2.0  # Half lane width
        transform = waypoint.transform
        location = transform.location
        yaw = math.radians(transform.rotation.yaw)  # Convert yaw to radians

        # Get unit vectors for the lane direction (yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Perpendicular direction (normal to yaw direction)
        # The perpendicular direction is given by rotating yaw by ±90 degrees
        # x' = -sin(yaw), y' = cos(yaw) gives the perpendicular unit vector
        perpendicular_cos = -sin_yaw
        perpendicular_sin = cos_yaw

        # Generate all points across the lane width at the current waypoint
        offsets = np.arange(-half_width, half_width + resolution, resolution)
        for offset in offsets:
            # Compute the point's global position
            x = location.x + offset * perpendicular_cos  # Perpendicular offset in x
            y = location.y + offset * perpendicular_sin  # Perpendicular offset in y
            # Add the point to the lane points list
            lane_points.append(Point(x, y))

    return lane_points


def generate_lane_edges(lane_waypoints):
    """
    Generate points for the left and right edges of a lane based on the center points.

    :param lane_waypoints: List of waypoints representing the center of the lane.
    :return: Two lists of points [(x_left, y_left)], [(x_right, y_right)] for the left and right edges.
    """
    left_edge_points = []  # Points on the left edge
    right_edge_points = []  # Points on the right edge

    for waypoint in lane_waypoints:
        # Get lane width and yaw (direction)
        half_width = waypoint.lane_width / 2.0  # Half lane width
        transform = waypoint.transform
        location = transform.location
        yaw = math.radians(transform.rotation.yaw)  # Convert yaw to radians

        # Get unit vectors for the perpendicular direction (normal to yaw direction)
        perpendicular_cos = -math.sin(yaw)  # Perpendicular offset x component
        perpendicular_sin = math.cos(yaw)  # Perpendicular offset y component

        # Calculate left edge point
        x_left = location.x + half_width * perpendicular_cos
        y_left = location.y + half_width * perpendicular_sin
        left_edge_points.append(Point(x_left, y_left))

        # Calculate right edge point
        x_right = location.x - half_width * perpendicular_cos
        y_right = location.y - half_width * perpendicular_sin
        right_edge_points.append(Point(x_right, y_right))

    return left_edge_points, right_edge_points


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


def compute_inner_safe_grid(lane_grid, resolution, safety_distance, min_neighbors=3, neighbor_check_x=3):
    """
    Compute the inner lane grid and safe boundary grid for a given lane.

    :param lane_grid: Set of grid points representing the lane grid.
    :param resolution: Grid resolution (size of each grid cell in meters).
    :param safety_distance: Safety distance (thickness) to check neighboring points.
    :param min_neighbors: Minimum number of neighbors a grid cell must have in `lane_grid` to be considered inner.
    :param neighbor_check_x: Range in grid units to check neighbors in cardinal directions (up, down, left, right).
    :return: A tuple (inner_lane_grid, safe_boundary_grid).
    """
    # Safety distance in grid units
    safety_distance_in_grids = int(safety_distance / resolution)

    # Initialize the safe boundary grid
    safe_boundary_grid = set()

    # Directions to check (include all neighbors within the square range)
    directions = [
        (dx, dy)
        for dx in range(-safety_distance_in_grids, safety_distance_in_grids + 1)
        for dy in range(-safety_distance_in_grids, safety_distance_in_grids + 1)
        if not (dx == 0 and dy == 0)  # Exclude the point itself
    ]

    # Iterate over all points in lane_grid
    for point in lane_grid:
        x, y = point

        # Count the number of neighbors within the safety distance
        not_neighbor_count = 0
        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            if neighbor not in lane_grid:
                not_neighbor_count += 1

            # If we already have enough neighbors, break early
            if not_neighbor_count >= min_neighbors:
                break

        # If the number of neighbors is less than `min_neighbors`, mark it as a safe boundary point
        if not_neighbor_count >= min_neighbors:
            safe_boundary_grid.add(point)

    # Additional check to refine safe_boundary_grid
    directions_cardinal = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # Cardinal directions: up, down, left, right
    points_to_remove = set()

    for point in safe_boundary_grid:
        x, y = point
        valid_direction_count = 0

        for dx, dy in directions_cardinal:
            # Check the line in the current direction for lane_grid points
            for step in range(1, neighbor_check_x + 1):
                neighbor = (x + dx * step, y + dy * step)
                if neighbor in lane_grid:
                    valid_direction_count += 1
                    break  # Found a valid point in this direction, move to the next direction

        # If the point has enough valid directions, mark it for removal
        if valid_direction_count >= 4:
            points_to_remove.add(point)

    # Remove points marked for removal from safe_boundary_grid
    safe_boundary_grid -= points_to_remove

    # Inner lane grid is lane_grid minus safe_boundary_grid
    inner_lane_grid = lane_grid - safe_boundary_grid

    return inner_lane_grid, safe_boundary_grid


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
