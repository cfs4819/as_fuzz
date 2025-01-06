import carla
import numpy as np
import math


class Env:
    def __init__(self, world, bounds, obstacles, map, waypoints, resolution=1.0, safety_distance=1.0):
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
        self.waypoints = waypoints
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

            # Add lanes to the left and right if available
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                self.add_lane_cells(left_lane)

            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                self.add_lane_cells(right_lane)

    def add_lane_cells(self, waypoint):
        """
        Add grid cells for a single lane based on a waypoint, considering the rotation of the lane.
        """
        current_waypoint = waypoint
        while current_waypoint:
            transform = current_waypoint.transform
            location = transform.location
            lane_width = current_waypoint.lane_width
            yaw = math.radians(transform.rotation.yaw)

            # Lane length for this segment (estimated based on resolution)
            lane_length = self.resolution * 3  # Assuming each lane segment covers ~3 grid cells

            # Calculate the 4 corners of the rotated rectangle
            half_width = lane_width / 2.0
            half_length = lane_length / 2.0

            corners = [
                (-half_length, -half_width),  # Bottom-left in local coordinates
                (-half_length, half_width),  # Top-left
                (half_length, half_width),  # Top-right
                (half_length, -half_width)  # Bottom-right
            ]

            # Transform corners to global coordinates
            global_corners = [
                (
                    location.x + x * math.cos(yaw) - y * math.sin(yaw),
                    location.y + x * math.sin(yaw) + y * math.cos(yaw)
                )
                for x, y in corners
            ]

            # Get bounding box of the rectangle
            x_min = min(c[0] for c in global_corners)
            x_max = max(c[0] for c in global_corners)
            y_min = min(c[1] for c in global_corners)
            y_max = max(c[1] for c in global_corners)

            # Traverse bounding box and fill points inside the rectangle
            for x in np.arange(x_min, x_max, self.resolution):
                for y in np.arange(y_min, y_max, self.resolution):
                    if self._is_point_inside_rotated_rectangle((x, y), global_corners):
                        self.lane_grid.add((int(x), int(y)))

            # Advance to the next waypoint
            next_waypoints = current_waypoint.next(self.resolution)
            if not next_waypoints:
                break
            current_waypoint = next_waypoints[0]

    def _is_point_inside_rotated_rectangle(self, point, corners):
        """
        Check if a point is inside a rotated rectangle using vector cross products.

        :param point: (x, y) tuple of the point
        :param corners: List of (x, y) tuples representing the corners of the rectangle (ordered counterclockwise)
        :return: True if point is inside the rectangle, False otherwise
        """
        px, py = point
        for i in range(len(corners)):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % len(corners)]  # Next corner
            # Cross product to check if the point is on the inside of the edge
            if (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) < 0:
                return False
        return True

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
        Convert a bounding box to occupied grid cells, expanded by safety distance.

        :param bounding_box: Carla bounding box object
        :param location: Location of the bounding box
        :param safety_distance: Distance to expand around the bounding box
        :return: Set of occupied grid cells
        """
        obs_cells = set()
        box_extent = bounding_box.extent

        # Expand the bounding box by safety distance
        x_min = location.x - box_extent.x - safety_distance
        x_max = location.x + box_extent.x + safety_distance
        y_min = location.y - box_extent.y - safety_distance
        y_max = location.y + box_extent.y + safety_distance

        # Generate occupied grid cells
        for x in np.arange(x_min, x_max, self.resolution):
            for y in np.arange(y_min, y_max, self.resolution):
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
