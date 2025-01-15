"""
RTAAstar 2D (Real-time Adaptive A*)
Modified for Carla Simulator Integration
"""

import signal
import sys
import time
import traceback
import carla
import numpy as np
import plotly.graph_objects as go

import math

from planning.Env import Env, to_grid, to_carla, generate_lane_edges
from ms_utils.apollo_routing_listener import ApolloRoutingListener


class DStar:
    def __init__(self, s_start, s_goal, env):
        self.s_start, self.s_goal = s_start, s_goal
        self.Env = env

        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.OPEN = set()
        self.t = {}
        self.PARENT = {}
        self.h = {}
        self.k = {}
        self.path = []
        self.visited = set()

    def init(self):
        """
        Initialize the heuristic table and all states using bounds.
        """
        x_min, y_min, x_max, y_max = self.Env.bounds  # Get bounds from the environment

        # Iterate over the bounded grid range
        for i in range(int(x_min), int(x_max) + 1):
            for j in range(int(y_min), int(y_max) + 1):
                self.t[(i, j)] = 'NEW'
                self.k[(i, j)] = 0.0
                self.h[(i, j)] = float("inf")
                self.PARENT[(i, j)] = None

        # Set the goal node's heuristic value to 0
        self.h[self.s_goal] = 0.0
        self.insert(self.s_goal, 0)

    def run(self):
        """
        Perform the initial run to compute the path from start to goal.
        Returns:
            - A list of the path if found.
            - None if no path exists.
        """
        while True:
            k_min = self.process_state()

            # If OPEN set is empty, no path exists
            if k_min == -1:
                print("[ERROR] No valid path found.")
                return None

            # If start node is closed, the path is found
            if self.t[self.s_start] == 'CLOSED':
                break
        # Extract path from start to goal
        self.path = self.extract_path(self.s_start, self.s_goal)
        return self.path

    def extract_path(self, s_start, s_goal):
        """
        Extract the path from start to goal.
        """
        path = [s_start]
        s = s_start
        while s != s_goal:
            s = self.PARENT[s]
            path.append(s)
        return path

    def process_state(self):
        """
        Process a single state in D* algorithm.
        """
        s = self.min_state()  # Get node with minimum k-value
        if s is None:
            return -1  # No more nodes in OPEN set

        k_old = self.get_k_min()
        self.delete(s)  # Move state s from OPEN to CLOSED

        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.h[s_n] <= k_old and self.h[s] > self.h[s_n] + self.cost(s_n, s):
                    self.PARENT[s] = s_n
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)
        elif k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                    (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                    (self.h[s_n] > self.h[s] + self.cost(s, s_n)):
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))

    def min_state(self):
        return min(self.OPEN, key=lambda x: self.k[x]) if self.OPEN else None

    def get_k_min(self):
        return min([self.k[x] for x in self.OPEN]) if self.OPEN else -1

    def insert(self, s, h_new):
        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)

        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'
        self.OPEN.remove(s)

    def get_neighbor(self, s):
        """
        Get all feasible neighbors of a given state.
        """
        neighbors = set()
        x_min, x_max = self.Env.x_range
        y_min, y_max = self.Env.y_range

        for u in self.u_set:
            s_next = (s[0] + u[0], s[1] + u[1])
            # Ensure neighbors are within grid bounds and not in obstacles
            if x_min <= s_next[0] <= x_max and y_min <= s_next[1] <= y_max and not self.Env.is_occupied(s_next[0],
                                                                                                        s_next[1]):
                neighbors.add(s_next)

        return neighbors

    def cost(self, s_start, s_goal):
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])


def path_to_waypoints(path, resolution, carla_map):
    """
    Convert a path of grid points into a list of waypoints for driving.

    :param path: List of grid cells representing the planned path.
    :param resolution: Resolution (size of each grid cell in meters).
    :param carla_map: Carla map object to fetch waypoints.
    :return: List of waypoints along the path.
    """

    def to_physical(grid_point):
        """Convert a grid point to real-world coordinates."""
        try:
            return grid_point[0] * resolution, grid_point[1] * resolution
        except Exception as e:
            print(f"[ERROR] Invalid grid point {grid_point}: {e}")
            return None, None

    waypoints = []
    for grid_point in path:
        # Convert grid point to real-world coordinates
        world_x, world_y = to_physical(grid_point)
        if world_x is not None and world_y is not None:
            # Use Carla's map to fetch a waypoint at the real-world location
            location = carla.Location(x=world_x, y=world_y, z=0.0)  # Assuming z=0.0
            waypoint = carla_map.get_waypoint(location)
            if waypoint:
                waypoints.append(waypoint)

    return waypoints


def save_grid_plotly(carla_map, ego_vehicle, lane_grid, obs, target_grid, path, bounds, resolution, safe_grid=None,
                     filename="grid_visualization.png"):
    """
    Save grid visualization to a file with the planned path, in real-world coordinates using Plotly.
    :param carla_map: Carla map object to fetch waypoints.
    :param ego_vehicle: Ego vehicle object.
    :param lane_grid: Set of grid points representing the lane grid.
    :param obs: Set of obstacle grid cells.
    :param target_grid: Tuple representing the target's grid position.
    :param path: List of grid cells representing the planned path.
    :param resolution: Resolution to adjust the path points.
    :param bounds: Bounds of the area to visualize (x_min, y_min, x_max, y_max).
    :param safe_grid: Set of safe grid cells (optional).
    :param filename: Filename to save the visualization.
    """

    # Convert grid coordinates to physical coordinates
    def to_physical(grid_point):
        """Convert a grid point to real-world coordinates."""
        return grid_point[0] * resolution, grid_point[1] * resolution
        # Extract bounds

    x_min, y_min, x_max, y_max = bounds

    # Generate waypoints from Carla map
    print("[INFO] Generating waypoints from Carla map...")
    waypoints = carla_map.generate_waypoints(distance=resolution)

    # Filter waypoints within bounds
    bounded_waypoints = [
        wp for wp in waypoints
        if x_min <= wp.transform.location.x <= x_max and y_min <= wp.transform.location.y <= y_max
    ]

    # Use `generate_lane_edges` to compute lane edges for all bounded waypoints
    left_edge_points, right_edge_points = generate_lane_edges(bounded_waypoints)

    # Separate x and y coordinates for left and right edges
    left_edge_x = [-point.x for point in left_edge_points]  # Flip X-axis
    left_edge_y = [point.y for point in left_edge_points]

    right_edge_x = [-point.x for point in right_edge_points]  # Flip X-axis
    right_edge_y = [point.y for point in right_edge_points]

    edge_x = left_edge_x + right_edge_x
    edge_y = left_edge_y + right_edge_y

    # Initialize figure
    fig = go.Figure()

    # Draw lane edges in real-world coordinates
    fig.add_trace(go.Scatter(
        x=edge_x,
        y=edge_y,
        mode='markers',
        marker=dict(color='grey', size=resolution*2),
        name="Lane Edge"
    ))



    # Draw lane grids in real-world coordinates
    if lane_grid:
        lane_points = np.array([to_physical(cell) for cell in lane_grid])
        fig.add_trace(go.Scatter(
            x=-lane_points[:, 0],
            y=lane_points[:, 1],
            mode='markers',
            marker=dict(color='green', size=4),
            name="Lane"
        ))

    # Draw safe grids in real-world coordinates
    if safe_grid:
        safe_points = np.array([to_physical(cell) for cell in safe_grid])
        fig.add_trace(go.Scatter(
            x=-safe_points[:, 0],
            y=safe_points[:, 1],
            mode='markers',
            marker=dict(color='cyan', size=4),
            name="Safe Grid"
        ))

    # Draw obstacle grids in real-world coordinates
    if obs:
        obstacle_points = np.array([to_physical(cell) for cell in obs])
        fig.add_trace(go.Scatter(
            x=-obstacle_points[:, 0],
            y=obstacle_points[:, 1],
            mode='markers',
            marker=dict(color='red', size=4),
            name="Obstacle"
        ))

    # Draw the target point
    target_x, target_y = to_physical(target_grid)
    fig.add_trace(go.Scatter(
        x=[-target_x],
        y=[target_y],
        mode='markers',
        marker=dict(color='orange', size=10),
        name="Target Point"
    ))

    # Draw the planned path
    if path:
        path_points = np.array([to_physical(p) for p in path])
        fig.add_trace(go.Scatter(
            x=-path_points[:, 0],
            y=path_points[:, 1],
            mode='lines',
            line=dict(color='blue', width=2),
            name="Planned Path"
        ))

    # Draw the ego vehicle as a triangle
    ego_location = ego_vehicle.get_location()
    ego_x, ego_y = ego_location.x, ego_location.y
    ego_yaw = math.radians(ego_vehicle.get_transform().rotation.yaw)

    triangle_size = 2.0  # Size of the triangle
    half_width = triangle_size / 2.0

    # Compute the vertices of the triangle
    front_x = ego_x + triangle_size * math.cos(ego_yaw)
    front_y = ego_y + triangle_size * math.sin(ego_yaw)
    rear_left_x = ego_x - half_width * math.cos(ego_yaw) - half_width * math.sin(ego_yaw)
    rear_left_y = ego_y - half_width * math.sin(ego_yaw) + half_width * math.cos(ego_yaw)
    rear_right_x = ego_x - half_width * math.cos(ego_yaw) + half_width * math.sin(ego_yaw)
    rear_right_y = ego_y - half_width * math.sin(ego_yaw) - half_width * math.cos(ego_yaw)

    # Add the three edges of the triangle
    fig.add_trace(go.Scatter(
        x=[-front_x, -rear_left_x],
        y=[front_y, rear_left_y],
        mode='lines',
        line=dict(color='purple', width=2),
        name="Ego Vehicle",
    ))

    fig.add_trace(go.Scatter(
        x=[-rear_left_x, -rear_right_x],
        y=[rear_left_y, rear_right_y],
        mode='lines',
        line=dict(color='purple', width=2),
        showlegend=False
    ))

    fig.add_trace(go.Scatter(
        x=[-rear_right_x, -front_x],
        y=[rear_right_y, front_y],
        mode='lines',
        line=dict(color='purple', width=2),
        showlegend=False
    ))

    # Set axis labels and layout
    fig.update_layout(
        title="Grid Visualization in Real-World Coordinates",
        xaxis_title="X (meters)",
        yaxis_title="Y (meters)",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        yaxis=dict(scaleanchor="x", scaleratio=1),
        showlegend=True,
        width=800,
        height=800
    )
    fig.update_layout(
        xaxis=dict(
            title="X (meters)",
            showgrid=False,
            zeroline=False,
            showline=True,
            linecolor='black',
            linewidth=1
        ),
        yaxis=dict(
            title="Y (meters)",
            showgrid=False,
            zeroline=False,
            showline=True,
            linecolor='black',
            linewidth=1
        ),
        # plot_bgcolor='rgba(0,0,0,0)',
        # paper_bgcolor='rgba(0,0,0,0)'
    )

    # Save the figure as a static image
    fig.write_image(filename)


def save_grid(ego_vehicle, lane_grid, obs, target_grid, path, carla_map, bounds, resolution, safe_grid=None,
              filename="grid_visualization.png"):
    """
    Save grid visualization to a file with the planned path, in real-world coordinates.
    :param ego_vehicle: Ego vehicle object.
    :param lane_grid: Set of grid points representing the lane grid.
    :param obs: Set of obstacle grid cells.
    :param target_grid: Tuple representing the target's grid position.
    :param path: List of grid cells representing the planned path.
    :param resolution: Resolution to adjust the path points.
    :param carla_map: Carla map object to fetch waypoints.
    :param bounds: Bounds of the area to visualize (x_min, y_min, x_max, y_max).
    :param safe_grid: Set of safe grid cells (optional).
    :param filename: Filename to save the visualization.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.use('Agg')

    plt.figure(figsize=(10, 10))
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')

    # Convert grid coordinates to physical coordinates
    def to_physical(grid_point):
        """Convert a grid point to real-world coordinates."""
        try:
            return grid_point[0] * resolution, grid_point[1] * resolution
        except Exception as e:
            print(f"[ERROR] Invalid grid point {grid_point}: {e}")
            return None, None

    # Extract bounds
    x_min, y_min, x_max, y_max = bounds
    # Generate waypoints from Carla map
    print("[INFO] Generating waypoints from Carla map...")
    waypoints = carla_map.generate_waypoints(distance=resolution)

    # Filter waypoints within bounds
    bounded_waypoints = [
        wp for wp in waypoints
        if x_min <= wp.transform.location.x <= x_max and y_min <= wp.transform.location.y <= y_max
    ]

    # Use `generate_lane_edges` to compute lane edges for all bounded waypoints
    left_edge_points, right_edge_points = generate_lane_edges(bounded_waypoints)

    # Separate x and y coordinates for left and right edges
    left_edge_x = [-point.x for point in left_edge_points]  # Flip X-axis
    left_edge_y = [point.y for point in left_edge_points]

    right_edge_x = [-point.x for point in right_edge_points]  # Flip X-axis
    right_edge_y = [point.y for point in right_edge_points]

    # Plot the left and right lane edges
    plt.scatter(left_edge_x, left_edge_y, c='grey', s=2, label="Lane Edge", zorder=0)
    plt.scatter(right_edge_x, right_edge_y, c='grey', s=2, label=None, zorder=0)
    print("[INFO] Lane edges plotted.")
    # Draw lane grids in real-world coordinates (second bottom layer)
    lane_points = np.array([to_physical(cell) for cell in lane_grid if to_physical(cell) != (None, None)])
    if lane_points.size > 0:
        plt.plot(-lane_points[:, 0], lane_points[:, 1], 'g.', markersize=4, label="Lane", zorder=1)  # Flip X-axis

    # Draw safe grids in real-world coordinates (middle layer)
    if safe_grid:
        safe_points = np.array([to_physical(cell) for cell in safe_grid if to_physical(cell) != (None, None)])
        if safe_points.size > 0:
            plt.plot(-safe_points[:, 0], safe_points[:, 1], 'c.', markersize=4, label="Safe Grid",
                     zorder=2)  # Flip X-axis

    # Draw obstacle grids in real-world coordinates (middle layer)
    obstacle_points = np.array([to_physical(cell) for cell in obs if to_physical(cell) != (None, None)])
    if obstacle_points.size > 0:
        plt.plot(-obstacle_points[:, 0], obstacle_points[:, 1], 'r.', markersize=4, label="Obstacle",
                 zorder=3)  # Flip X-axis

    # Draw the target point in real-world coordinates (top layer)
    target_x, target_y = to_physical(target_grid)
    plt.scatter([-target_x], [target_y], c='orange', s=75, label="Target Point", zorder=6)  # Flip X-axis

    # Draw the planned path in real-world coordinates (above grid but below vehicle)
    if path:
        path_points = np.array([to_physical(p) for p in path])
        plt.plot(-path_points[:, 0], path_points[:, 1], 'b-', linewidth=2, label="Planned Path",
                 zorder=4)  # Flip X-axis

        # Convert path to waypoints
        planned_waypoints = path_to_waypoints(path, resolution, carla_map)

        # Draw the planned waypoints
        waypoint_x = [-wp.transform.location.x for wp in planned_waypoints]  # Flip X-axis
        waypoint_y = [wp.transform.location.y for wp in planned_waypoints]
        plt.plot(waypoint_x, waypoint_y, 'm--', linewidth=1, label="Planned Waypoints", zorder=7)

    # Draw the ego vehicle as a triangle
    ego_location = ego_vehicle.get_location()
    ego_grid = to_grid(ego_location, resolution)
    ego_x, ego_y = to_physical(ego_grid)
    ego_yaw = math.radians(ego_vehicle.get_transform().rotation.yaw)

    # Triangle size (vehicle representation)
    triangle_size = 2.0  # Adjust size as necessary
    half_width = triangle_size / 2.0

    # Compute the three vertices of the triangle
    # Front vertex (tip of the triangle in the direction of the vehicle's yaw)
    front_x = ego_x + triangle_size * math.cos(ego_yaw)
    front_y = ego_y + triangle_size * math.sin(ego_yaw)

    # Rear-left vertex
    rear_left_x = ego_x - half_width * math.cos(ego_yaw) - half_width * math.sin(ego_yaw)
    rear_left_y = ego_y - half_width * math.sin(ego_yaw) + half_width * math.cos(ego_yaw)

    # Rear-right vertex
    rear_right_x = ego_x - half_width * math.cos(ego_yaw) + half_width * math.sin(ego_yaw)
    rear_right_y = ego_y - half_width * math.sin(ego_yaw) - half_width * math.cos(ego_yaw)

    # Flip X-axis for plotting
    front_x = -front_x
    rear_left_x = -rear_left_x
    rear_right_x = -rear_right_x

    # Plot the triangle for the ego vehicle
    plt.fill(
        [front_x, rear_left_x, rear_right_x],
        [front_y, rear_left_y, rear_right_y],
        color='purple',
        alpha=0.7,
        zorder=5
    )

    # Add legend to the plot
    plt.legend()

    # Set axis labels, title, and save the plot
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.title("Grid Visualization in Real-World Coordinates (X-axis Flipped)")

    plt.savefig(filename)
    print(f"[INFO] Grid visualization saved to {filename}")
    plt.close()


def main():
    from cyber.python.cyber_py3 import cyber
    from loguru import logger

    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # Initialize Apollo Cyber RT
    cyber.init()
    logger.info("Apollo Cyber RT initialized.")

    # Find Ego vehicle
    max_retries = 5
    retry_interval = 2  # seconds
    resolution = 0.5
    ego_vehicle = None

    for attempt in range(max_retries):
        print(f"[INFO] Attempting to find ego vehicle (Attempt {attempt + 1}/{max_retries})...")
        for vehicle in world.get_actors().filter('vehicle.*'):
            if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
                ego_vehicle = vehicle
                print("[INFO] Ego vehicle found.")
                break
        if ego_vehicle:
            break
        else:
            print(f"[WARNING] Ego vehicle not found. Retrying in {retry_interval} seconds...")
            time.sleep(retry_interval)

    if not ego_vehicle:
        print("[ERROR] Failed to find vehicle.lincoln.mkz_2017 after multiple attempts.")
        return

    # Initialize Apollo Routing Listener
    apollo_listener = ApolloRoutingListener(carla_world=world, ego_vehicle=ego_vehicle, debug=True)
    apollo_listener.start("routing_test_node")

    def signal_handler(sig, frame):
        """Handle Ctrl+C to gracefully exit."""
        print("\n[INFO] Ctrl+C detected. Shutting down...")
        print("[INFO] Current code execution point before shutdown:")

        # Print the current stack trace
        stack_trace = traceback.format_stack(frame)
        for line in stack_trace:
            print(line.strip())

        # Ensure Apollo listener stops gracefully
        apollo_listener.stop()
        sys.exit(0)

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    print("Waiting for routing response...")
    while not apollo_listener.routing_wps:
        time.sleep(0.5)

    print("Routing response received. Starting visualization loop...")

    # try:
    #     while True:
    # Check if ego vehicle still exists
    if ego_vehicle is None or ego_vehicle not in world.get_actors():
        print("[WARNING] Ego vehicle is missing. Attempting to reacquire...")
        ego_vehicle = None
        for vehicle in world.get_actors().filter('vehicle.*'):
            if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
                ego_vehicle = vehicle
                print("[INFO] Ego vehicle reacquired.")
        if ego_vehicle is None:
            print("[ERROR] Ego vehicle could not be reacquired. Exiting loop.")

    # Get Ego vehicle location
    ego_location = ego_vehicle.get_location()
    ego_grid = (int(ego_location.x / resolution), int(ego_location.y / resolution))

    # Update obstacles
    obstacles = set()  # Ensure obstacles are a set for quick lookup
    obstacle_grids = set()  # To store obstacle grid coordinates

    for actor in world.get_actors():
        if "vehicle" in actor.type_id and actor.id != ego_vehicle.id:
            obstacles.add(actor)
            # Convert obstacle location to grid coordinates
            obstacle_location = actor.get_location()
            obstacle_grid = to_grid(obstacle_location, resolution)
            obstacle_grids.add(obstacle_grid)

    # Update routing waypoints
    routing_waypoints = apollo_listener.routing_wps
    if routing_waypoints:
        waypoints = [wp[0] for wp in routing_waypoints if wp[0]]
        target_waypoint = routing_waypoints[-1][-1]
        target_location = target_waypoint.transform.location
        target_grid = to_grid(target_location, resolution)  # Convert target location to grid coordinates

        # Calculate bounds to cover all waypoints and obstacles in grid coordinates
        grid_waypoints = [to_grid(wp.transform.location, resolution) for wp in waypoints]

        # Collect all grid coordinates from waypoints and obstacles
        all_grids = grid_waypoints + list(obstacle_grids) + [target_grid]
        x_min = min(wp[0] for wp in all_grids)
        y_min = min(wp[1] for wp in all_grids)
        x_max = max(wp[0] for wp in all_grids)
        y_max = max(wp[1] for wp in all_grids)

        # Add a buffer to the bounds for safety
        buffer = int(100 / resolution)  # Convert buffer to grid units
        x_min -= buffer
        y_min -= buffer
        x_max += buffer
        y_max += buffer
    else:
        print("[ERROR] No valid routing waypoints available. Exiting loop.")

    # Set bounds around ego vehicle in grid coordinates
    bounds = (x_min, y_min, x_max, y_max)
    print(f"[INFO] Bounds set to {bounds}")

    # Initialize environment with updated obstacles
    env = Env(world, bounds, obstacles, carla_map, waypoints, resolution=resolution,
              safety_distance=1.5)

    # Initialize DStar planner
    planner = DStar(s_start=ego_grid, s_goal=target_grid, env=env)

    # Perform planning
    planner.init()
    is_path_found = planner.run()

    # Save grid visualization
    planned_path = planner.path  # Extract the planned path
    carla_navigation_path = []
    for grid_point in planned_path:
        world_x = grid_point[0] * resolution
        world_y = grid_point[1] * resolution
        waypoint = carla_map.get_waypoint(carla.Location(x=world_x, y=world_y, z=0.0))
        if waypoint:
            carla_navigation_path.append(waypoint)
        else:
            print(f"[WARNING] No valid waypoint found for grid point {grid_point}")

    # Check if a valid path was generated
    if carla_navigation_path:
        print(f"[INFO] Navigation path with {len(carla_navigation_path)} waypoints generated.")
    else:
        print("[INFO] No valid navigation path could be generated.")
    filename = f"grid_visualization_{int(time.time())}.png"
    safe_grid = env.safe_grid_lane.union(env.safe_grid_obs)
    save_grid(ego_vehicle, env.lane_grid, env.obs, target_grid, planned_path, carla_map, bounds,
              resolution=resolution,
              safe_grid=safe_grid, filename=filename)
    print(f"[INFO] Grid visualization saved to {filename}")

    if is_path_found:
        print(f"[INFO] Path found at iteration {time.time()}.")
    else:
        print("[INFO] No path found in this iteration.")
    apollo_listener.stop()
    print("[INFO] Exiting main function.")

    # Wait before the next update
    # time.sleep(1)

    # except KeyboardInterrupt:
    #     print("[INFO] Stopping main loop due to user interruption.")
    #     sys.exit(0)
    #
    # except Exception as e:
    #     print(f"[ERROR] Exception occurred during main loop: {e}")
    #     traceback.print_exc()
    #     sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Exiting...")
        sys.exit(0)
