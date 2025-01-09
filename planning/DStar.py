"""
RTAAstar 2D (Real-time Adaptive A*)
Modified for Carla Simulator Integration
"""

import math
import heapq
import pdb
import signal
import sys
import time
import traceback

import matplotlib.pyplot as plt
from Env import Env
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


def save_grid(lane_grid, obstacles, ego_grid, target_grid, path, resolution, safe_grid=None,
              filename="grid_visualization.png"):
    """
    Save grid visualization to a file with the planned path.

    :param lane_grid: Set of lane grid cells
    :param obstacles: Set of obstacle grid cells
    :param ego_grid: Tuple representing the ego vehicle's grid position
    :param target_grid: Tuple representing the target's grid position
    :param path: List of grid cells representing the planned path
    :param resolution: Resolution to adjust the path points
    :param safe_grid: Set of safe grid cells (optional)
    :param filename: Filename to save the visualization
    """
    plt.figure(figsize=(10, 10))
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')

    # Draw lane grids
    for cell in lane_grid:
        plt.plot(cell[0], cell[1], 'g.', markersize=2, label="Lane" if cell == list(lane_grid)[0] else "")

    # Draw safe grids
    if safe_grid:
        for cell in safe_grid:
            plt.plot(cell[0], cell[1], 'c.', markersize=3, label="Safe Grid" if cell == list(safe_grid)[0] else "")
    # Draw obstacle grids
    for cell in obstacles:
        plt.plot(cell[0], cell[1], 'r.', markersize=4, label="Obstacle" if cell == list(obstacles)[0] else "")

    # Draw ego vehicle
    plt.plot(ego_grid[0], ego_grid[1], 'bo', markersize=10, label="Ego Vehicle")

    # Draw target
    plt.plot(target_grid[0], target_grid[1], 'yo', markersize=10, label="Target")

    # Draw the planned path
    if path:
        path_x, path_y = zip(*[(p[0] / resolution, p[1] / resolution) for p in path])
        plt.plot(path_x, path_y, 'b-', linewidth=2, label="Planned Path")

    # Add legend and save the file
    plt.legend()
    plt.savefig(filename)
    plt.close()


def main():
    from cyber.python.cyber_py3 import cyber
    from loguru import logger
    import carla

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
    resolution = 1
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
        apollo_listener.stop()
        sys.exit(0)

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    print("Waiting for routing response...")
    while not apollo_listener.routing_wps:
        time.sleep(0.5)

    print("Routing response received. Starting visualization loop...")

    try:
        while True:
            # Check if ego vehicle still exists
            if ego_vehicle is None or ego_vehicle not in world.get_actors():
                print("[WARNING] Ego vehicle is missing. Attempting to reacquire...")
                ego_vehicle = None
                for vehicle in world.get_actors().filter('vehicle.*'):
                    if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
                        ego_vehicle = vehicle
                        print("[INFO] Ego vehicle reacquired.")
                        break
                if ego_vehicle is None:
                    print("[ERROR] Ego vehicle could not be reacquired. Exiting loop.")
                    break

            # Get Ego vehicle location
            ego_location = ego_vehicle.get_location()
            ego_grid = (int(ego_location.x / resolution), int(ego_location.y / resolution))

            # Update obstacles
            obstacles = set()  # Ensure obstacles are a set for quick lookup
            for actor in world.get_actors():
                if "vehicle" in actor.type_id and actor.id != ego_vehicle.id:
                    obstacles.add(actor)

            # Update routing waypoints
            routing_waypoints = apollo_listener.routing_wps
            if routing_waypoints:
                waypoints = [wp[0] for wp in routing_waypoints if wp[0]]
                target_waypoint = routing_waypoints[-1][-1]
                target_location = target_waypoint.transform.location
                target_grid = (int(target_location.x / resolution), int(target_location.y / resolution))
                # Calculate bounds to cover all waypoints
                x_min = min(wp.transform.location.x for wp in waypoints)
                y_min = min(wp.transform.location.y for wp in waypoints)
                x_max = max(wp.transform.location.x for wp in waypoints)
                y_max = max(wp.transform.location.y for wp in waypoints)

                # Add a buffer to the bounds for safety
                buffer = 10  # Adjust buffer size as needed
                x_min -= buffer
                y_min -= buffer
                x_max += buffer
                y_max += buffer
            else:
                print("[ERROR] No valid routing waypoints available. Exiting loop.")
                break

            # Set bounds around ego vehicle
            bounds = (x_min, y_min, x_max, y_max)

            # Initialize environment with updated obstacles
            env = Env(world, bounds, obstacles, carla_map, waypoints, target_waypoint, resolution=resolution,
                      safety_distance=1.0)

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
            save_grid(env.lane_grid, env.obs, ego_grid, target_grid, planned_path, resolution=resolution,
                      safe_grid=env.safe_grid, filename=filename)
            print(f"[INFO] Grid visualization saved to {filename}")

            if is_path_found:
                print(f"[INFO] Path found at iteration {time.time()}.")
            else:
                print("[INFO] No path found in this iteration.")

            # Wait before the next update
            time.sleep(1)

    except KeyboardInterrupt:
        print("[INFO] Stopping main loop due to user interruption.")
        sys.exit(0)

    except Exception as e:
        print(f"[ERROR] Exception occurred during main loop: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Exiting...")
        sys.exit(0)
