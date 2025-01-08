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


class QueuePrior:
    """
    Class: QueuePrior
    Description: QueuePrior reorders elements using value [priority]
    """

    def __init__(self):
        self.queue = []

    def empty(self):
        return len(self.queue) == 0

    def put(self, item, priority):
        heapq.heappush(self.queue, (priority, item))  # reorder s using priority

    def get(self):
        return heapq.heappop(self.queue)[1]  # pop out the smallest item

    def enumerate(self):
        return self.queue


class RTAAStar:
    def __init__(self, s_start, s_goal, N, heuristic_type, env):
        """
        Initialize RTAAStar planner.

        :param s_start: Start position (grid coordinate tuple)
        :param s_goal: Goal position (grid coordinate tuple)
        :param N: Number of nodes to expand per iteration
        :param heuristic_type: Type of heuristic ('manhattan' or 'euclidean')
        :param env: Env object containing the grid environment
        """

        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        # Initialize the Env object with provided parameters
        self.Env = env
        self.resolution = self.Env.resolution

        # Define possible motions (8-connected directions for more flexibility)
        self.u_set = [
            (-self.resolution, 0), (self.resolution, 0),
            (0, -self.resolution), (0, self.resolution),
            (-self.resolution, -self.resolution), (-self.resolution, self.resolution),
            (self.resolution, -self.resolution), (self.resolution, self.resolution)
        ]

        self.bounds = self.Env.bounds  # Bounds of the grid environment
        self.N = N  # Number of nodes to expand per iteration
        self.visited = []  # Order of visited nodes in planning
        self.path = []  # Path of each iteration
        self.h_table = {}  # Heuristic value table

    def extract_path(self, x_start, parent):
        path = [self.s_goal]
        s = self.s_goal

        while s != x_start:
            s = parent[s]
            path.append(s)

        return list(reversed(path))

    def extract_path_in_CLOSE(self, s_end, s_start, h_value):
        path = [s_start]
        s = s_start

        while True:
            h_list = {}
            for s_n in self.get_neighbor(s):
                if s_n in h_value:
                    h_list[s_n] = h_value[s_n]
            s_key = max(h_list, key=h_list.get)  # move to the smallest node with min h_value
            path.append(s_key)  # generate path
            s = s_key  # use end of this iteration as the start of next

            if s_key == s_end:  # reach the expected node in OPEN set
                return s_start, list(reversed(path))

    def init(self):
        """
        Initialize the heuristic table based on the grid bounds and the heuristic type.
        """
        x_min, y_min, x_max, y_max = self.bounds
        step = self.resolution  # ???????
        for x in range(int(x_min / step), int(x_max / step) + 1):
            for y in range(int(y_min / step), int(y_max / step) + 1):
                grid_x, grid_y = x * step, y * step
                self.h_table[(grid_x, grid_y)] = self.h((grid_x, grid_y))

    def searching(self):
        self.init()
        s_start = self.s_start  # initialize start node

        while True:
            OPEN, CLOSED, g_table, PARENT = self.Astar(s_start, self.N)

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)
                break

            s_next, h_value = self.cal_h_value(OPEN, CLOSED, g_table, PARENT)

            # Check if s_next is None
            if s_next is None:
                print("[ERROR] No valid next node found. Terminating search.")
                break  # Exit the loop

            # Update heuristic table
            for x in h_value:
                self.h_table[x] = h_value[x]

            # Extract path and update start node
            s_start, path_k = self.extract_path_in_CLOSE(s_start, s_next, h_value)
            self.path.append(path_k)

    def cal_h_value(self, OPEN, CLOSED, g_table, PARENT):
        """
        Calculate updated heuristic values based on the expanded nodes.

        :param OPEN: Open list
        :param CLOSED: Closed list
        :param g_table: Cost table
        :param PARENT: Parent map
        :return: Next node and updated heuristic values
        """
        v_open = {}
        h_value = {}

        # Construct v_open
        for (_, x) in OPEN.enumerate():
            if x in PARENT and x in g_table and x in self.h_table:
                v_open[x] = g_table[PARENT[x]] + 1 + self.h_table[x]

        # Check if v_open is empty
        if not v_open:
            print("[WARNING] v_open is empty. Returning default values.")
            return None, h_value  # Return None to indicate failure

        # Find node with minimum heuristic value
        s_open = min(v_open, key=v_open.get)
        f_min = v_open[s_open]

        # Update heuristic values for CLOSED nodes
        for x in CLOSED:
            h_value[x] = f_min - g_table[x]

        return s_open, h_value

    def Astar(self, x_start, N):
        """
        Perform A* search for a limited number of nodes.

        :param x_start: Start node
        :param N: Maximum number of nodes to expand
        :return: Open list, closed list, cost table, and parent map
        """
        OPEN = QueuePrior()
        OPEN.put(x_start, self.h_table[x_start])
        CLOSED = []
        g_table = {x_start: 0}
        PARENT = {x_start: x_start}
        count = 0

        while not OPEN.empty():
            count += 1
            s = OPEN.get()
            CLOSED.append(s)

            if s == self.s_goal:
                self.visited.append(CLOSED)
                return "FOUND", self.extract_path(x_start, PARENT), [], []

            for s_n in self.get_neighbor(s):
                if s_n not in CLOSED:
                    new_cost = g_table[s] + self.cost(s, s_n)
                    if s_n not in g_table:
                        g_table[s_n] = float("inf")
                    if new_cost < g_table[s_n]:
                        g_table[s_n] = new_cost
                        PARENT[s_n] = s
                        OPEN.put(s_n, g_table[s_n] + self.h_table[s_n])

            if count == N:  # Expand needed CLOSED nodes
                break

        self.visited.append(CLOSED)
        return OPEN, CLOSED, g_table, PARENT

    def get_neighbor(self, s):
        """
        Get all feasible neighbors of the current node.

        :param s: Current node (grid coordinate tuple)
        :return: Set of feasible neighbor nodes
        """
        s_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if not self.Env.is_occupied(s_next[0], s_next[1]):
                s_list.add(s_next)
        return s_list

    def h(self, s):
        """
        Heuristic function to estimate the cost from a node to the goal.
        """
        goal = self.s_goal
        if self.heuristic_type == "manhattan":
            return (abs(goal[0] - s[0]) + abs(goal[1] - s[1])) * self.resolution
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1]) * self.resolution

    def cost(self, s_start, s_goal):
        """
        Calculate the cost of moving from one node to another.

        :param s_start: Start node
        :param s_goal: Goal node
        :return: Cost
        """
        if self.is_collision(s_start, s_goal):
            return float("inf")
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        Check if moving from s_start to s_end results in a collision.

        :param s_start: Start node
        :param s_end: End node
        :return: True if collision, False otherwise
        """
        if self.Env.is_occupied(s_start[0], s_start[1]) or self.Env.is_occupied(s_end[0], s_end[1]):
            return True
        return False


def save_grid(lane_grid, obstacles, ego_grid, target_grid, path, filename="grid_visualization.png"):
    """
    Save grid visualization to a file with the planned path.

    :param lane_grid: Set of lane grid cells
    :param obstacles: Set of obstacle grid cells
    :param ego_grid: Tuple representing the ego vehicle's grid position
    :param target_grid: Tuple representing the target's grid position
    :param path: List of grid cells representing the planned path
    :param filename: Filename to save the visualization
    """
    plt.figure(figsize=(10, 10))
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')

    # Draw lane grids
    for cell in lane_grid:
        plt.plot(cell[0], cell[1], 'g.', markersize=2, label="Lane" if cell == list(lane_grid)[0] else "")

    # Draw obstacle grids
    for cell in obstacles:
        plt.plot(cell[0], cell[1], 'r.', markersize=4, label="Obstacle" if cell == list(obstacles)[0] else "")

    # Draw ego vehicle
    plt.plot(ego_grid[0], ego_grid[1], 'bo', markersize=10, label="Ego Vehicle")

    # Draw target
    plt.plot(target_grid[0], target_grid[1], 'yo', markersize=10, label="Target")

    # Draw the planned path
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label="Planned Path")

    plt.legend()
    plt.savefig(filename)
    plt.close()


def main():
    import carla
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
            ego_grid = (int(ego_location.x), int(ego_location.y))

            # Update obstacles
            obstacles = [
                actor for actor in world.get_actors()
                if "vehicle" in actor.type_id and actor.id != ego_vehicle.id
            ]

            # Update routing waypoints
            routing_waypoints = apollo_listener.routing_wps
            if routing_waypoints:
                waypoints = [wp[0] for wp in routing_waypoints if wp[0]]
                target_waypoint = routing_waypoints[-1][-1]
                target_location = target_waypoint.transform.location
                target_grid = (int(target_location.x), int(target_location.y))
            else:
                print("[ERROR] No valid routing waypoints available. Exiting loop.")
                break

            # Set bounds around ego vehicle
            x_min, y_min, x_max, y_max = (
                ego_location.x - 50,
                ego_location.y - 50,
                ego_location.x + 50,
                ego_location.y + 50
            )
            bounds = (x_min, y_min, x_max, y_max)
            # Initialize the environment
            env = Env(world, bounds, obstacles, carla_map, waypoints, target_waypoint, resolution=0.5,
                      safety_distance=1.0)

            # Initialize RTAAStar
            planner = RTAAStar(s_start=ego_grid, s_goal=target_grid, N=10, heuristic_type="manhattan", env=env)

            # Perform planning
            is_path_found = planner.searching()

            # Save grid visualization
            planned_path = planner.path[-1] if planner.path else []  # Use the last planned path
            filename = f"grid_visualization_{int(time.time())}.png"
            save_grid(env.lane_grid, env.obs, ego_grid, target_grid, planned_path, filename=filename)
            print(f"[INFO] Grid visualization saved to {filename}")

            if is_path_found:
                print(f"[INFO] Path found at iteration {time.time()}.")

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
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Exiting...")
        sys.exit(0)
