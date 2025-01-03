"""
RTAAstar 2D (Real-time Adaptive A*)
Modified for Carla Simulator Integration
"""

import math
import heapq
from planing.Env import Env


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
    def __init__(self, s_start, s_goal, N, heuristic_type, env_params):
        """
        Initialize RTAAStar planner.

        :param s_start: Start position (grid coordinate tuple)
        :param s_goal: Goal position (grid coordinate tuple)
        :param N: Number of nodes to expand per iteration
        :param heuristic_type: Type of heuristic ('manhattan' or 'euclidean')
        :param env_params: Dictionary containing parameters to initialize Env
        """
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        # Initialize the Env object with provided parameters
        self.Env = Env(
            world=env_params['world'],
            bounds=env_params['bounds'],
            obstacles=env_params['obstacles'],
            map=env_params['map'],
            waypoints=env_params['waypoints'],
            resolution=env_params.get('resolution', 0.5),  # Default resolution
            safety_distance=env_params.get('safety_distance', 0.5)  # Default safety distance
        )

        # Define possible motions (8-connected directions for more flexibility)
        self.u_set = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        self.bounds = env_params['bounds']  # Save bounds for grid range
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
    def init(self):
        """
        Initialize the heuristic table based on the grid bounds and the heuristic type.
        """
        x_min, y_min, x_max, y_max = self.bounds
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                self.h_table[(x, y)] = self.h((x, y))

    def searching(self, max_iterations=1000):
        """
        Perform RTAA* search with a termination condition.

        :param max_iterations: Maximum number of iterations to prevent infinite loops.
        :return: Boolean indicating whether a valid path was found.
        """
        self.init()
        s_start = self.s_start  # initialize start node
        iteration = 0

        while iteration < max_iterations:
            iteration += 1
            OPEN, CLOSED, g_table, PARENT = self.Astar(s_start, self.N)

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)
                return True

            if OPEN.empty():  # No nodes left to explore
                print("Path not found. Search terminated.")
                return False

            s_next, h_value = self.cal_h_value(OPEN, CLOSED, g_table, PARENT)

            for x in h_value:
                self.h_table[x] = h_value[x]

            s_start, path_k = self.extract_path_in_CLOSE(s_start, s_next, h_value)
            self.path.append(path_k)

        print("Max iterations reached. Path not found.")
        return False

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
        for (_, x) in OPEN.enumerate():
            v_open[x] = g_table[PARENT[x]] + 1 + self.h_table[x]
        s_open = min(v_open, key=v_open.get)
        f_min = v_open[s_open]
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

        :param s: Current node (grid coordinate tuple)
        :return: Heuristic cost
        """
        goal = self.s_goal
        if self.heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

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
