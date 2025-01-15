import carla
from planning.Env import Env
from planning.DStar import save_grid_plotly, DStar
from ms_utils.apollo_routing_listener import ApolloRoutingListener
import os
import signal
import sys
import time
import traceback

parent_directory = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_directory)


class VehicleNavigation:
    def __init__(self, world,ego_vehicle, apollo_routing_wps, carla_map):
        self.world = world
        self.apollo_routing_wps = apollo_routing_wps
        self.carla_map = carla_map
        self.resolution = 0.5
        self.ego_vehicle = ego_vehicle
        self.bounds = None
        self.env = None
        self.planner = None

    def reacquire_ego_vehicle(self):
        """Reacquire the ego vehicle if missing."""
        if self.ego_vehicle is None:
            print("[WARNING] Ego vehicle is missing. Attempting to reacquire...")
            self.ego_vehicle = None
            for vehicle in self.world.get_actors().filter('vehicle.*'):
                if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
                    self.ego_vehicle = vehicle
                    print("[INFO] Ego vehicle reacquired.")
                    break
            if self.ego_vehicle is None:
                print("[ERROR] Ego vehicle could not be reacquired. Exiting loop.")
                return False
        return True

    def update_obstacles(self):
        """Update obstacles in the world."""
        obstacles = set()  # Ensure obstacles are a set for quick lookup
        obstacle_grids = set()  # To store obstacle grid coordinates
        for actor in self.world.get_actors():
            if "vehicle" in actor.type_id and actor.id != self.ego_vehicle.id:
                obstacles.add(actor)
                obstacle_location = actor.get_location()
                obstacle_grid = self.to_grid(obstacle_location)
                obstacle_grids.add(obstacle_grid)
        return obstacles, obstacle_grids

    def update_routing_waypoints(self):
        """Update the routing waypoints and calculate bounds."""
        routing_waypoints = self.apollo_routing_wps
        if not routing_waypoints:
            print("[ERROR] No valid routing waypoints available. Exiting loop.")
            return False

        # Extract Waypoints from routing_waypoints (assuming each wp is [Waypoint, OtherData])
        # wp[0] should be the Waypoint object
        waypoints = [wp[0] for wp in routing_waypoints if wp[0]]

        # Correctly extract the target waypoint (the last waypoint)
        # Get the first element from the last waypoint list
        target_waypoint = routing_waypoints[-1][0]
        target_location = target_waypoint.transform.location
        target_grid = self.to_grid(target_location)

        # Calculate bounds
        grid_waypoints = [self.to_grid(wp.transform.location)
                          for wp in waypoints]

        # Update obstacles
        obstacle_grids = self.update_obstacles()[1]
        all_grids = grid_waypoints + list(obstacle_grids) + [target_grid]

        # Calculate the bounding box for all grids
        x_min = min(wp[0] for wp in all_grids)
        y_min = min(wp[1] for wp in all_grids)
        x_max = max(wp[0] for wp in all_grids)
        y_max = max(wp[1] for wp in all_grids)

        # Add a buffer for safety
        buffer = int(100 / self.resolution)
        x_min -= buffer
        y_min -= buffer
        x_max += buffer
        y_max += buffer

        # Store the calculated bounds
        self.bounds = (x_min, y_min, x_max, y_max)

        return True

    def initialize_environment(self):
        """Initialize the environment for the planner."""
        obstacles = self.update_obstacles()[0]
        waypoints = [wp[0] for wp in self.apollo_routing_wps if wp[0]]
        self.env = Env(self.world, self.bounds, obstacles, self.carla_map,
                       waypoints, self.resolution, safety_distance=1.5)

    def initialize_planner(self):
        """Initialize the DStar planner."""
        ego_location = self.ego_vehicle.get_location()
        ego_grid = (int(ego_location.x / self.resolution),
                    int(ego_location.y / self.resolution))
        target_location = self.apollo_routing_wps[-1][-1].transform.location
        target_grid = self.to_grid(target_location)

        self.planner = DStar(
            s_start=ego_grid, s_goal=target_grid, env=self.env)
        self.planner.init()

    def perform_planning(self):
        """Perform the pathfinding and return if a path was found."""
        return self.planner.run()

    def get_navigation_path(self):
        """Returns the planned navigation path."""
        planned_path = self.planner.path
        carla_navigation_path = []
        for grid_point in planned_path:
            world_x = grid_point[0] * self.resolution
            world_y = grid_point[1] * self.resolution
            waypoint = self.carla_map.get_waypoint(
                carla.Location(x=world_x, y=world_y, z=0.0))
            if waypoint:
                carla_navigation_path.append(waypoint)
            else:
                print(
                    f"[WARNING] No valid waypoint found for grid point {grid_point}")
        return carla_navigation_path

    def save_visualization(self, planned_path, filename):
        """Save the grid visualization."""
        safe_grid = self.env.safe_grid_lane.union(self.env.safe_grid_obs)
        # save_grid(self.ego_vehicle, self.env.lane_grid, self.env.obs,
        #           self.to_grid(
        #               self.apollo_routing_wps[-1][-1].transform.location),
        #           planned_path, self.carla_map, self.bounds, self.resolution, safe_grid, filename=filename)
        save_grid_plotly(self.carla_map,self.ego_vehicle, self.env.lane_grid, self.env.obs,
                         self.to_grid(
                             self.apollo_routing_wps[-1][-1].transform.location),
                         planned_path, self.bounds, self.resolution, safe_grid, filename=filename)
        # print(f"[INFO] Grid visualization saved to {filename}")

    def to_grid(self, location):
        """Convert a location to grid coordinates."""
        return int(location.x / self.resolution), int(location.y / self.resolution)

    def run(self):
        """Main function to run the vehicle navigation process."""
        if not self.reacquire_ego_vehicle():
            return
        if not self.update_routing_waypoints():
            return
        self.initialize_environment()
        self.initialize_planner()


def handle_exit_signal(signal, frame):
    print("\n[INFO] Exiting program due to signal interrupt (Ctrl+C)...")
    sys.exit(0)

def main():
    from cyber.python.cyber_py3 import cyber
    # Initialize Carla client and world
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # Initialize Apollo Cyber RT (if necessary)
    cyber.init()

    # Find Ego vehicle
    max_retries = 5
    retry_interval = 2  # seconds
    ego_vehicle = None
    for attempt in range(max_retries):
        print(
            f"[INFO] Attempting to find ego vehicle (Attempt {attempt + 1}/{max_retries})...")
        for vehicle in world.get_actors().filter('vehicle.*'):
            if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
                ego_vehicle = vehicle
                print("[INFO] Ego vehicle found.")
                break
        if ego_vehicle:
            break
        else:
            print(
                f"[WARNING] Ego vehicle not found. Retrying in {retry_interval} seconds...")
            time.sleep(retry_interval)

    if not ego_vehicle:
        print("[ERROR] Failed to find vehicle.lincoln.mkz_2017 after multiple attempts.")
        return

    # Initialize Apollo Routing Listener
    apollo_listener = ApolloRoutingListener(
        carla_world=world, ego_vehicle=ego_vehicle, debug=True)
    apollo_listener.start("routing_test_node")
    print("Waiting for routing response...")
    while not apollo_listener.routing_wps:
        time.sleep(0.5)
    print("Routing response received. Starting visualization loop...")

    # Create an instance of VehicleNavigation
    vehicle_navigation = VehicleNavigation(world, ego_vehicle,apollo_listener.routing_wps, carla_map)

    try:
        # Plan a path using VehicleNavigation
        vehicle_navigation.run()
        # waypoints in planned_path
        planned_path = vehicle_navigation.perform_planning()
        # save the pic in grid_visualization_{int(time.time())}.png
        vehicle_navigation.save_visualization(
            planned_path, f"grid_visualization_{int(time.time())}.png")

        # Test if the planned path was successfully generated
        if planned_path:
            print(
                f"[INFO] Path successfully generated with {len(planned_path)} waypoints.")
        else:
            print("[ERROR] Failed to generate a valid path.")

    except KeyboardInterrupt:
        print("[INFO] Stopping main loop due to user interruption.")
        sys.exit(0)
    except Exception as e:
        print(f"[ERROR] Exception occurred during main loop: {e}")
        traceback.print_exc()
        sys.exit(1)
    apollo_listener.stop()

if __name__ == "__main__":
    # Register signal handler for graceful shutdown (Ctrl+C)
    signal.signal(signal.SIGINT, handle_exit_signal)
    main()
