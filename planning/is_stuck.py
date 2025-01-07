import collections
import math
import random
import sys
import time
from typing import List, Dict


def set_carla_api_path():
    # print('carla 0914 neednot be installed in this version ')
    # return
    try:
        api_path = "../PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg"
    except IndexError:
        print("Couldn't set Carla API path.")
        exit(-1)

    if api_path not in sys.path:
        sys.path.append(api_path)
        print(f"API: {api_path}")


set_carla_api_path()
import carla


def is_vehicle_in_front(ego_vehicle, other_vehicle, distance_threshold=30.0) -> bool:
    """
    Determines if another vehicle is in front of the ego vehicle.

    :param ego_vehicle: Carla ego vehicle actor.
    :param other_vehicle: Carla other vehicle actor.
    :param distance_threshold: Max distance to consider the vehicle as 'in front'.
    :return: True if the other vehicle is in front, False otherwise.
    """
    ego_transform = ego_vehicle.get_transform()
    other_transform = other_vehicle.get_transform()

    # Calculate the relative position vector
    ego_location = ego_transform.location
    other_location = other_transform.location
    relative_position = other_location - ego_location

    # Compute the dot product of relative position and ego vehicle's forward vector
    forward_vector = ego_transform.get_forward_vector()
    dot_product = forward_vector.x * relative_position.x + forward_vector.y * relative_position.y

    # Check if the other vehicle is within the distance threshold and in front
    distance = math.sqrt(relative_position.x ** 2 + relative_position.y ** 2)
    return dot_product > 0 and distance <= distance_threshold


# Function to calculate distance between two vehicles
def calculate_distance(v1, v2):
    """
    Calculates the shortest distance between the bounding boxes of two vehicles.

    :param v1: The first vehicle actor.
    :param v2: The second vehicle actor.
    :return: The shortest distance between the bounding boxes of the two vehicles.
    """
    bbox1 = v1.bounding_box
    bbox2 = v2.bounding_box

    # Transform the bounding boxes to world coordinates
    vertices1 = bbox1.get_world_vertices(v1.get_transform())
    vertices2 = bbox2.get_world_vertices(v2.get_transform())

    min_distance = float('inf')

    # Calculate the shortest distance between all pairs of vertices
    for vertex1 in vertices1:
        for vertex2 in vertices2:
            distance = math.sqrt(
                (vertex1.x - vertex2.x) ** 2 +
                (vertex1.y - vertex2.y) ** 2 +
                (vertex1.z - vertex2.z) ** 2
            )
            min_distance = min(min_distance, distance)

    return min_distance


def is_vehicle_accelerating(vehicle: carla.Vehicle) -> bool:
    """
    Checks if the vehicle is currently accelerating.
    """
    control = vehicle.get_control()
    return control.throttle > 0.1 and control.brake == 0.0


def resolve_stuck_vehicles(vehicles: List[carla.Actor], condition_func, throttle: float = 0.5,
                           duration: float = 3.0):
    """
    Abstract function to resolve stuck vehicles based on a given condition.

    :param vehicles: List of vehicle actors to evaluate.
    :param condition_func: A function that takes a vehicle as input and returns True if the vehicle should be resolved.
    :param throttle: Throttle value to apply to stuck vehicles (default: 0.5).
    :param duration: Duration to apply the throttle (in seconds, default: 3.0).
    """
    # Filter vehicles based on the condition
    target_vehicles = [vehicle for vehicle in vehicles if condition_func(vehicle)]

    if not target_vehicles:
        return

    # Randomly choose one vehicle from the filtered list
    vehicle_to_resolve = random.choice(target_vehicles)

    print(f"[ACTION] Resolving stuck vehicle: Vehicle ID {vehicle_to_resolve.id}")
    vehicle_to_resolve.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0))

    time.sleep(duration)

    print(f"[ACTION] Resetting control for vehicle ID: {vehicle_to_resolve.id}")
    vehicle_to_resolve.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))


def resolve_intersection_stuck(vehicles: List[carla.Actor], throttle: float = 0.5, duration: float = 3.0):
    """
    Resolves vehicles stuck at intersections by applying throttle to vehicles
    stopped for too long and with no vehicles in front.
    """

    def condition(vehicle):
        # Vehicle is stopped and no vehicles are in front
        speed = vehicle.get_velocity().length()
        is_in_front = any(is_vehicle_in_front(vehicle, other_vehicle) for other_vehicle in vehicles if
                          other_vehicle.id != vehicle.id)
        return speed < 0.5 and not is_in_front

    resolve_stuck_vehicles(vehicles, condition, throttle, duration)


class RoadBlockageChecker:
    def __init__(self, carla_map: carla.Map, carla_world: carla.World):
        """
        Initializes the RoadBlockageChecker.
        :param carla_map: CARLA map object
        :param carla_world: CARLA world object
        """
        self.carla_map = carla_map
        self.carla_world = carla_world

    def get_vehicle_lane_occupation(self, vehicle: carla.Actor) -> List[int]:
        """
        Calculates the list of lane IDs occupied by the vehicle.
        :param vehicle: CARLA vehicle actor
        :return: List of lane IDs occupied by the vehicle
        """
        vehicle_bbox = vehicle.bounding_box
        vehicle_transform = vehicle.get_transform()

        # Use bounding box corners to calculate lane occupation
        bbox_vertices = vehicle_bbox.get_world_vertices(vehicle_transform)
        occupied_lanes = set()

        for vertex in bbox_vertices:
            waypoint = self.carla_map.get_waypoint(vertex, project_to_road=True, lane_type=carla.LaneType.Driving)
            if waypoint:
                occupied_lanes.add(waypoint.lane_id)

        return list(occupied_lanes)

    def compute_distance_from_lane_start(self, vehicle: carla.Actor) -> float:
        """
        Computes the distance of the vehicle from the start of the lane.
        :param vehicle: CARLA vehicle actor
        :return:  Distanced from the start of the lane in meters
        """
        vehicle_location = vehicle.get_location()
        waypoint = self.carla_map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        return waypoint.s if waypoint else float('inf')  # Return infinity if no valid waypoint is found

    def get_all_lane_ids(self, vehicle: carla.Actor) -> List[int]:
        """
        Dynamically determine all lane IDs for the road where the given vehicle is located.

        :param vehicle: The Carla vehicle actor
        :return: List of lane IDs for the road
        """
        waypoint = self.carla_map.get_waypoint(
            vehicle.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        if not waypoint:
            print("[DEBUG] No valid waypoint found for vehicle. Returning empty lane ID list.")
            return []

        # Retrieve all lanes on the current road
        road_id = waypoint.road_id
        lane_ids = set()
        visited_waypoints = set()

        # Function to traverse in one direction (right or left)
        def traverse_lanes(start_waypoint, direction_func, max_iterations=50):
            """
            Traverse lanes in a specified direction (left or right).

            :param start_waypoint: Starting waypoint for traversal
            :param direction_func: Function to get the next waypoint (get_right_lane or get_left_lane)
            :param max_iterations: Maximum iterations to prevent infinite loops
            """
            current_waypoint = start_waypoint
            iteration_count = 0

            while current_waypoint and current_waypoint.road_id == road_id:
                if current_waypoint.lane_id in lane_ids:
                    break  # Avoid re-visiting lanes
                lane_ids.add(current_waypoint.lane_id)
                visited_waypoints.add(current_waypoint)

                current_waypoint = direction_func()
                iteration_count += 1

                if iteration_count >= max_iterations:
                    print("[WARNING] Reached maximum iterations while traversing lanes.")
                    break

        # Traverse right lanes
        traverse_lanes(waypoint, waypoint.get_right_lane)

        # Traverse left lanes
        traverse_lanes(waypoint.get_left_lane(), lambda: waypoint.get_left_lane())

        return sorted(lane_ids)

    def is_road_blocked(self, vehicles: List[carla.Actor], distance_threshold: float) -> Dict:
        """
        Determines whether the road is blocked based on vehicles' positions using an expanding search.
        Each road is independently checked.

        :param vehicles: List of vehicle actors to consider for blockage checking.
        :param distance_threshold: Max distance to consider neighboring vehicles for expansion.
        :return: Dictionary containing blockage status and additional details.
        """

        result = {
            "blocked": False,
            "blocked_road_id": None,
            "vehicles_on_blocked_road": []
        }

        # Group vehicles by road ID
        road_vehicle_map = collections.defaultdict(list)
        for vehicle in vehicles:
            waypoint = self.carla_map.get_waypoint(vehicle.get_location(), project_to_road=True,
                                                   lane_type=carla.LaneType.Driving)
            if not waypoint:
                continue
            road_vehicle_map[waypoint.road_id].append(vehicle)

        # Check each road
        for road_id, road_vehicles in road_vehicle_map.items():
            # Dynamically fetch all lane IDs for this road
            all_lane_ids = self.get_all_lane_ids(road_vehicles[0])  # Use any vehicle on this road
            visited = set()  # Tracks visited vehicles
            blocks = []  # List of vehicle blocks

            # Perform expansion for each vehicle
            for vehicle in road_vehicles:
                if vehicle.id in visited:
                    continue
                # Start a new block
                block = []
                queue = [vehicle]
                while queue:
                    current_vehicle = queue.pop(0)
                    if current_vehicle.id in visited:
                        continue
                    # Mark as visited and add to current block
                    visited.add(current_vehicle.id)
                    block.append(current_vehicle)
                    # Find neighbors within distance_threshold
                    for other_vehicle in road_vehicles:
                        if other_vehicle.id not in visited and calculate_distance(current_vehicle,
                                                                                  other_vehicle) <= distance_threshold:
                            queue.append(other_vehicle)
                # Print the computed block
                block_ids = [v.id for v in block]
                print(f"[DEBUG] Computed cluster: {block_ids}")
                # Save the completed block
                blocks.append(block)

            # Check if any block covers all lanes
            for block in blocks:
                covered_lanes = set()
                for vehicle in block:
                    covered_lanes.update(self.get_vehicle_lane_occupation(vehicle))
                if set(all_lane_ids).issubset(covered_lanes):
                    result["blocked"] = True
                    result["blocked_road_id"] = road_id
                    result["vehicles_on_blocked_road"] = block
                    return result

        return result

    def solve_blockage(self, slow_vehicles, ego_vehicle: carla.Vehicle, throttle: float = 0.5, duration: float = 3.0):
        """
        Solves blockage by applying throttle to vehicles in front of the ego vehicle.
        """

        def condition(vehicle):
            # The Vehicle is in front of ego and not speeding up
            return is_vehicle_in_front(ego_vehicle,
                                       vehicle) and vehicle.get_velocity().length() < 1.0 and not is_vehicle_accelerating(
                vehicle)

        resolve_stuck_vehicles(slow_vehicles, condition, throttle, duration)


if __name__ == '__main__':

    # Example usage
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # Retrieve the Tesla Model 3 as the ego vehicle
    ego_vehicle = None

    for vehicle in world.get_actors().filter('vehicle.*'):
        print(vehicle.type_id)
        if "vehicle.lincoln.mkz_2017" in vehicle.type_id:
            ego_vehicle = vehicle
            print(f"[INFO] Ego vehicle (Lincoln MKZ) found with ID: {ego_vehicle.id}")
            break

    if not ego_vehicle:
        print("[ERROR] No vehicle.lincoln.mkz_2017 found as ego vehicle.")
        exit(1)

    # Initialize the RoadBlockageChecker
    checker = RoadBlockageChecker(carla_map, world)

    distance_threshold = 5.0  # Define the distance threshold for neighboring vehicles
    try:
        while True:
            # Retrieve background vehicles with speed < 0.5
            slow_vehicles = [
                actor for actor in world.get_actors()
                if "vehicle" in actor.type_id and
                   actor.get_velocity().length() < 0.5
            ]
            if ego_vehicle in slow_vehicles:
                slow_vehicles.remove(ego_vehicle)
            # Solve blockage in front of ego vehicle
            print("slow_vehicles:")
            for slow_vehicle in slow_vehicles:
                print(slow_vehicle.id)
            print("[INFO] Checking for road blockage...")
            blockage_result = checker.is_road_blocked(slow_vehicles, distance_threshold=5.0)
            if blockage_result:
                checker.solve_blockage(slow_vehicles, ego_vehicle, throttle=1.0, duration=3.0)

            # # Resolve vehicles stuck at intersections
            # resolve_intersection_stuck(slow_vehicles, throttle=0.5, duration=3.0)

            # Wait for 1 second before the next check
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] Stopping the road blockage checker.")
