import collections
import math
import random
import time
from typing import List, Dict, Set, Tuple

import carla


def _calculate_relative_position_and_distance(ego_vehicle: carla.Vehicle, other_vehicle: carla.Vehicle):
    """
    Calculates the relative position and distance between the ego vehicle and another vehicle.

    :param ego_vehicle: Carla ego vehicle actor.
    :param other_vehicle: Carla other vehicle actor.
    :return: A tuple containing the relative position vector, the distance, and the ego vehicle's forward vector.
    """
    ego_transform = ego_vehicle.get_transform()
    other_transform = other_vehicle.get_transform()

    # Calculate the relative position vector
    ego_location = ego_transform.location
    other_location = other_transform.location
    relative_position = other_location - ego_location

    # Compute the distance between vehicles
    distance = math.sqrt(relative_position.x ** 2 + relative_position.y ** 2)

    # Get the forward vector of the ego vehicle
    forward_vector = ego_transform.get_forward_vector()

    return relative_position, distance, forward_vector


def is_vehicle_in_front(ego_vehicle, other_vehicle, distance_threshold=30.0) -> bool:
    """
    Determines if another vehicle is in front of the ego vehicle.

    :param ego_vehicle: Carla ego vehicle actor.
    :param other_vehicle: Carla other vehicle actor.
    :param distance_threshold: Max distance to consider the vehicle as 'in front'.
    :return: True if the other vehicle is in front, False otherwise.
    """
    relative_position, distance, forward_vector = _calculate_relative_position_and_distance(
        ego_vehicle, other_vehicle)

    # Compute the dot product of relative position and ego vehicle's forward vector
    dot_product = forward_vector.x * relative_position.x + \
        forward_vector.y * relative_position.y

    # Check if the other vehicle is within the distance threshold and in front
    return dot_product > 0 and distance <= distance_threshold


def is_vehicle_around(ego_vehicle, other_vehicle, distance_threshold=5.0) -> bool:
    """
    Determines if another vehicle is around the ego vehicle.

    :param ego_vehicle: Carla ego vehicle actor.
    :param other_vehicle: Carla other vehicle actor.
    :param distance_threshold: Max distance to consider the vehicle as 'around'.
    :return: True if the other vehicle is around, False otherwise.
    """
    _, distance, _ = _calculate_relative_position_and_distance(
        ego_vehicle, other_vehicle)

    # Check if the other vehicle is within the distance threshold
    return distance <= distance_threshold


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
                (vertex1.x - vertex2.x) ** 2
                + (vertex1.y - vertex2.y) ** 2
                + (vertex1.z - vertex2.z) ** 2
            )
            min_distance = min(min_distance, distance)

    return min_distance


def get_block_head_vehicle(ego_vehicle: carla.Vehicle,
                           block_vehicles: List[carla.Vehicle],
                           max_iterations: int = 10) -> carla.Vehicle:
    """
    Finds the head vehicle that is blocking the ego vehicle in a queue of block vehicles.

    :param ego_vehicle: The ego vehicle actor.
    :param block_vehicles: List of blocking vehicles to evaluate (can include None).
    :param max_iterations: Maximum number of iterations to prevent infinite loop (default: 10).
    :return: The head vehicle blocking the ego vehicle, or None if no such vehicle is found.
    """
    # Filter out None objects from block_vehicles
    block_vehicles = [
        vehicle for vehicle in block_vehicles if vehicle is not None]

    # Initialize the current vehicle as the ego vehicle
    current_vehicle = ego_vehicle

    for _ in range(max_iterations):  # Limit the number of iterations
        # Filter the vehicles that are in front of the current vehicle
        front_vehicles = [
            vehicle for vehicle in block_vehicles
            if vehicle.id != current_vehicle.id and is_vehicle_in_front(current_vehicle, vehicle)
        ]

        if not front_vehicles:
            # If no vehicles are found in front, the current vehicle is the head
            return current_vehicle if current_vehicle != ego_vehicle else None

        # Find the closest vehicle in front of the current vehicle
        current_vehicle = min(
            front_vehicles,
            key=lambda v: calculate_distance(current_vehicle, v)
        )

    # If the maximum number of iterations is reached, return None
    print("[WARNING] Maximum iterations reached. Returning None.")
    return None


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
    target_vehicles = [
        vehicle for vehicle in vehicles if condition_func(vehicle)]

    if not target_vehicles:
        return

    # Randomly choose one vehicle from the filtered list
    vehicle_to_resolve = random.choice(target_vehicles)

    print(
        f"[ACTION] Resolving stuck vehicle: Vehicle ID {vehicle_to_resolve.id}")
    vehicle_to_resolve.apply_control(
        carla.VehicleControl(throttle=throttle, brake=0.0))

    time.sleep(duration)

    print(
        f"[ACTION] Resetting control for vehicle ID: {vehicle_to_resolve.id}")
    vehicle_to_resolve.apply_control(
        carla.VehicleControl(throttle=0.0, brake=0.0))


class RoadBlockageChecker:
    def __init__(self, carla_map: carla.Map, carla_world: carla.World):
        """
        Initializes the RoadBlockageChecker.
        :param carla_map: CARLA map object
        :param carla_world: CARLA world object
        """
        self.carla_map = carla_map
        self.carla_world = carla_world

    def get_vehicle_lane_occupation(self, vehicle: carla.Actor) -> List[Tuple[int, int]]:
        """
        Calculates the list of (road_id, lane_id) pairs occupied by the vehicle.
        :param vehicle: CARLA vehicle actor
        :return: List of (road_id, lane_id) pairs occupied by the vehicle
        """
        vehicle_bbox = vehicle.bounding_box
        vehicle_transform = vehicle.get_transform()

        # Use bounding box corners to calculate lane occupation
        bbox_vertices = vehicle_bbox.get_world_vertices(vehicle_transform)
        occupied_lanes = set()

        for vertex in bbox_vertices:
            waypoint = self.carla_map.get_waypoint(
                vertex, project_to_road=True, lane_type=carla.LaneType.Driving)
            if waypoint:
                occupied_lanes.add((waypoint.road_id, waypoint.lane_id))

        return list(occupied_lanes)

    def get_lane_clusters(self, waypoint: carla.Waypoint) -> List[Set[int]]:
        """
        Get all lane clusters based on lane change relationships starting from a given waypoint.

        :param waypoint: Starting waypoint for clustering
        :return: List of lane clusters,is_road_blocked where each cluster is a set of lane IDs
        """
        if not waypoint:
            print("[ERROR] Invalid starting waypoint.")
            return []

        road_id = waypoint.road_id  # Restrict clustering to this road
        visited = set()
        clusters = []

        def traverse_cluster(start_waypoint, cluster):
            """
            Recursively traverse connected lanes and add them to the cluster.
            """
            if not start_waypoint:
                return

            lane_key = (start_waypoint.road_id, start_waypoint.lane_id)
            if lane_key in visited or start_waypoint.road_id != road_id:
                return

            visited.add(lane_key)
            cluster.add(lane_key)

            # Expand left and right based on lane change
            if start_waypoint.lane_change in [carla.LaneChange.Left, carla.LaneChange.Both]:
                traverse_cluster(start_waypoint.get_left_lane(), cluster)
            if start_waypoint.lane_change in [carla.LaneChange.Right, carla.LaneChange.Both]:
                traverse_cluster(start_waypoint.get_right_lane(), cluster)

        # Start clustering from the given waypoint
        cluster = set()
        traverse_cluster(waypoint, cluster)
        if cluster:
            clusters.append(cluster)

        return clusters

    def get_vehicle_clusters(self, vehicles: List[carla.Actor], distance_threshold: float) -> List[Set[carla.Actor]]:
        """
        Get all vehicle clusters based on distance relationships.
        A vehicle can appear in multiple clusters if it satisfies the distance threshold with multiple clusters.

        :param vehicles: List of vehicle actors
        :param distance_threshold: Distance threshold for clustering
        :return: List of vehicle clusters, where each cluster is a set of vehicle actors
        """
        clusters = []

        def are_vehicles_close(vehicle1, vehicle2):
            """
            Determine if two vehicles are within the distance threshold.
            """
            return calculate_distance(vehicle1, vehicle2) <= distance_threshold

        for vehicle in vehicles:
            added_to_cluster = False
            for cluster in clusters:
                # Check if this vehicle is close to any vehicle in the existing cluster
                if any(are_vehicles_close(vehicle, cluster_vehicle) for cluster_vehicle in cluster):
                    cluster.add(vehicle)
                    added_to_cluster = True
            if not added_to_cluster:
                # Create a new cluster if the vehicle doesn't belong to any existing cluster
                clusters.append({vehicle})

        return clusters

    def get_all_lane_clusters(self, vehicles: List[carla.Actor]) -> List[Set[int]]:
        """
        Compute all unique lane clusters based on all vehicles' starting positions.

        :param vehicles: List of vehicle actors
        :return: List of unique lane clusters (each cluster is a set of lane IDs)
        """

        def are_clusters_equal(cluster1: Set[int], cluster2: Set[int]) -> bool:
            """
            Compare two clusters (sets) to determine if they contain the same elements.

            :param cluster1: First cluster (set of integers)
            :param cluster2: Second cluster (set of integers)
            :return: True if the clusters are equal, False otherwise
            """
            if len(cluster1) != len(cluster2):
                return False
            for elem in cluster1:
                if elem not in cluster2:
                    return False
            return True

        all_clusters = []

        for vehicle in vehicles:
            waypoint = self.carla_map.get_waypoint(
                vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if waypoint:
                clusters = self.get_lane_clusters(waypoint)
                all_clusters.extend(clusters)

        # Manual removal of duplicates
        unique_clusters = []
        for cluster in all_clusters:
            if not any(are_clusters_equal(cluster, unique) for unique in unique_clusters):
                unique_clusters.append(cluster)

        return unique_clusters

    def is_road_blocked(self, vehicles: List[carla.Actor], distance_threshold: float) -> Dict:
        """
        Determines whether any road is blocked based on overlapping vehicle clusters.

        :param vehicles: List of vehicle actors
        :param distance_threshold: Max distance to consider neighboring vehicles for clustering
        :return: Dictionary containing blockage status and additional details:
                 - blocked (bool): Whether the road is blocked.
                 - blocked_road_id (int): The road ID of the blocked road.
                 - vehicles_on_blocked_road (List[carla.Actor]): Vehicles causing the blockage.
        """
        result = {
            "blocked": False,
            "blocked_road_id": None,
            "vehicles_on_blocked_road": []
        }

        # Group vehicles by road ID
        road_vehicle_map = collections.defaultdict(list)
        for vehicle in vehicles:
            waypoint = self.carla_map.get_waypoint(
                vehicle.get_location(),
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if waypoint:
                road_vehicle_map[waypoint.road_id].append(vehicle)

        # Check each road
        for road_id, road_vehicles in road_vehicle_map.items():
            # Generate lane clusters specific to this road
            lane_clusters_on_road = self.get_all_lane_clusters(road_vehicles)

            # Generate vehicle clusters (vehicles can be in multiple clusters)
            vehicle_clusters = self.get_vehicle_clusters(
                road_vehicles, distance_threshold)

            # Check each lane cluster for blockage
            for lane_cluster in lane_clusters_on_road:
                for vehicle_cluster in vehicle_clusters:
                    # Collect all lanes occupied by vehicles in this cluster
                    vehicle_lanes = set()
                    for vehicle in vehicle_cluster:
                        vehicle_lanes.update(
                            self.get_vehicle_lane_occupation(vehicle))

                    # Check if the vehicle cluster fully blocks the lane cluster
                    if lane_cluster.issubset(vehicle_lanes):
                        result["blocked"] = True
                        result["blocked_road_id"] = road_id
                        result["vehicles_on_blocked_road"] = list(
                            vehicle_cluster)
                        return result

        return result

    def solve_blockage(self, slow_vehicles, ego_vehicle: carla.Vehicle, throttle: float = 0.5, duration: float = 3.0):
        """
        Solves blockage by applying throttle to vehicles in front of the ego vehicle.
        """

        def condition(vehicle):
            # The Vehicle is in front of ego and not speeding up
            return is_vehicle_in_front(ego_vehicle, vehicle) and vehicle.get_velocity().length() < 1.0

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
            print(
                f"[INFO] Ego vehicle (Lincoln MKZ) found with ID: {ego_vehicle.id}")
            break

    if not ego_vehicle:
        print("[ERROR] No vehicle.lincoln.mkz_2017 found as ego vehicle.")
        exit(1)

    # Initialize the RoadBlockageChecker
    checker = RoadBlockageChecker(carla_map, world)

    distance_threshold = 2.5  # Define the distance threshold for neighboring vehicles
    try:
        while True:
            # Retrieve background vehicles with speed < 0.5
            slow_vehicles = [
                actor for actor in world.get_actors()
                if "vehicle" in actor.type_id
                   and actor.get_velocity().length() < 0.5
            ]
            if ego_vehicle in slow_vehicles:
                slow_vehicles.remove(ego_vehicle)
            # Solve blockage in front of ego vehicle
            print("slow_vehicles:")
            for slow_vehicle in slow_vehicles:
                print(slow_vehicle.id)
            print("[INFO] Checking for road blockage...")
            blockage_result = checker.is_road_blocked(
                slow_vehicles, distance_threshold=distance_threshold)
            if blockage_result:
                checker.solve_blockage(
                    slow_vehicles, ego_vehicle, throttle=1.0, duration=3.0)

            # # Resolve vehicles stuck at intersections
            # resolve_intersection_stuck(slow_vehicles, throttle=0.5, duration=3.0)

            # Wait for 1 second before the next check
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] Stopping the road blockage checker.")
