import collections
import sys
import time


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
from typing import List, Dict


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
        :return: Distance from the start of the lane in meters
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

        print(f"[DEBUG] Extracted lane IDs for road {road_id}: {sorted(lane_ids)}")
        return sorted(lane_ids)

    def is_road_blocked(self, vehicles: List[carla.Actor], distance_threshold: float) -> Dict:
        """
        Determines whether the road is blocked based on the vehicles' positions and lane coverage.
        Each road is independently checked.

        :param vehicles: List of vehicle actors to consider for blockage checking.
        :param distance_threshold: Threshold distance for neighboring vehicles.
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
        for road_id, vehicles in road_vehicle_map.items():

            # Dynamically fetch all lane IDs for this road
            all_lane_ids = self.get_all_lane_ids(vehicles[0])  # Pass any vehicle on this road

            # Gather data for each vehicle: occupied lanes and distance from lane start
            vehicle_data = [
                {
                    "vehicle": vehicle,
                    "occupied_lanes": self.get_vehicle_lane_occupation(vehicle),
                    "distance": self.compute_distance_from_lane_start(vehicle)
                }
                for vehicle in vehicles
            ]

            # Sort vehicles by their distance from the lane start
            vehicle_data.sort(key=lambda x: x["distance"])

            # Traverse vehicles and check blockage conditions
            for i, vehicle_info in enumerate(vehicle_data):

                current_lanes = set(vehicle_info["occupied_lanes"])
                for other_vehicle_info in vehicle_data:
                    # Skip vehicles outside the threshold distance
                    if abs(other_vehicle_info["distance"] - vehicle_info["distance"]) > distance_threshold:
                        continue
                    # Merge occupied lanes from neighboring vehicles
                    current_lanes.update(other_vehicle_info["occupied_lanes"])

                # Check if all lanes are covered
                if set(all_lane_ids).issubset(current_lanes):
                    result["blocked"] = True
                    result["blocked_road_id"] = road_id
                    result["vehicles_on_blocked_road"] = [v["vehicle"] for v in vehicle_data]
                    return result

        return result


if __name__ == '__main__':
    import time

    # Example usage
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # Initialize the RoadBlockageChecker
    checker = RoadBlockageChecker(carla_map, world)

    distance_threshold = 5.0  # Define the distance threshold for neighboring vehicles

    try:
        while True:
            # Retrieve background vehicles with speed < 0.5
            slow_vehicles = [
                actor for actor in world.get_actors()
                if "vehicle" in actor.type_id and actor.get_velocity().length() < 0.5
            ]
            print(f"[INFO] Found {len(slow_vehicles)} vehicles with speed < 0.5.")

            # Check if the road is blocked using slow vehicles
            result = checker.is_road_blocked(slow_vehicles, distance_threshold)
            if result["blocked"]:
                print(f"[RESULT] Road is blocked on road ID {result['blocked_road_id']} "
                      f"with {len(result['vehicles_on_blocked_road'])} vehicles.")
            else:
                print("[RESULT] No roads are blocked.")

            # Wait for 1 second before the next check
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] Stopping the road blockage checker.")

