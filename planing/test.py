"""
RTAAstar 2D (Real-time Adaptive A*)
Integrated with Carla Simulator
"""
import glob
import math
import os
import sys

from planing.Env import Env


def set_carla_api_path():
    # print('carla 0914 neednot be installed in this version ')
    # return
    dist_path = "../PythonAPI/carla/dist"
    glob_path = os.path.join(dist_path, "carla-0.9.14-py3.7-linux-x86_64.egg" )
    print(glob_path)
    try:
        api_path = glob.glob(glob_path)[0]
    except IndexError:
        print("Couldn't set Carla API path.")
        exit(-1)

    if api_path not in sys.path:
        sys.path.append(api_path)
        print(f"API: {api_path}")

set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    exit(-1)
import time

from planing.RTAAStar import RTAAStar


def main():
    # Carla connection
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    ego_vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    # Spawn the ego vehicle
    spawn_points = world.get_map().get_spawn_points()
    ego_vehicle = world.spawn_actor(ego_vehicle_bp, spawn_points[0])

    # Set up Carla environment
    carla_env = Env(client, world)
    s_start = carla_env.get_vehicle_position(ego_vehicle)  # Get current vehicle position
    s_goal = (40, 25)  # Define a fixed goal point for testing
    obs = carla_env.get_obstacles()  # Extract static obstacles

    # Update RTAAStar environment
    rtaa = RTAAStar(s_start, s_goal, 240, "euclidean")
    rtaa.Env.update_obs(obs)  # Pass Carla obstacles to RTAA*

    # Run RTAAStar path planning
    rtaa.searching()
    print("Generated Path:", rtaa.path)

    # Follow the planned path
    for waypoint in rtaa.path[-1]:  # Use the final generated path
        control = carla.VehicleControl()
        control.throttle = 0.5
        ego_vehicle.apply_control(control)

        # Update vehicle position to simulate stepwise movement
        ego_vehicle.set_transform(carla.Transform(
            carla.Location(x=waypoint[0], y=waypoint[1], z=0.5),
            ego_vehicle.get_transform().rotation))

        # Wait for the vehicle to "move" to the next waypoint
        time.sleep(0.5)

    # Destroy vehicle after test
    ego_vehicle.destroy()


if __name__ == '__main__':
    main()
