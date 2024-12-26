#!/usr/bin/env python3

# Python packages
import pygame
import DummyWorld
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
import glob
import os
import sys
import argparse
from subprocess import Popen, PIPE
import signal
import numpy as np
import random
import time
import math
import traceback

import docker

import config
import states
import constants as c
from fuzz_utils import quaternion_from_euler, get_carla_transform

import carla

import pdb

from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
from carla_bridge.dreamview_carla import dreamview
from carla_bridge.utils.transforms import carla_transform_to_cyber_pose
from MS_fuzz.common.camera_agent_imageio import ScenarioRecorder
from MS_fuzz.common.result_saver import ResultSaver
from loguru import logger
import threading
import multiprocessing
import socket
import datetime


client = None
tm = None
list_spawn_points = None


def _on_collision(event, state):
    # print("COLLISION:", event)

    if event.frame > state.first_frame_id + state.num_frames:
        # ignore collision happened AFTER simulation ends
        # (can happen because of sluggish garbage collection of Carla)
        return

    if event.other_actor.type_id != "static.road":
        # do not count collision while spawning ego vehicle (hard drop)

        state.crashed = True
        state.collision_event = event


def _on_invasion(event, state):
    # lane_types = set(x.type for x in event.crossed_lane_markings)
    # text = ['%r' % str(x).split()[-1] for x in lane_types]
    # self.hud.notification('Crossed line %s' % ' and '.join(text))

    if event.frame > state.first_frame_id + state.num_frames:
        return

    crossed_lanes = event.crossed_lane_markings
    for crossed_lane in crossed_lanes:
        if crossed_lane.lane_change == carla.LaneChange.NONE:
            # print("LANE INVASION:", event)
            state.laneinvaded = True
            state.laneinvasion_event.append(event)

    # print(crossed_lane.color, crossed_lane.lane_change, crossed_lane.type)
    # print(type(crossed_lane.color), type(crossed_lane.lane_change),
            # type(crossed_lane.type))


def _on_front_camera_capture(image):
    image.save_to_disk(f"/tmp/fuzzerdata/front-{image.frame}.jpg")


def _on_top_camera_capture(image):
    image.save_to_disk(f"/tmp/fuzzerdata/top-{image.frame}.jpg")

# def _on_view_image(self, image):
    # """
    # Callback when receiving a camera image
    # """
    # global _surface
    # array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
    # array = np.reshape(array, (image.height, image.width, 4))
    # array = array[:, :, :3]
    # array = array[:, :, ::-1]
    # _surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))


def set_camera(conf, player, spectator):
    if conf.view == c.BIRDSEYE:
        cam_over_player(player, spectator)
    elif conf.view == c.ONROOF:
        cam_chase_player(player, spectator)
    else:  # fallthru default
        cam_chase_player(player, spectator)


def cam_chase_player(player, spectator):
    location = player.get_location()
    rotation = player.get_transform().rotation
    fwd_vec = rotation.get_forward_vector()

    # chase from behind
    constant = 4
    location.x -= constant * fwd_vec.x
    location.y -= constant * fwd_vec.y
    # and above
    location.z += 3
    rotation.pitch -= 5
    spectator.set_transform(
        carla.Transform(location, rotation)
    )


def cam_over_player(player, spectator):
    location = player.get_location()
    location.z += 100
    # rotation = player.get_transform().rotation
    rotation = carla.Rotation()  # fix rotation for better sim performance
    rotation.pitch -= 90
    spectator.set_transform(
        carla.Transform(location, rotation)
    )


def is_player_on_puddle(player_loc, actor_frictions):
    for friction in actor_frictions:
        len_x = float(friction.attributes["extent_x"])
        len_y = float(friction.attributes["extent_y"])
        loc_x = friction.get_location().x
        loc_y = friction.get_location().y
        p1 = loc_x - len_x / 100
        p2 = loc_x + len_x / 100
        p3 = loc_y - len_y / 100
        p4 = loc_y + len_y / 100
        p_x = player_loc.x
        p_y = player_loc.y
        if p1 <= p_x and p_x <= p2 and p3 <= p_y and p_y <= p4:
            return True
        else:
            return False

def load_carla(server_ip, server_port):
    """Send 'reload' command to the specified server"""
    message = 'reload'
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send message to server
        sock.sendto(message.encode('utf-8'), (server_ip, server_port))
        
        # Receive response from server
        data, server = sock.recvfrom(4096)
        print(f"Received response from server {server}: \r{data.decode('utf-8')}")

    finally:
        sock.close()
        print("waiting for carla")
def connect(conf:config.Config):
    global client
    global tm

    client = carla.Client(conf.sim_host, conf.sim_port)
    need_reload_carla = False
    try:
        client.set_timeout(20.0)
        client.get_server_version()
    except RuntimeError as e:
        if 'time-out' in str(e):
            print("[debug] carla died, try reload")
        need_reload_carla = True
    if need_reload_carla:
        load_carla(conf.sim_host, conf.reload_carla_port)
        print('retrying at, ',time.time())
        success = False
        retry_time = 30
        while retry_time > 0:          
            retry_time -= 1  
            try:
                client = carla.Client(conf.sim_host, conf.sim_port)
                client.set_timeout(2.0)
                v = client.get_server_version()
                print("carla ready at, ",time.time(), v)
                success = True
                break
            except Exception as e:
                print(f'carla not ready, {time.time()}\r',end="")
        if not success:
            print('carla load fail, exit')
            sys.exit(-1)
    
    if conf.debug:
        print("Connected to:", client)

    tm = None
    # tm = client.get_trafficmanager(conf.sim_tm_port)
    # tm.set_synchronous_mode(True)
    # if conf.debug:
    #     print("Traffic Manager Server:", tm)

    return (client, tm)    
def switch_map(conf, town):
    """
    Switch map in the simulator and retrieve legitimate waypoints (a list of
    carla.Transform objects) in advance.
    """
    global client
    global list_spawn_points

    assert (client is not None)

    try:
        client,_ = connect(conf)
        time.sleep(2)
        client,_ = connect(conf) # in case carla died 
        client.set_timeout(20)  # Handle sluggish loading bug
        world = client.get_world()
        # if world.get_map().name != town: # force load every time
        if conf.debug:
            print("[*] Switching town to {} (slow)".format(town))
        client.load_world(str(town))  # e.g., "/Game/Carla/Maps/Town01"
        if conf.debug:
            print("[+] Switched")

        town_map = world.get_map()
        list_spawn_points = town_map.get_spawn_points()

    except Exception as e:
        print("[-] Error:", e)
        sys.exit(-1)

def carla_bridge_handler(host,
                         port,
                         timeout,
                         carla_map,
                         ego_spawn_point: dict = None,
                         close_event: multiprocessing.Event=None):

    carla_bridge = CarlaCyberBridge()
    carla_world = carla.Client(host, port).get_world()
    
    try:
        parameters = {
            'carla': {
                'host': host,
                'port': port,
                'timeout': timeout,
                'passive': False,
                'synchronous_mode': True,
                'synchronous_mode_wait_for_vehicle_control_command': False,
                'fixed_delta_seconds': 0.05,
                'register_all_sensors': True,
                'town': carla_map,
                'ego_vehicle': {
                    'role_name': ["hero", "ego_vehicle"]
                }
            }
        }
        carla_bridge.ego_initial_pos = ego_spawn_point
        carla_bridge.initialize_bridge(carla_world, parameters, logger)
    except (IOError, RuntimeError) as e:
        print(f"[Bridge] Error: {e}")
    except KeyboardInterrupt as e:
        print(f"[Bridge] Error: {e}")
    except Exception as e:  # pylint: disable=W0718
        print(e)
    finally:
        if carla_bridge.shutdown_event:
            carla_bridge.shutdown_event.set()
        time.sleep(0.1)
        settings = carla_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        carla_world.apply_settings(settings)
        carla_bridge.destroy()
        time.sleep(3)
        os.kill(os.getpid(), signal.SIGKILL)

def simulate(conf:config.Config, 
             state:states.State, 
             town, 
             sp:carla.Transform, wp:carla.Transform, 
             weather_dict, 
             frictions_list, 
             actors_list):
    # simulate() is always called by TestScenario instance,
    # so we won't need to switch map unless a new instance is created.
    # switch_map(conf, client, town)

    global client
    global tm

    assert (client is not None)

    retval = 0
    stuck_start_time = time.time()
    curr_datetime = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    sim_result_path = os.path.join(conf.rosbag_dir, f'sim_{curr_datetime}')
    if not os.path.exists(sim_result_path):
            os.makedirs(sim_result_path)
    result_saver = ResultSaver(sim_result_path)
    player_loc = None
    try:
        print("before world setting", time.time())
        client.set_timeout(20.0)
        world = client.get_world()
        if conf.debug:
            print("[debug] world:", world)

        town_map = world.get_map()
        if conf.debug:
            print("[debug] map:", town_map)

        blueprint_library = world.get_blueprint_library()
        
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / c.FRAME_RATE  # FPS
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        
        # MSfuzz: set but never used
        frame_id = world.tick()
        init_frame_id = frame_id
        frame_0 = frame_id
        start_time = time.time()
        clock = pygame.time.Clock()

        # set weather
        weather = world.get_weather()
        weather.cloudiness = weather_dict["cloud"]
        weather.precipitation = weather_dict["rain"]
        weather.precipitation_deposits = weather_dict["puddle"]
        weather.wetness = weather_dict["wetness"]
        weather.wind_intensity = weather_dict["wind"]
        weather.fog_density = weather_dict["fog"]
        weather.sun_azimuth_angle = weather_dict["angle"]
        weather.sun_altitude_angle = weather_dict["altitude"]
        world.set_weather(weather)
        print('MS dbg: weather set')
        
        sensors = []
        actor_vehicles = []
        actor_walkers = []
        actor_controllers = []
        actor_frictions = []
        ros_pid = 0

        world.tick()  # sync once with simulator

        # spawn player
        # how DriveFuzz spawns a player vehicle depends on
        # the autonomous driving agent
        # player_bp = blueprint_library.filter('mercedes-benz')[0] # duplicate in carla 0914
        player_bp = blueprint_library.filter('mercedes')[0]
        # player_bp.set_attribute("role_name", "ego")
        player = None

        goal_loc = wp.location
        goal_rot = wp.rotation

        # mark goal position
        if conf.debug:
            world.debug.draw_box(
                box=carla.BoundingBox(
                    goal_loc,
                    carla.Vector3D(0.2, 0.2, 1.0)
                ),
                rotation=goal_rot,
                life_time=0,
                thickness=1.0,
                color=carla.Color(r=0, g=255, b=0)
            )

        print("MS dbg: enviornment initialied")
        
        apollo_carla_brige_p = None
        apollo_dv:dreamview.Connection = None
        apollo_stop_event:multiprocessing.Event = None
        apollo_recorder:ScenarioRecorder = None
        
        if conf.agent_type == c.BASIC:
            player = world.try_spawn_actor(player_bp, sp)
            if player is None:
                print("[-] Failed spawning player")
                state.spawn_failed = True
                state.spawn_failed_object = 0  # player
                retval = -1
                return  # trap to finally

            world.tick()  # sync once with simulator
            player.set_simulate_physics(True)

            agent = BasicAgent(player)
            agent.set_destination(
                (wp.location.x, wp.location.y, wp.location.z))
            print("[+] spawned BasicAgent")

        elif conf.agent_type == c.BEHAVIOR:
            player = world.try_spawn_actor(player_bp, sp)
            if player is None:
                print("[-] Failed spawning player")
                state.spawn_failed = True
                state.spawn_failed_object = 0  # player
                retval = -1
                return  # trap to finally

            world.tick()  # sync once with simulator
            player.set_simulate_physics(True)

            agent = BehaviorAgent(
                player,
                ignore_traffic_light=True,
                behavior="cautious"
            )
            agent.set_destination(
                start_location=sp.location,
                end_location=wp.location,
                clean=True
            )

            # BehaviorAgent requires a World object to be supplied
            # but internally only dereferences the player object.
            # We'll simply create a DummyWorld instance to avoid the hassle.
            dummy_world = DummyWorld.DummyWorld(world, player)

            print("[+] spawned cautious BehaviorAgent")

        elif conf.agent_type == c.AUTOWARE:
            loc = sp.location
            rot = sp.rotation
            if conf.function == "collision":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = 1.0
                goal_ow = 0.0
            elif conf.function == "traction":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = -0.96
                goal_ow = 0.26
            elif conf.function == "eval-us":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = -0.01
                goal_ow = 0.9998
            elif conf.function == "eval-os":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = 0.679
                goal_ow = 0.733
            else:
                goal_quaternion = quaternion_from_euler(0.0, 0.0, goal_rot.yaw)
                goal_ox = goal_quaternion[0]
                goal_oy = goal_quaternion[1]
                goal_oz = goal_quaternion[2]
                goal_ow = goal_quaternion[3]
            sp_str = "{},{},{},{},{},{}".format(loc.x, loc.y, loc.z, rot.roll,
                                                rot.pitch, rot.yaw * -1)
            goal_str = "{},{},{},{},{},{},{}".format(goal_loc.x, goal_loc.y,
                                                     goal_loc.z, goal_ox, goal_oy, goal_oz, goal_ow)

            docker_client = docker.from_env()
            proj_root = config.get_proj_root()
            xauth = os.path.join(os.getenv("HOME"), ".Xauthority")
            username = os.getenv("USER")
            vol_dict = {
                "{}/carla-autoware/autoware-contents".format(proj_root): {
                    "bind": "/home/autoware/autoware-contents",
                    "mode": "ro"
                },
                "/tmp/.X11-unix": {
                    "bind": "/tmp/.X11-unix",
                    "mode": "rw"
                },
                f"/home/{username}/.Xauthority": {
                    "bind": xauth,
                    "mode": "rw"
                },
                "/tmp/fuzzerdata": {
                    "bind": "/tmp/fuzzerdata",
                    "mode": "rw"
                }
            }
            env_dict = {
                "DISPLAY": os.getenv("DISPLAY"),
                "XAUTHORITY": xauth
            }

            autoware_cla = "{} \'{}\'".format(town_map.name, sp_str)
            print(autoware_cla)
            state.autoware_cmd = autoware_cla

            autoware_container = None
            killed = False
            while autoware_container is None:
                try:
                    autoware_container = docker_client.containers.run(
                        "carla-autoware:improved",
                        command=autoware_cla,
                        detach=True,
                        auto_remove=True,
                        name="autoware-{}".format(os.getenv("USER")),
                        volumes=vol_dict,
                        privileged=True,
                        network_mode="host",
                        runtime="nvidia",
                        environment=env_dict,
                    )
                except docker.errors.APIError as e:
                    print("[-] Could not launch docker:", e)
                    if "Conflict" in str(e):
                        os.system("docker rm -f autoware-{}".format(
                            os.getenv("USER")))
                        killed = True
                    time.sleep(1)
                except:
                    # https://github.com/docker/for-mac/issues/4957
                    print("[-] Fatal error. Check dmesg")
                    exit(-1)

            while True:
                running_container_list = docker_client.containers.list()
                if autoware_container in running_container_list:
                    break
                print("[*] Waiting for Autoware container to be launched")
                time.sleep(1)

            # wait for autoware bridge to spawn player vehicle
            autoware_agent_found = False
            i = 0
            while True:
                print("[*] Waiting for Autoware agent "
                      + "." * i + "\r", end="")
                vehicles = world.get_actors().filter("*vehicle.*")
                for vehicle in vehicles:
                    if vehicle.attributes["role_name"] == "ego_vehicle":
                        autoware_agent_found = True
                        player = vehicle
                        print("\n    [*] found [{}] at {}".format(player.id,
                                                                  player.get_location()))
                        break
                if autoware_agent_found:
                    break
                if i > 60:
                    print("\n something is wrong")
                    exit(-1)
                i += 1
                time.sleep(0.5)

            world.tick()  # sync with simulator
            player.set_transform(sp)
            while True:
                world.tick()  # spin until the player is moved to the sp
                if player.get_location().distance(sp.location) < 1:
                    break
                
        elif conf.agent_type == c.APOLLO:
            print("[*] Spawning Apollo vehicle")
            # pdb.set_trace()
            sp_apollo_pose = carla_transform_to_cyber_pose(sp)
            print(sp.rotation)
            sp_dict = {
                "x": sp.location.x, 
                "y": -sp.location.y, 
                "z": sp.location.z+1, 
                "roll": sp.rotation.roll, 
                "pitch": sp.rotation.pitch, 
                "yaw": sp.rotation.yaw
                }
            town_map_name = town_map.name
            if town_map_name.split("/")[0] == 'Carla':
                town_map_name = town_map_name.split("/")[-1]
                
            apollo_stop_event = multiprocessing.Event()
            apollo_carla_brige_p = multiprocessing.Process(
                target=carla_bridge_handler,
                args=(conf.sim_host, conf.sim_port, conf.timeout,
                      town_map_name, sp_dict, apollo_stop_event))
            apollo_carla_brige_p.start()
            
            # wait for apollo bridge to spawn player vehicle
            apollo_agent_found = False
            i = 0
            while True:
                print("[*] Waiting for Apollo agent "
                      + "." * i + "\r", end="")
                vehicles = world.get_actors().filter("*vehicle.*")
                for vehicle in vehicles:
                    if vehicle.attributes["role_name"] == "ego_vehicle":
                        apollo_agent_found = True
                        player = vehicle
                        print("\n    [*] found [{}] at {}".format(player.id,
                                                                  player.get_location()))
                        break
                if apollo_agent_found:
                    break
                if i > 60:
                    print("\n [Apollo Bridge] something is wrong")
                    exit(-1)
                i += 1
                time.sleep(0.5)

            world.wait_for_tick()  # sync with simulator
            apollo_dv = dreamview.Connection(
                player,
                ip=conf.dreamview_ip, 
                port=str(conf.dreamview_port))
            time.sleep(1)
            
            
            dv_map = conf.dreamview_map.get(town_map_name.lower())
            if not dv_map:
                print(f"[Apollo Bridge] Dreamview map not found for {town_map_name.lower()}")
                sys.exit(-1)
            print(f"[Apollo Bridge] Setting up Dreamview for '{dv_map}', '{conf.dreamview_vehicle}'")
            apollo_dv.set_hd_map(dv_map)
            apollo_dv.set_vehicle(conf.dreamview_vehicle)
            apollo_dv.set_setup_mode('Mkz Standard Debug')
            
            for mod in conf.apollo_modules:
                print("[Apollo Bridge] Starting {} module...".format(mod))
                apollo_dv.enable_module(mod)
            time.sleep(3)
            print('[+][Apollo Bridge] Apollo Ready! at', time.time())
            # pdb.set_trace()
        
        
        # Attach collision detector
        collision_bp = blueprint_library.find('sensor.other.collision')
        sensor_collision = world.spawn_actor(collision_bp, carla.Transform(),
                                             attach_to=player)
        sensor_collision.listen(lambda event: _on_collision(event, state))
        sensors.append(sensor_collision)

        # Attach lane invasion sensor
        lanesensor_bp = blueprint_library.find("sensor.other.lane_invasion")
        sensor_lane = world.spawn_actor(lanesensor_bp, carla.Transform(),
                                        attach_to=player)
        sensor_lane.listen(lambda event: _on_invasion(event, state))
        sensors.append(sensor_lane)

        # add cam
        if conf.agent_type in [c.BASIC, c.BEHAVIOR]:
            # Attach RGB camera (front)
            rgb_camera_bp = blueprint_library.find("sensor.camera.rgb")

            rgb_camera_bp.set_attribute("image_size_x", "800")
            rgb_camera_bp.set_attribute("image_size_y", "600")
            rgb_camera_bp.set_attribute("fov", "105")

            # position relative to the parent actor (player)
            camera_tf = carla.Transform(carla.Location(z=1.8))

            # time in seconds between sensor captures - should sync w/ fps?
            # rgb_camera_bp.set_attribute("sensor_tick", "1.0")

            camera_front = world.spawn_actor(
                rgb_camera_bp,
                camera_tf,
                attach_to=player,
                attachment_type=carla.AttachmentType.Rigid
            )

            camera_front.listen(lambda image: _on_front_camera_capture(image))

            sensors.append(camera_front)

            camera_tf = carla.Transform(
                carla.Location(z=50.0),
                carla.Rotation(pitch=-90.0)
            )
            camera_top = world.spawn_actor(
                rgb_camera_bp,
                camera_tf,
                attach_to=player,
                attachment_type=carla.AttachmentType.Rigid
            )

            camera_top.listen(lambda image: _on_top_camera_capture(image))

        elif conf.agent_type == c.APOLLO:
            apollo_recorder = ScenarioRecorder(world,
                                               player,
                                               '/tmp/fuzzerdata')
            apollo_recorder.start_recording('/tmp/fuzzerdata/recorder.mp4')
        
        
        
        if conf.agent_type == c.APOLLO:
            world.wait_for_tick()
        else:
            world.tick()  # sync with simulator

        # get vehicle's maximum steering angle
        physics_control = player.get_physics_control()
        max_steer_angle = 0
        for wheel in physics_control.wheels:
            if wheel.max_steer_angle > max_steer_angle:
                max_steer_angle = wheel.max_steer_angle

        # (optional) attach spectator
        # spectator = world.get_spectator()
        # set_camera(conf, player, spectator)

        # spawn friction triggers
        friction_bp = blueprint_library.find('static.trigger.friction')
        for friction in frictions_list:
            friction_bp.set_attribute('friction', str(friction["level"]))
            friction_bp.set_attribute('extent_x', str(friction["size"][0]))
            friction_bp.set_attribute('extent_y', str(friction["size"][1]))
            friction_bp.set_attribute('extent_z', str(friction["size"][2]))

            friction_sp_transform = get_carla_transform(
                friction["spawn_point"]
            )
            friction_size_loc = carla.Location(
                friction["size"][0],
                friction["size"][1],
                friction["size"][2]
            )

            friction_trigger = world.try_spawn_actor(
                friction_bp, friction_sp_transform)

            if friction_trigger is None:
                print("[-] Failed spawning lvl {} puddle at ({}, {})".format(
                    friction["level"],
                    friction_sp_transform.location.x,
                    friction_sp_transform.location.y)
                )

                state.spawn_failed = True
                state.spawn_failed_object = friction
                retval = -1
                return
            actor_frictions.append(friction_trigger)  # to destroy later

            # Optional for visualizing trigger (for debugging)
            if conf.debug:
                world.debug.draw_box(
                    box=carla.BoundingBox(
                        friction_sp_transform.location,
                        friction_size_loc * 1e-2
                    ),
                    rotation=friction_sp_transform.rotation,
                    life_time=0,
                    thickness=friction["level"]
                    * 1,  # the stronger the thicker
                    color=carla.Color(r=0, g=0, b=255)
                )
            print("[+] New puddle [%d] @(%.2f, %.2f) lvl %.2f" % (
                friction_trigger.id,
                friction_sp_transform.location.x,
                friction_sp_transform.location.y,
                friction["level"])
            )

        # spawn actors
        vehicle_bp = blueprint_library.find("vehicle.bmw.grandtourer")
        vehicle_bp.set_attribute("color", "255,0,0")
        walker_bp = blueprint_library.find(
            "walker.pedestrian.0001")  # 0001~0014
        walker_controller_bp = blueprint_library.find('controller.ai.walker')

        for actor in actors_list:
            actor_sp = get_carla_transform(actor["spawn_point"])
            if actor["type"] == c.VEHICLE:  # vehicle
                actor_vehicle = world.try_spawn_actor(vehicle_bp, actor_sp)
                actor_nav = c.NAVTYPE_NAMES[actor["nav_type"]]
                if actor_vehicle is None:
                    actor_str = c.ACTOR_NAMES[actor["type"]]
                    print("[-] Failed spawning {} {} at ({}, {})".format(
                        actor_nav, actor_str, actor_sp.location.x,
                        actor_sp.location.y)
                    )

                    state.spawn_failed = True
                    state.spawn_failed_object = actor
                    retval = -1
                    return  # trap to finally

                actor_vehicles.append(actor_vehicle)
                print("[+] New %s vehicle [%d] @(%.2f, %.2f) yaw %.2f" % (
                    actor_nav,
                    actor_vehicle.id,
                    actor_sp.location.x,
                    actor_sp.location.y,
                    actor_sp.rotation.yaw)
                )

            elif actor["type"] == c.WALKER:  # walker
                actor_walker = world.try_spawn_actor(walker_bp, actor_sp)
                actor_nav = c.NAVTYPE_NAMES[actor["nav_type"]]
                if actor_walker is None:
                    actor_str = c.ACTOR_NAMES[actor["type"]]
                    print("[-] Failed spawning {} {} at ({}, {})".format(
                        actor_nav, actor_str, actor_sp.location.x,
                        actor_sp.location.y)
                    )

                    state.spawn_failed = True
                    state.spawn_failed_object = actor
                    retval = -1
                    return  # trap to finally

                actor_walkers.append(actor_walker)
                print("[+] New %s walker [%d] @(%.2f, %.2f) yaw %.2f" % (
                    actor_nav,
                    actor_walker.id,
                    actor_sp.location.x,
                    actor_sp.location.y,
                    actor_sp.rotation.yaw)
                )
        # print("after spawning actors", time.time())

        if conf.agent_type == c.AUTOWARE:
            # print("before launching autoware", time.time())
            num_vehicle_topics = len(actor_vehicles)
            num_walker_topics = 0
            if len(actor_walkers) > 0:
                num_walker_topics = 2
            # clock = pygame.time.Clock()
            i = 0
            while True:
                print("[*] Waiting for Autoware nodes "
                      + "." * i + "\r", end="")
                proc1 = Popen(["rostopic", "list"], stdout=PIPE)
                proc2 = Popen(["wc", "-l"], stdin=proc1.stdout, stdout=PIPE)
                proc1.stdout.close()
                output = proc2.communicate()[0]

                num_topics = c.WAIT_AUTOWARE_NUM_TOPICS + \
                    num_vehicle_topics + num_walker_topics
                if int(output) >= num_topics:
                    # FIXME: hardcoding the num of topics :/
                    # on top of that, each vehicle adds one topic, and any walker
                    # contribute to two pedestrian topics.
                    print("")
                    break
                i += 1
                if i == 60:
                    print(
                        "    [-] something went wrong while launching Autoware.")
                    raise KeyboardInterrupt
                time.sleep(0.5)

            world.tick()

            # exec a detached process that monitors the output of Autoware's
            # decision-maker state, with which we can get an idea of when Autoware
            # thinks it has reached the goal
            proc_state = Popen(["rostopic echo /decision_maker/state"],
                               shell=True, stdout=PIPE, stderr=PIPE)

            # set_camera(conf, player, spectator)

            # Wait for Autoware (esp, for Town04)
            while True:
                output_state = proc_state.stdout.readline()
                if b"---" in output_state:
                    output_state = proc_state.stdout.readline()
                if b"VehicleReady" in output_state:
                    break
                time.sleep(0.5)

            pub_topic = "/move_base_simple/goal"
            msg_type = "geometry_msgs/PoseStamped"
            goal_hdr = "header: {stamp: now, frame_id: \'map\'}"
            goal_pose = "pose: {position: {x: %.6f, y: %.6f, z: 0}, orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}" % (
                goal_loc.x, (-1) * float(goal_loc.y), goal_ox, goal_oy, goal_oz, goal_ow)
            goal_msg = "'{" + goal_hdr + ", " + goal_pose + "}'"
            pub_cmd = "rostopic pub --once {} {} {} > /dev/null".format(
                pub_topic, msg_type, goal_msg)
            os.system(pub_cmd)
            if conf.debug:
                print(goal_msg)
            print("[carla] Goal published")
            time.sleep(1)  # give some time (Autoware initialization is slow)
            state.autoware_goal = pub_cmd

            world.tick()
            # print("after launching autoware", time.time())

        elif conf.agent_type == c.APOLLO:
            print('[*] Settting up and setting goal for Apollo')
            def wait_until_vehicle_moving(ego_vehicle, timeout=5.0):
                to = timeout
                while to > 0:
                    curr_speed = ego_vehicle.get_velocity().x
                    print(f'curr speed : {int(curr_speed*100)/100}\r', end='')
                    if curr_speed > 0.01 or curr_speed < -0.01:
                        return True
                    time.sleep(0.1)
                    to -= 0.1
                return False
            goal_t = wp
            for i in range(10):
                apollo_dv.set_destination_tranform(goal_t)
                if wait_until_vehicle_moving(player,3):
                    logger.info('[Simulator] Vehicle is started')
                    break
            # pdb.set_trace()
        
        # print("real simulation begins", time.time())
        # handle actor missions after Autoware's goal is published
        cnt_v = 0
        cnt_w = 0
        for actor in actors_list:
            actor_sp = get_carla_transform(actor["spawn_point"])
            actor_dp = get_carla_transform(actor["dest_point"])
            if actor["type"] == c.VEHICLE:  # vehicle
                actor_vehicle = actor_vehicles[cnt_v]
                cnt_v += 1
                if actor["nav_type"] == c.LINEAR:
                    forward_vec = actor_sp.rotation.get_forward_vector()
                    actor_vehicle.set_target_velocity(
                        forward_vec * actor["speed"])

                elif actor["nav_type"] == c.MANEUVER:
                    # set initial speed 0
                    # trajectory control happens in the control loop below
                    forward_vec = actor_sp.rotation.get_forward_vector()
                    actor_vehicle.set_target_velocity(forward_vec * 0)

                elif actor["nav_type"] == c.AUTOPILOT:
                    # TODO: Get out of the control of traffic manager and use basic agent instead
                    # actor_vehicle.set_autopilot(True, tm.get_port())
                    actor_vehicle.set_autopilot(True)

                actor_vehicle.set_simulate_physics(True)
            elif actor["type"] == c.WALKER:  # walker
                actor_walker = actor_walkers[cnt_w]
                cnt_w += 1
                if actor["nav_type"] == c.LINEAR:
                    forward_vec = actor_sp.rotation.get_forward_vector()
                    controller_walker = carla.WalkerControl()
                    controller_walker.direction = forward_vec
                    controller_walker.speed = actor["speed"]
                    actor_walker.apply_control(controller_walker)

                elif actor["nav_type"] == c.AUTOPILOT:
                    controller_walker = world.spawn_actor(
                        walker_controller_bp,
                        actor_sp,
                        actor_walker)

                    if conf.agent_type == c.APOLLO:
                        world.wait_for_tick()
                    else:
                        world.tick()  # without this, walker vanishes
                    controller_walker.start()
                    controller_walker.set_max_speed(float(actor["speed"]))
                    controller_walker.go_to_location(actor_dp.location)

                elif actor["nav_type"] == c.IMMOBILE:
                    controller_walker = None

                if controller_walker:  # can be None if immobile walker
                    actor_controllers.append(controller_walker)

        elapsed_time = 0
        start_time = time.time()

        yaw = sp.rotation.yaw

        player_loc = player.get_transform().location
        init_x = player_loc.x
        init_y = player_loc.y

        result_saver.clear_result()
        result_saver.result_to_save['video_path'] = None
        result_saver.result_to_save['start_loc'] = {
            'x': player_loc.x,
            'y': player_loc.y,
            'z': player_loc.z
        }
        result_saver.result_to_save['dest_loc'] = {
            'x': wp.location.x,
            'y': wp.location.y,
            'z': wp.location.z
        }
        result_saver.result_to_save['start_time'] = time.time()
        
        # SIMULATION LOOP FOR AUTOWARE and BasicAgent
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGSEGV, state.sig_handler)
        signal.signal(signal.SIGABRT, state.sig_handler)

        try:
            # actual monitoring of the driving simulation begins here
            snapshot0 = world.get_snapshot()
            first_frame_id = snapshot0.frame
            first_sim_time = snapshot0.timestamp.elapsed_seconds

            last_frame_id = first_frame_id

            state.first_frame_id = first_frame_id
            state.sim_start_time = snapshot0.timestamp.platform_timestamp
            state.num_frames = 0
            state.elapsed_time = 0
            s_started = False
            s_stopped_frames = 0

            if conf.debug:
                print("[*] START DRIVING: {} {}".format(first_frame_id,
                                                        first_sim_time))

            while True:
                # Use sampling frequency of FPS*2 for precision!
                clock.tick(c.FRAME_RATE * 2)

                # Carla agents are running in synchronous mode,
                # so we need to send ticks. Not needed for Autoware
                if conf.agent_type == c.BASIC or conf.agent_type == c.BEHAVIOR:
                    world.tick()
                # elif conf.agent_type == c.APOLLO:
                #     world.wait_for_tick()

                snapshot = world.get_snapshot()
                ego_ss = snapshot.find(player.id)
                npc_vehicles_ss = [snapshot.find(av.id) for av in actor_vehicles]
                cur_frame_id = snapshot.frame
                cur_sim_time = snapshot.timestamp.elapsed_seconds
                
                saver_frame= {
                    'timestamp': time.time(),
                    'frame': cur_frame_id,
                    'npc_vehicles_ss': npc_vehicles_ss,
                    'ego_ss': ego_ss
                }
                result_saver.add_frame(saver_frame)
                
                if cur_frame_id <= last_frame_id:
                    # skip if we got the same frame data as last
                    continue

                last_frame_id = cur_frame_id  # update last
                state.num_frames = cur_frame_id - first_frame_id
                state.elapsed_time = cur_sim_time - first_sim_time

                player_transform = player.get_transform()
                player_loc = player_transform.location
                player_rot = player_transform.rotation

                # Get speed
                vel = player.get_velocity()
                speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                speed_limit = player.get_speed_limit()

                try:
                    last_speed_limit = state.speed_lim[-1]
                except:
                    last_speed_limit = 0

                if speed_limit != last_speed_limit:
                    frame_speed_lim_changed = cur_frame_id

                state.speed.append(speed)
                state.speed_lim.append(speed_limit)

                print("(%.2f,%.2f)>(%.2f,%.2f)>(%.2f,%.2f) %.2f m left, %.2f/%d km/h   \r" % (
                    sp.location.x, sp.location.y, player_loc.x,
                    player_loc.y, goal_loc.x, goal_loc.y,
                    player_loc.distance(goal_loc),
                    speed, speed_limit), end="")

                if player.is_at_traffic_light():
                    traffic_light = player.get_traffic_light()
                    if traffic_light.get_state() == carla.TrafficLightState.Red:
                        # within red light triggerbox
                        if state.on_red:
                            state.on_red_speed.append(speed)
                        else:
                            state.on_red = True
                            state.on_red_speed = list()
                else:
                    # not at traffic light
                    if state.on_red:
                        # out of red light triggerbox
                        state.on_red = False
                        stopped_at_red = False
                        for i, ors in enumerate(state.on_red_speed):
                            if ors < 0.1:
                                stopped_at_red = True

                        if not stopped_at_red:
                            state.red_violation = True

                # world.debug.draw_point(
                        # player_loc + carla.Location(z=10),
                        # size=0.1,
                        # life_time=0.1,
                        # color=carla.Color(255, 0, 0)
                    # )
                # set_camera(player, spectator)
                control:carla.VehicleControl = None
                if conf.agent_type == c.BASIC:
                    # for carla agents, we should apply controls ourselves
                    # XXX: check and resolve BehaviorAgent's run_step issue of
                    # not being able to get adjacent waypoints
                    control = agent.run_step(debug=True)
                    player.apply_control(control)

                elif conf.agent_type == c.BEHAVIOR:
                    agent.update_information(dummy_world)
                    agent.get_local_planner().set_speed(speed_limit)

                    control = agent.run_step(debug=True)
                    player.apply_control(control)

                elif conf.agent_type in [c.AUTOWARE, c.APOLLO]:
                    # autoware does it on its own. we just retrieve the
                    # control for state computation
                    control = player.get_control()

                state.cont_throttle.append(control.throttle)
                state.cont_brake.append(control.brake)
                state.cont_steer.append(control.steer)
                steer_angle = control.steer * max_steer_angle
                state.steer_angle_list.append(steer_angle)

                current_yaw = player_rot.yaw
                state.yaw_list.append(current_yaw)

                yaw_diff = current_yaw - yaw
                # Yaw range is -180 ~ 180. When vehicle's yaw is oscillating
                # b/w -179 and 179, yaw_diff can be messed up even if the
                # diff is very small. Assuming that it's unrealistic that
                # a vehicle turns more than 180 degrees in under 1/20 seconds,
                # just round the diff around 360.
                if yaw_diff > 180:
                    yaw_diff = 360 - yaw_diff
                elif yaw_diff < -180:
                    yaw_diff = 360 + yaw_diff

                yaw_rate = yaw_diff * c.FRAME_RATE
                state.yaw_rate_list.append(yaw_rate)
                yaw = current_yaw

                # uncomment below to follow along the player
                # set_camera(conf, player, spectator)

                for v in actor_vehicles:
                    dist = player_loc.distance(v.get_location())
                    if dist < state.min_dist:
                        state.min_dist = dist

                for w in actor_walkers:
                    dist = player_loc.distance(w.get_location())
                    if dist < state.min_dist:
                        state.min_dist = dist

                # Get the lateral speed
                player_right_vec = player_rot.get_right_vector()

                # [Note]
                # Lateral velocity is a scalar projection of velocity vector.
                # A: velocity vector.
                # B: right vector. B is a unit vector, thus |B| = 1
                # lat_speed = |A| * cos(theta)
                # As dot_product(A, B) = |A| * |B| * cos(theta),
                # lat_speed = dot_product(A, B) / |B|
                # Given that |B| is always 1,
                # we get lat_speed = dot_product(A, B), which is equivalent to
                # lat_speed = vel.x * right_vel.x + vel.y * right_vel.y

                lat_speed = abs(vel.x * player_right_vec.x
                                + vel.y * player_right_vec.y)
                lat_speed *= 3.6  # m/s to km/h
                state.lat_speed_list.append(lat_speed)

                player_fwd_vec = player_rot.get_forward_vector()
                lon_speed = abs(vel.x * player_fwd_vec.x
                                + vel.y * player_fwd_vec.y)
                lon_speed *= 3.6
                state.lon_speed_list.append(lon_speed)

                # Handle actor maneuvers
                if conf.strategy == c.TRAJECTORY:
                    actor = actors_list[0]
                    maneuvers = actor["maneuvers"]

                    maneuver_id = int(state.num_frames / c.FRAMES_PER_TIMESTEP)
                    if maneuver_id < 5:
                        maneuver = maneuvers[maneuver_id]

                        if maneuver[2] == 0:
                            # print(f"\nPerforming maneuver #{maneuver_id} at frame {state.num_frames}")
                            # mark as done
                            maneuver[2] = state.num_frames

                            # retrieve the actual actor vehicle object
                            # there is only one actor in Trajectory mode
                            actor_vehicle = actor_vehicles[0]

                            # perform the action
                            actor_direction = maneuver[0]
                            actor_speed = maneuver[1]

                            forward_vec = get_carla_transform(
                                actor["spawn_point"]).rotation.get_forward_vector()

                            if actor_direction == 0:  # forward

                                actor_vehicle.set_target_velocity(
                                    forward_vec * actor_speed
                                )

                        elif maneuver[2] > 0 and abs(maneuver[2] - state.num_frames) < 40:
                            # continuously apply lateral force to the vehicle
                            # for 40 frames (2 secs)
                            actor_direction = maneuver[0]
                            apex_degree = maneuver[1]

                            """
                            Model smooth lane changing through varying thetas
                            (theta)
                            45           * *
                            30       * *     * *
                            15     *             * *
                            0  * *                   *
                               0 5 10 15 20 25 30 35 40 (t = # frame)
                            """

                            theta_max = apex_degree
                            force_constant = 5  # should weigh by actor_speed?

                            t = abs(maneuver[2] - state.num_frames)
                            if t < 20:
                                theta = t * (theta_max / 20)
                            else:
                                theta = t * -1 * \
                                    (theta_max / 20) + 2 * theta_max

                            if actor_direction != 0:  # skip if fwd
                                if actor_direction == -1:  # switch to left lane
                                    theta *= -1  # turn cc-wise
                                elif actor_direction == 1:  # switch to right lane
                                    pass  # turn c-wise

                                theta_rad = math.radians(theta)
                                sin = math.sin(theta_rad)
                                cos = math.cos(theta_rad)

                                x0 = forward_vec.x
                                y0 = forward_vec.y

                                x1 = cos * x0 - sin * y0
                                y1 = sin * x0 + cos * y0

                                dir_vec = carla.Vector3D(x=x1, y=y1, z=0.0)
                                actor_vehicle.set_target_velocity(
                                    dir_vec * force_constant
                                )

                # Check Autoware-defined destination
                # VehicleReady\nDriving\nMoving\nLaneArea\nCruise\nStraight\nDrive\nGo\n
                # VehicleReady\nWaitOrder\nStopping\nWaitDriveReady\n

                # Check destination
                dist_to_goal = player_loc.distance(goal_loc)

                if conf.agent_type == c.AUTOWARE:
                    if not conf.function.startswith("eval"):
                        output_state = proc_state.stdout.readline()
                        if b"---" in output_state:
                            output_state = proc_state.stdout.readline()
                        if b"Go" in output_state:
                            s_started = True
                        elif b"nWaitDriveReady" in output_state and s_started:
                            print("\n[*] (Autoware) Reached the destination")
                            print("      dist to goal:", dist_to_goal)

                            if dist_to_goal > 2 and state.num_frames > 300:
                                state.other_error = "goal"
                                state.other_error_val = dist_to_goal

                            retval = 0
                            break
                elif conf.agent_type == c.APOLLO:
                    if dist_to_goal < 10 and control.brake > 0.65:
                        # after reaching destination, apollo will set brake to 70%
                        print("\n[*] (Apollo) Reached the destination")
                        retval = 0
                        break
                elif conf.agent_type == c.BASIC:
                    if hasattr(agent, "done") and agent.done():
                        print("\n[*] (BasicAgent) Reached the destination")

                        if dist_to_goal > 2 and state.num_frames > 300:
                            state.other_error = "goal"
                            state.other_error_val = dist_to_goal

                        break

                elif conf.agent_type == c.BEHAVIOR:
                    lp = agent.get_local_planner()
                    if len(lp.waypoints_queue) == 0:
                        print("\n[*] (BehaviorAgent) Reached the destination")

                        if dist_to_goal > 2 and state.num_frames > 300:
                            state.other_error = "goal"
                            state.other_error_val = dist_to_goal

                        break

                if dist_to_goal < 2:
                    print("\n[*] (Carla heuristic) Reached the destination")
                    retval = 0
                    break

                # Check speeding
                if conf.check_dict["speed"]:
                    # allow T seconds to slow down if speed limit suddenly
                    # decreases
                    T = 3  # 0 for strict checking
                    if (speed > speed_limit
                            and cur_frame_id > frame_speed_lim_changed + T * c.FRAME_RATE):
                        print("\n[*] Speed violation: {} km/h on a {} km/h road".format(
                            speed, speed_limit))
                        state.speeding = True
                        retval = 1
                        break

                # Check crash
                if conf.check_dict["crash"]:
                    if state.crashed:
                        print("\n[*] Collision detected: %.2f" % (
                            state.elapsed_time))
                        retval = 1
                        break

                # Check lane violation
                if conf.check_dict["lane"]:
                    if state.laneinvaded:
                        print("\n[*] Lane invasion detected: %.2f" % (
                            state.elapsed_time))
                        retval = 1
                        break

                # Check traffic light violation
                if conf.check_dict["red"]:
                    if state.red_violation:
                        print("\n[*] Red light violation detected: %.2f" % (
                            state.elapsed_time))
                        retval = 1
                        break

                # Check inactivity
                if speed < 1:  # km/h
                    state.stuck_duration += 1
                else:
                    state.stuck_duration = 0
                    # refresh time
                    stuck_start_time = time.time()

                if conf.check_dict["stuck"]:
                    stuck_for = time.time() - stuck_start_time
                    if stuck_for >= 10 and (int(stuck_for) % 2) == 0 :
                        # print(f'\r has stuck for {int(stuck_for)} \n')
                        pass
                    if state.stuck_duration > (conf.timeout * c.FRAME_RATE) or \
                        stuck_for > conf.timeout:
                        state.stuck = True
                        print("\n[*] Stuck for too long: %d" %
                              (state.stuck_duration))
                        retval = 1
                        break

                if conf.check_dict["other"]:
                    if state.num_frames > 12000:  # over 10 minutes
                        print("\n[*] Simulation taking too long")
                        state.other_error = "timeout"
                        state.other_error_val = state.num_frames
                        retval = 1
                        break
                    if state.other_error:
                        print("\n[*] Other error: %d" % (state.signal))
                        retval = 1
                        break

        except KeyboardInterrupt:
            print("quitting")
            retval = 128

        # jump to finally
        return

    except Exception as e:
        # update states
        # state.num_frames = frame_id - frame_0
        # state.elapsed_time = time.time() - start_time

        print("[-] Runtime error:")
        traceback.print_exc()
        # exc_type, exc_obj, exc_tb = sys.exc_info()
        # print("   (line #{0}) {1}".format(exc_tb.tb_lineno, exc_type))

        retval = 1

    finally:
        # Finalize simulation
        # rospy.signal_shutdown("fin")

        if retval == 1:
            if state.stuck:
                result_saver.result_to_save['unsafe'] = True
                result_saver.result_to_save['unsafe_type'] = 'STUCK'
            elif state.crashed:
                result_saver.result_to_save['unsafe'] = True
                result_saver.result_to_save['unsafe_type'] = 'COLLISION'
        if retval != -1 and player_loc:
            result_saver.save_result(player_loc, save_video=True)
        
        if conf.agent_type in [c.BASIC, c.BEHAVIOR]:
            # assemble images into an mp4 container
            # remove jpg files
            print("Saving front camera video", end=" ")

            vid_filename = "/tmp/fuzzerdata/front.mp4"
            if os.path.exists(vid_filename):
                os.remove(vid_filename)

            cmd_cat = "cat /tmp/fuzzerdata/front-*.jpg"
            cmd_ffmpeg = " ".join([
                "ffmpeg",
                "-f image2pipe",
                f"-r {c.FRAME_RATE}",
                "-vcodec mjpeg",
                "-i -",
                "-vcodec libx264",
                vid_filename
            ])

            cmd = f"{cmd_cat} | {cmd_ffmpeg} {c.DEVNULL}"
            os.system(cmd)
            print("(done)")

            cmd = "rm -f /tmp/fuzzerdata/front-*.jpg"
            os.system(cmd)

            print("Saving top camera video", end=" ")

            vid_filename = "/tmp/fuzzerdata/top.mp4"
            if os.path.exists(vid_filename):
                os.remove(vid_filename)

            cmd_cat = "cat /tmp/fuzzerdata/top-*.jpg"
            cmd_ffmpeg = " ".join([
                "ffmpeg",
                "-f image2pipe",
                f"-r {c.FRAME_RATE}",
                "-vcodec mjpeg",
                "-i -",
                "-vcodec libx264",
                vid_filename
            ])

            cmd = f"{cmd_cat} | {cmd_ffmpeg} {c.DEVNULL}"
            os.system(cmd)
            print("(done)")

            cmd = "rm -f /tmp/fuzzerdata/top-*.jpg"
            os.system(cmd)

        elif conf.agent_type == c.AUTOWARE:
            os.system("rosnode kill /recorder_video_front")
            os.system("rosnode kill /recorder_video_rear")
            os.system("rosnode kill /recorder_bag")
            while os.path.exists("/tmp/fuzzerdata/bagfile.lz4.bag.active"):
                print("waiting for rosbag to dump data")
                time.sleep(1)

            try:
                autoware_container.kill()
            except docker.errors.APIError as e:
                print("[-] Couldn't kill Autoware container:", e)
            except UnboundLocalError:
                print("[-] Autoware container was not launched")
            except:
                print("[-] Autoware container was not killed for an unknown reason")
                print("    Trying manually")
                os.system("docker rm -f autoware-{}".format(os.getenv("USER")))
                # still doesn't fix docker hanging..

        elif conf.agent_type == c.APOLLO:
            try:
                if apollo_recorder:
                    print("Stopped recording")
                    apollo_recorder.stop_recording()
                    apollo_recorder.rm_cams()
                    world.wait_for_tick()
                    # del apollo_recorder
                    apollo_recorder = None
                for v in actor_vehicles:
                    print('removing vehicle',v.id)
                    v.destroy()
                for w in actor_walkers:
                    print('removing walker',w.id)
                    w.destroy()
                for s in sensors:
                    print('removing sensor',s.id)
                    s.stop()
                    s.destroy()
                world.wait_for_tick()
                print('all objects cleaned')
            except Exception as e:
                print('Error Cleaning',e)
            if 'apollo_carla_brige_p' in locals():
                if apollo_carla_brige_p :
                    os.kill(apollo_carla_brige_p.pid, signal.SIGINT)
                    apollo_carla_brige_p.join(7)
                    if apollo_carla_brige_p.is_alive():
                        print("[-] Couldn't kill apollo_carla_bridge process")
                        print("    Trying Force")
                        os.kill(apollo_carla_brige_p.pid, signal.SIGKILL)
                    apollo_carla_brige_p = None      
                    try:
                        settings = world.get_settings()
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = None
                        world.apply_settings(settings) 
                    except Exception as e:
                        print('reset synch mode',e)

        # Don't reload and exit if user requests so
        if retval == 128:
            return retval

        else:
            # try:
            #     client.set_timeout(10.0)
            #     client.reload_world()
            # except RuntimeError:
            #     pass
            
            if conf.debug:
                print('[debug] done.')

            return retval
