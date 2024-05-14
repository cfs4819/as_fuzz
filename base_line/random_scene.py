#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import glob
import os
import sys
import time
from datetime import datetime


import carla
from carla import VehicleLightState as vls

import logging
from loguru import logger
from numpy import random
import threading

from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
from carla_bridge.dreamview_carla import dreamview
from carla_bridge.utils.transforms import carla_transform_to_cyber_pose
from MS_fuzz.fuzz_config.Config import Config
from MS_fuzz.common.camera_agent import ScenarioRecorder
from MS_fuzz.common.scenario import LocalScenario
import pdb


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(
                x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


class RandomScenario():
    def __init__(self, cfgs: Config, tm_port=5005):
        self.carla_client = None
        self.carla_world = None
        self.carla_map = None
        self.ego_vehicle = None
        self.destination = None
        self.carla_bridge: CarlaCyberBridge = None
        self.carla_bridge_thread = None
        self.cfgs: Config = cfgs
        self.tm_port = tm_port

        self.sim_status = False

        # self.ego_spawn_loc = {'x': -227,
        #                       'y': -34,
        #                       'z': 0.2}  # loc in apollo map
        self.ego_spawn_loc = None
        self.simulation_count = 0

        self.modules = [
            # 'Localization',  # ok
            'Transform',  # ok
            'Routing',
            'Prediction',  # ok
            'Planning',  # ok
            'Control',
            'Storytelling'  # ok
        ]

        if cfgs.load_bridge:
            self.carla_bridge = CarlaCyberBridge()

        self.max_v_num = 20
        self.max_w_num = 10

        self.tm_thread: threading.Thread = None
        self.tm_close_event: threading.Event = threading.Event()

        self.close_event = threading.Event()

        self.recorder: ScenarioRecorder = None

        self.destination = None

        self.random_scenario: LocalScenario = None

    def select_valid_dest(self, radius=100) -> carla.Transform:
        '''
            Select a destination outside the specified radius from current position
        '''
        ego_curr_point = self.ego_vehicle.get_transform()
        valid_destination = False
        while not valid_destination:
            des_transform = random.choice(
                self.carla_map.get_spawn_points())
            distance = ego_curr_point.location.distance(des_transform.location)
            if distance > radius:
                valid_destination = True
        return des_transform

    def connect_carla(self, reload_world=True):
        '''
        Connect to carla simualtor.
        '''
        if (self.carla_client != None
                and self.carla_world != None):
            logger.warning("Connection already exists")
            return
        try:
            logger.info(
                f"[Simulator] Connecting to Carla on {self.cfgs.sim_host} {self.cfgs.sim_port}")
            self.carla_client = carla.Client(
                host=self.cfgs.sim_host, port=self.cfgs.sim_port
            )
            self.carla_client.set_timeout(self.cfgs.load_world_timeout)
            if self.cfgs.load_bridge and reload_world:
                self.carla_client.load_world(self.cfgs.carla_map)
            self.carla_world = self.carla_client.get_world()
            self.carla_map = self.carla_world.get_map()

        except Exception as e:
            logger.error(f'[Simulator] Connect Carla wrong: {e}')
        except KeyboardInterrupt as e:
            logger.error('[Simulator]KeyboardInterrupt: {e.message}')
            self.close()
            return
        logger.info(
            f'[Simulator] Connected {self.cfgs.sim_host} {self.cfgs.sim_port}')
        if self.cfgs.load_bridge:
            # if load_bridge, ego_vehicle can only found after bridge is loaded.
            return
        all_vehicles = self.carla_world.get_actors().filter("*vehicle.*")
        ego_existed = False
        for vehicle in all_vehicles:
            if vehicle.attributes["role_name"] == "ego_vehicle":
                self.ego_vehicle = vehicle
                ego_existed = True
                break
        if not ego_existed:
            logger.error("[Simulator] Can't find ego_vehicle.\
                         Check if carla_bridge is running properly.")
            self.close()

    def load_carla_bridge(self, ego_spawn_point: dict = None):
        '''
            parameter: 
                ego_spawn_point: the point ego vehicle to spawn
                    type: dict{
                        'x': float,
                        'y': float,
                        'z': float,
                        'roll': float,
                        'pitch': float,
                        'yaw': float,
                    }
            return: None
        '''
        if self.carla_world == None:
            logger.error(
                "[Bridge] carla needed to be connected before loading bridge")
            return
        self.carla_bridge_thread = threading.Thread(
            target=self.carla_bridge_handler,
            args=(ego_spawn_point,))
        self.carla_bridge_thread.start()

    def carla_bridge_handler(self, ego_spawn_point: dict = None):
        try:
            parameters = {
                'carla': {
                    'host': self.cfgs.sim_host,
                    'port': self.cfgs.sim_port,
                    'timeout': self.cfgs.load_world_timeout,
                    'passive': False,
                    'synchronous_mode': True,
                    'synchronous_mode_wait_for_vehicle_control_command': False,
                    'fixed_delta_seconds': 0.05,
                    'register_all_sensors': True,
                    'town': self.cfgs.carla_map,
                    'ego_vehicle': {
                        'role_name': ["hero", "ego_vehicle"]
                    }
                }
            }
            self.carla_bridge.ego_initial_pos = ego_spawn_point
            self.carla_bridge.initialize_bridge(
                self.carla_world, parameters, logger)
        except (IOError, RuntimeError) as e:
            logger.error(f"[Bridge] Error: {e}")
        except KeyboardInterrupt as e:      # if keyboard signal is catched, this try should be deleted
            logger.error(f"[Bridge] Error: {e}")
        except Exception as e:  # pylint: disable=W0718
            logger.error(e)
        finally:
            if self.carla_client:
                self.close()

    def init_environment(self) -> bool:
        self.connect_carla()
        if self.cfgs.load_bridge:
            logger.info("Loading bridge")
            sp_dic = None
            if self.ego_spawn_loc:
                sp_wp = self.carla_map.get_waypoint(carla.Location(x=self.ego_spawn_loc['x'],
                                                                   y=-self.ego_spawn_loc['y'],
                                                                   z=self.ego_spawn_loc['z']))
                sp_tf = sp_wp.transform
                sp_pose = carla_transform_to_cyber_pose(sp_tf)
                sp_dic = {"x": sp_pose.position.x,
                          "y": sp_pose.position.y,
                          "z": sp_pose.position.z + 2,
                          "roll": sp_tf.rotation.roll,
                          "pitch": sp_tf.rotation.pitch,
                          "yaw": -sp_tf.rotation.yaw}

            self.load_carla_bridge(ego_spawn_point=sp_dic)
            time.sleep(2)
            retry_times = 0
            self.ego_vehicle = None
            while True:
                print("[*] Waiting for carla_bridge "
                      + "." * retry_times + "\r", end="")
                vehicles = self.carla_world.get_actors().filter("*vehicle.*")
                for vehicle in vehicles:
                    if vehicle.attributes["role_name"] == "ego_vehicle":
                        self.ego_vehicle = vehicle
                        break
                if self.ego_vehicle:
                    break
                if retry_times > 5:
                    print("\n check if the carla_bridge is loaded properly")
                    self.close()
                    return False
                retry_times += 1
                time.sleep(2)
        return True

    def freeze_and_set_green_all_tls(self):
        traffic_lights = self.carla_world.get_actors().filter('traffic.traffic_light')
        for tl in traffic_lights:
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)
        logger.info('[Simulator] freeze and set green all tls')

    def wait_until_vehicle_moving(self, timeout=5.0):
        to = timeout
        while to > 0:
            curr_speed = self.ego_vehicle.get_velocity().x
            print(f'curr speed : {int(curr_speed*100)/100}\r', end='')
            if curr_speed > 0.01 or curr_speed < -0.01:
                return True
            time.sleep(0.1)
            to -= 0.1
        return False
        
    def check_if_ego_close_dest(self, radius=5):
        if not self.ego_vehicle:
            return False
        dis = self.ego_vehicle.get_location().distance(
            self.destination.location)
        print(f'distance to destination: {int(dis)} \r', end='')
        if dis < radius:
            return True
        else:
            return False
    
    def reload_local(self, dv: dreamview.Connection,
                     ego_initial_tf: carla.Transform = None,
                     dest=None):
        dv.disable_apollo()
        time.sleep(5)
        sp = random.choice(
                self.carla_world.get_map().get_spawn_points())
        if ego_initial_tf:
            sp = ego_initial_tf

        self.ego_vehicle.set_transform(sp)       
        
        time.sleep(5)
        dest_tf = self.select_valid_dest()
        if dest:
            dest_tf = dest
        self.destination = dest_tf
        dv.enable_apollo(self.destination, self.modules)
        dv.set_destination_tranform(self.destination)
        time.sleep(3)
        dv.enable_apollo(self.destination, self.modules)
        dv.set_destination_tranform(self.destination)
        time.sleep(3)
        dv.enable_apollo(self.destination, self.modules)
        dv.set_destination_tranform(self.destination)
        time.sleep(3)
        
    def reload(self, ego_initial_tf: carla.Transform = None):
        if self.carla_client:
            if self.carla_bridge.shutdown:
                self.carla_bridge.shutdown.set()
            self.carla_bridge.destroy()
            self.carla_bridge = None
            self.ego_vehicle = None
        self.carla_bridge = CarlaCyberBridge()
        if ego_initial_tf:
            sp_wp = self.carla_map.get_waypoint(ego_initial_tf.location)
            sp_tf = sp_wp.transform
            sp_pose = carla_transform_to_cyber_pose(sp_tf)
            sp_dic = {"x": sp_pose.position.x,
                      "y": sp_pose.position.y,
                      "z": sp_pose.position.z + 2,
                      "roll": sp_tf.rotation.roll,
                      "pitch": sp_tf.rotation.pitch,
                      "yaw": -sp_tf.rotation.yaw}

        self.load_carla_bridge(ego_spawn_point=sp_dic)
        time.sleep(2)
        retry_times = 0
        self.ego_vehicle = None
        while True:
            print("[*] Waiting for carla_bridge "
                  + "." * retry_times + "\r", end="")
            vehicles = self.carla_world.get_actors().filter("*vehicle.*")
            for vehicle in vehicles:
                if vehicle.attributes["role_name"] == "ego_vehicle":
                    self.ego_vehicle = vehicle
                    break
            if self.ego_vehicle:
                break
            if retry_times > 5:
                print("\n check if the carla_bridge is loaded properly")
                self.close()
                return False
            retry_times += 1
            time.sleep(2)
    def initialization(self):
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        logger.info(
            '[Simulator] === Simulation Start:  [' + date_time + '] ===')

        self.simulation_count += 1
        # time.sleep(1)

        if not self.init_environment():
            return

        self.freeze_and_set_green_all_tls()

        # self.load_random_scenario()

        times = 0
        success = False
        dv: dreamview.Connection = None
        dv = dreamview.Connection(
            self.ego_vehicle,
            ip=self.cfgs.dreamview_ip,
            port=str(self.cfgs.dreamview_port))
        dv.disable_apollo()
        while times < 3:
            try:
                time.sleep(2)
                dv.set_hd_map(self.cfgs.dreamview_map)
                dv.set_vehicle(self.cfgs.dreamview_vehicle)
                dv.set_setup_mode('Mkz Standard Debug')
                self.destination = self.select_valid_dest(20)
                dv.enable_apollo(self.destination, self.modules)
                dv.set_destination_tranform(self.destination)
                success = True
                break
            except:
                logger.warning(
                    '[Simulator] Fail to spin up apollo, try again!')
                times += 1
        if not success:
            raise RuntimeError('Fail to spin up apollo')
        logger.info(
            '[Simulator] Set Apollo (EGO) destination: '
            + str(self.destination.location.x)
            + ',' + str(self.destination.location.y))
        logger.info('[Simulator] Waiting for the vehicle to move')
        
        for i in range(10): 
            dv.set_destination_tranform(self.destination)
            if self.wait_until_vehicle_moving(3):
                logger.info('[Simulator] Vehicle is started')
                break
       

        self.load_random_scenario()

        dv.disconnect()

    def main_loop(self):
        dv: dreamview.Connection = dreamview.Connection(
            self.ego_vehicle,
            ip=self.cfgs.dreamview_ip,
            port=str(self.cfgs.dreamview_port))
        self.sim_status = True
        while self.sim_status:
            if self.check_if_ego_close_dest(10):
                if not self.ego_vehicle.get_control().brake > 0.5:
                    self.carla_world.wait_for_tick()
                    self.random_scenario.npc_refresh()
                    continue
                print('\r')
                print('reach des')
                # dv.disable_apollo()
                # sp = random.choice(
                #     self.carla_world.get_map().get_spawn_points())
                # time.sleep(10)

                # self.ego_vehicle.set_transform(sp)
                
                self.destination = self.select_valid_dest()
                
                time.sleep(5)
                dv.enable_apollo(self.destination, self.modules)
                for i in range(3):                     
                    logger.info('[Simulator] Vehicle is started')
                    dv.set_destination_tranform(self.destination)
                    if self.wait_until_vehicle_moving(2):
                        break
               

            self.carla_world.wait_for_tick()
            self.random_scenario.npc_refresh()
        dv.disconnect()

    def load_random_scenario(self):
        self.random_scenario = LocalScenario(self.carla_world,
                                             self.ego_vehicle,
                                             logger=logger)

        spawn_points = self.carla_world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        # Add vehicles
        # --------------
        for i in range(self.max_v_num):
            spawn_point = random.choice(spawn_points)
            end_point = random.choice(spawn_points)
            sp_dic = {'x': spawn_point.location.x,
                      'y': spawn_point.location.y,
                      'z': spawn_point.location.z}
            ep_dic = {'x': end_point.location.x,
                      'y': end_point.location.y,
                      'z': end_point.location.z}
            self.random_scenario.add_npc_vehicle(
                sp_dic, ep_dic, 0, 0, 0, free_roam=True)

        # -------------
        # Add Walkers
        # -------------
        for i in range(self.max_w_num):
            spawn_point = carla.Transform()
            loc = self.carla_world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
            end_point = carla.Transform()
            loc = self.carla_world.get_random_location_from_navigation()
            if (loc != None):
                end_point.location = loc
            sp_dic = {'x': spawn_point.location.x,
                      'y': spawn_point.location.y,
                      'z': spawn_point.location.z}
            ep_dic = {'x': end_point.location.x,
                      'y': end_point.location.y,
                      'z': end_point.location.z}
            self.random_scenario.add_npc_walker(sp_dic, ep_dic, 0, 0, 1.4)

        self.random_scenario.spawn_all_npcs()
        self.random_scenario.scenario_start()

    

    def close(self):
        logger.warning("Shutting down.")
        self.tm_close_event.set()
        if self.tm_thread:
            self.tm_thread.join()

        if self.sim_status:
            self.sim_status = False

        if self.random_scenario:
            self.random_scenario.scenario_end()

        if self.cfgs.load_bridge:
            if self.carla_client:
                self.carla_world = self.carla_client.get_world()
                settings = self.carla_world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.carla_world.apply_settings(settings)
            if self.carla_bridge.shutdown:
                self.carla_bridge.shutdown.set()
            self.carla_bridge.destroy()
            logger.warning("[Shutdown] Brigde destroied")

        if self.carla_world:
            del self.carla_world
            self.carla_world = None
            logger.warning("[Shutdown] Carla world destroied")
        if self.carla_client:
            del self.carla_client
            self.carla_client = None
            logger.warning("[Shutdown] Carla client destroied")


if __name__ == '__main__':
    cfg = Config()
    sim = RandomScenario(cfg)
    sim.initialization()
    sim.main_loop()
    sim.close()
    pdb.set_trace()
