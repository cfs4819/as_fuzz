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
from datetime import datetime, timedelta


import carla
from carla import VehicleLightState as vls

import logging
from loguru import logger
from numpy import random
import threading
from typing import List
import json

from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
from carla_bridge.dreamview_carla import dreamview
from carla_bridge.utils.transforms import carla_transform_to_cyber_pose
from MS_fuzz.fuzz_config.Config import Config
from MS_fuzz.common.camera_agent_imageio import ScenarioRecorder
from MS_fuzz.common.scenario import LocalScenario
from MS_fuzz.common.unsafe_detector import *
from MS_fuzz.ms_utils import predict_collision
import pdb


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

        self.carla_bridge = CarlaCyberBridge()

        self.dv: dreamview.Connection = None

        self.max_v_num = 15
        self.max_w_num = 12

        self.tm_thread: threading.Thread = None
        self.tm_close_event: threading.Event = threading.Event()

        self.close_event = threading.Event()

        self.recorder: ScenarioRecorder = None

        self.unsafe_detector: UnsafeDetector = None

        self.destination = None

        self.random_scenario: LocalScenario = None

        self.result_path = '/apollo/data/random_scenario/result'
        self.sim_result_path = ''
        self.sce_result_path = ''

        self.result_to_save = {}
        self.clear_result()

        self.frames_record = []

        self.sce_index = 0

        self.is_recording = False
        self.recorder_start_time = time.time()

        self.on_unsafe_lock = False

        self.max_reload = 20
        self.curr_reload = 0

    def select_valid_dest(self, radius=100) -> carla.Transform:
        '''
            Select a destination outside the specified radius from current position
        '''
        ego_curr_point = self.ego_vehicle.get_transform()
        valid_destination = False
        sps = self.carla_map.get_spawn_points()
        while not valid_destination:
            des_transform = random.choice(sps)
            des_wp = self.carla_map.get_waypoint(des_transform.location,
                                                 project_to_road=False)
            distance = ego_curr_point.location.distance(des_transform.location)
            if distance < radius:
                continue
            if des_wp.is_junction:
                continue
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
            self.close()
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
                exit()

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

        self.dv = dreamview.Connection(
            self.ego_vehicle,
            ip=self.cfgs.dreamview_ip,
            port=str(self.cfgs.dreamview_port))
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

    def reload_local(self,
                     ego_initial_tf: carla.Transform = None,
                     dest=None):
        self.curr_reload += 1
        if self.curr_reload >= self.max_reload:
            # give up, exit()
            logger.error('[Exit] Too Much Reload, Give up Simulation')
            self.close()
            exit()
        self.dv.disable_apollo()
        ego_ctrl = self.ego_vehicle.get_control()
        ego_ctrl.throttle = 0
        ego_ctrl.steer = 0
        ego_ctrl.brake = 0.7
        ego_ctrl.hand_brake = True
        self.ego_vehicle.apply_control(ego_ctrl)

        wait_time = 5.0
        while wait_time > 0:
            self.ego_vehicle.apply_control(ego_ctrl)
            time.sleep(0.01)
            wait_time -= 0.01

        sp = random.choice(
            self.carla_world.get_map().get_spawn_points())
        if ego_initial_tf:
            sp = ego_initial_tf

        self.ego_vehicle.set_transform(sp)

        wait_time = 3.0
        while wait_time > 0:
            self.ego_vehicle.apply_control(ego_ctrl)
            time.sleep(0.1)
            wait_time -= 0.1

        dest_tf = self.select_valid_dest()
        if dest:
            dest_tf = dest
        self.destination = dest_tf

        self.dv.enable_apollo(self.destination, self.modules)
        self.dv.set_destination_tranform(self.destination)
        time.sleep(3)

        ego_ctrl = self.ego_vehicle.get_control()
        ego_ctrl.throttle = 0
        ego_ctrl.steer = 0
        ego_ctrl.brake = 0.7
        ego_ctrl.hand_brake = False
        self.ego_vehicle.apply_control(ego_ctrl)

        self.dv.enable_apollo(self.destination, self.modules)
        self.dv.set_destination_tranform(self.destination)
        if self.dv.check_module_status(['Prediction', 'Planning']) and\
            self.wait_until_vehicle_moving(3):
            return
        self.dv.enable_apollo(self.destination, self.modules)
        self.dv.set_destination_tranform(self.destination)
        time.sleep(3)

    def reload(self, ego_initial_tf: carla.Transform = None):
        if self.recorder:
            self.recorder.stop_recording()

        if self.unsafe_detector:
            self.unsafe_detector.cleanup()

        if self.sim_status:
            self.sim_status = False

        if self.random_scenario:
            self.random_scenario.scenario_end()

        logger.warning("[Shutdown] Brigde destroied")
        if self.carla_bridge.shutdown:
            self.carla_bridge.shutdown.set()
        self.carla_bridge.destroy()
        self.carla_bridge = None
        self.ego_vehicle = None

        if self.dv:
            self.dv.disconnect()
            self.dv = None
        self.carla_bridge = CarlaCyberBridge()

        if not self.init_environment():
            return
        self.freeze_and_set_green_all_tls()

        self.load_random_scenario()
        self.recorder = ScenarioRecorder(self.carla_world,
                                         self.ego_vehicle,
                                         self.result_path)
        self.unsafe_detector = UnsafeDetector(self.carla_world,
                                              self.ego_vehicle)
        self.unsafe_detector.register_callback(self.on_unsafe)

        self.reload_local()
        self.unsafe_detector.init_sensors()
        self.unsafe_detector.start_detection()

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
        self.load_random_scenario()

        times = 0
        success = False
        self.dv.disable_apollo()
        self.destination = self.select_valid_dest()
        while times < 3:
            try:
                time.sleep(2)
                self.dv.set_hd_map(self.cfgs.dreamview_map)
                self.dv.set_vehicle(self.cfgs.dreamview_vehicle)
                self.dv.set_setup_mode('Mkz Standard Debug')

                self.result_to_save['end_loc'] = {
                    'x': self.destination.location.x,
                    'y': self.destination.location.y,
                    'z': self.destination.location.z
                }

                self.dv.enable_apollo(self.destination, self.modules)
                self.dv.set_destination_tranform(self.destination)
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
            self.dv.set_destination_tranform(self.destination)
            if self.wait_until_vehicle_moving(3):
                logger.info('[Simulator] Vehicle is started')
                break

        curr_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.sim_result_path = os.path.join(self.result_path, curr_datetime)

        if not os.path.exists(self.sim_result_path):
            os.makedirs(self.sim_result_path)
        self.recorder = ScenarioRecorder(self.carla_world,
                                         self.ego_vehicle,
                                         self.result_path)
        self.unsafe_detector = UnsafeDetector(self.carla_world,
                                              self.ego_vehicle)

        self.unsafe_detector.register_callback(self.on_unsafe)
        self.unsafe_detector.init_sensors()
        self.unsafe_detector.start_detection()

    def on_unsafe(self, type, message, data=None):
        # print(type, message)
        if self.on_unsafe_lock:
            return
        self.on_unsafe_lock = True

        triger_time = time.time()
        time_pass = triger_time - self.result_to_save['start_time']
        time_pass_str = str(timedelta(seconds=time_pass))
        if time_pass < 5:
            self.on_unsafe_lock = False
            return

        if type == UNSAFE_TYPE.ACCELERATION:
            if self.result_to_save['minor_unsafe'] and \
                    'unsafe_acc' in self.result_to_save['minor_unsafe']:
                if data == self.result_to_save['minor_unsafe']['unsafe_acc'][-1]['acc']:
                    self.on_unsafe_lock = False
                    return
                self.result_to_save['minor_unsafe']['unsafe_acc'].append({
                    'time': time_pass,
                    'time_str': time_pass_str,
                    'acc': data
                })
            elif self.result_to_save['minor_unsafe'] and \
                    'unsafe_acc' not in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['unsafe_acc'] = [{
                    'time': time_pass,
                    'time_str': time_pass_str,
                    'acc': data
                }]
            else:
                self.result_to_save['minor_unsafe'] = {
                    'unsafe_acc': [{
                        'time': time_pass,
                        'time_str': time_pass_str,
                        'acc': data
                    }]
                }
            self.on_unsafe_lock = False
            return

        if type == UNSAFE_TYPE.LANE_CHANGE:
            if self.result_to_save['minor_unsafe'] and \
                    'lanechange_timeout' in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['lanechange_timeout'].append({
                    'time': time_pass,
                    'time_str': time_pass_str
                })
            elif self.result_to_save['minor_unsafe'] and \
                    'lanechange_timeout' not in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['lanechange_timeout'] = [{
                    'time': time_pass,
                    'time_str': time_pass_str
                }]
            else:
                self.result_to_save['minor_unsafe'] = {
                    'lanechange_timeout': [{
                        'time': time_pass,
                        'time_str': time_pass_str
                    }]
                }
            self.on_unsafe_lock = False
            return

        if type == UNSAFE_TYPE.CROSSING_SOLID_LANE:
            if data == 1:
                if self.result_to_save['minor_unsafe'] and \
                        'crossing_solid_lane' in self.result_to_save['minor_unsafe']:
                    self.result_to_save['minor_unsafe']['crossing_solid_lane'].append({
                        'time': time_pass,
                        'time_str': time_pass_str
                    })
                elif self.result_to_save['minor_unsafe'] and \
                        'crossing_solid_lane' not in self.result_to_save['minor_unsafe']:
                    self.result_to_save['minor_unsafe']['crossing_solid_lane'] = [{
                        'time': time_pass,
                        'time_str': time_pass_str
                    }]
                else:
                    self.result_to_save['minor_unsafe'] = {
                        'crossing_solid_lane': [{
                            'time': time_pass,
                            'time_str': time_pass_str
                        }]
                    }
                self.on_unsafe_lock = False
                return

        self.result_to_save['unsafe'] = True
        self.result_to_save['unsafe_type'] = UNSAFE_TYPE.type_str[type]
        logger.info('reload')
        
        if type not in [UNSAFE_TYPE.COLLISION, UNSAFE_TYPE.STUCK]:
            self.save_result(save_video=True)
        else:
            self.save_result(save_video=False)

        near_by_tf = self.carla_map.get_waypoint(
            self.ego_vehicle.get_location()).transform

        if type == UNSAFE_TYPE.STUCK:
            # stucked, need to move to another place
            near_by_tf = None

        if type in [UNSAFE_TYPE.COLLISION, UNSAFE_TYPE.STUCK]:
            # collision, or stuck, need to clear others
            self.random_scenario.scenario_end()
            self.random_scenario = None
            self.carla_world.wait_for_tick()
            self.load_random_scenario()

        self.reload_local(ego_initial_tf=near_by_tf)
        self.sce_index += 1
        self.start_record()

        self.on_unsafe_lock = False

    def start_record(self):
        self.clear_result()
        curr_loc = self.ego_vehicle.get_location()
        self.result_to_save['start_loc'] = {
            'x': curr_loc.x,
            'y': curr_loc.y,
            'z': curr_loc.z
        }
        self.result_to_save['dest_loc'] = {
            'x': self.destination.location.x,
            'y': self.destination.location.y,
            'z': self.destination.location.z
        }

        self.result_to_save['start_time'] = time.time()

        self.frames_record = []

        sce_result_path = os.path.join(
            self.sim_result_path, f'Scenario_{self.sce_index}')

        if not os.path.exists(sce_result_path):
            os.makedirs(sce_result_path)

        self.sce_result_path = sce_result_path

        sce_video_path = os.path.join(sce_result_path, 'recorder.mp4')
        self.result_to_save['video_path'] = sce_video_path

        self.recorder.start_recording(save_path=sce_video_path)

        self.is_recording = True
        self.recorder_start_time = time.time()

    def save_result(self, save_video=True):
        if self.is_recording:
            self.is_recording = False
        self.recorder.stop_recording()
        self.result_to_save['end_loc'] = {
            'x': self.ego_vehicle.get_location().x,
            'y': self.ego_vehicle.get_location().y,
            'z': self.ego_vehicle.get_location().z
        }
        now = time.time()

        self.result_to_save['end_time'] = now
        self.result_to_save['run_time'] = now - \
            self.result_to_save['start_time']

        if not self.frames_record or len(self.frames_record) == 0:
            self.result_to_save['interaction'] = None
            self.result_to_save['Odometer'] = None
        else:
            ego_pos_l = []
            run_distance = 0.0
            interaction_per_frame = []
            total_frame_num = len(self.frames_record)

            will_collide_frame_cnt = 0

            pre_frame = None

            for frame in self.frames_record:
                ego_ss = frame['ego_ss']
                npcs_ss = frame['npc_vehicles_ss']
                ego_pos_l.append({
                    'timestamp': frame['timestamp'],
                    'frame_num': frame['frame'],
                    'x': ego_ss.get_transform().location.x,
                    'y': ego_ss.get_transform().location.y,
                    'z': ego_ss.get_transform().location.z,
                    'yaw': ego_ss.get_transform().rotation.yaw,
                    'pitch': ego_ss.get_transform().rotation.pitch,
                    'roll': ego_ss.get_transform().rotation.roll,
                    'speed': ego_ss.get_velocity().length(),
                    'run_distance': round(run_distance, 2)
                })

                if pre_frame:
                    pre_ego_ss = pre_frame['ego_ss']
                    delta_dis = abs(ego_ss.get_transform().location.distance(
                        pre_ego_ss.get_transform().location))
                    run_distance += delta_dis

                pre_colli_cunt = 0
                this_frame_has_colli = False
                for npc_ss in npcs_ss:
                    will_collide, t = predict_collision(ego_ss, npc_ss)
                    pre_colli_cunt += 1 if will_collide else 0
                    if will_collide:
                        this_frame_has_colli = True

                will_collide_frame_cnt += 1 if this_frame_has_colli else 0
                will_collide_rate = pre_colli_cunt / \
                    len(npcs_ss) if npcs_ss else 0

                interaction_per_frame.append({
                    'timestamp': frame['timestamp'],
                    'frame_num': frame['frame'],
                    'interaction_rate': will_collide_rate
                })

                pre_frame = frame

            self.result_to_save['Odometer'] = {
                'total_distance': round(run_distance, 2),
                'pos_per_frame': ego_pos_l
            }
            self.result_to_save['interaction'] = {
                'interact_frame_rate': will_collide_frame_cnt / total_frame_num,
                'per_frame': interaction_per_frame
            }

        try:
            resule_str = json.dumps(self.result_to_save, indent=4)
            result_path = os.path.join(self.sce_result_path, 'result.json')
            with open(result_path, 'w') as f:
                f.write(resule_str)
            if not save_video:
                if os.path.isfile(self.result_to_save['video_path']):
                    os.remove(self.result_to_save['video_path'])
                    self.result_to_save['video_path'] = 'deleted'
        except Exception as e:
            print(e)

    def clear_result(self):
        self.result_to_save = {}
        self.result_to_save = {
            'unsafe': False,
            'unsafe_type': None,
            'minor_unsafe': [],
            'start_loc': None,
            'dest_loc': None,
            'end_loc': None,
            'start_time': time.time(),
            'end_time': None,
            'run_time': None,
            'interaction': None,
            'video_path': None,
            'Odometer': None,
        }

    def main_loop(self):
        self.sim_status = True

        self.start_record()

        while self.sim_status:
            if self.check_if_ego_close_dest(10):
                if not self.ego_vehicle.get_control().brake > 0.5:
                    world_ss = self.carla_world.wait_for_tick()
                    if not self.sim_status:
                        break
                    if not self.loading_random_scenario and self.random_scenario:
                        eval_frame = self.random_scenario.evaluate_snapshot_record(
                            world_ss)
                        self.frames_record.append(eval_frame)
                        self.random_scenario.npc_refresh()
                    continue
                print('\r')
                print('reach des')

                # Sce end
                self.save_result()

                # Next sce
                self.destination = self.select_valid_dest()

                time.sleep(5)
                if not self.sim_status:
                    break
                self.dv.enable_apollo(self.destination, self.modules)
                for i in range(3):
                    self.dv.set_destination_tranform(self.destination)
                    if self.wait_until_vehicle_moving(2):
                        logger.info('[Simulator] Vehicle is started')
                        break
                self.sce_index += 1
                self.start_record()

            world_ss = self.carla_world.wait_for_tick()
            if not self.sim_status:
                break
            if not self.loading_random_scenario and self.random_scenario:
                eval_frame = self.random_scenario.evaluate_snapshot_record(
                    world_ss)
                self.frames_record.append(eval_frame)
                self.random_scenario.npc_refresh()

    def load_random_scenario(self):
        self.loading_random_scenario = True
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

        self.loading_random_scenario = False

    def close(self):
        logger.warning("Shutting down.")

        if self.recorder:
            self.recorder.stop_recording()
            self.recorder = None

        if self.unsafe_detector:
            self.unsafe_detector.cleanup()
            self.unsafe_detector = None

        if self.sim_status:
            self.sim_status = False

        if self.random_scenario:
            self.random_scenario.scenario_end()
            self.random_scenario = None

        if self.cfgs.load_bridge:
            if self.carla_client:
                self.carla_world = self.carla_client.get_world()
                settings = self.carla_world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.carla_world.apply_settings(settings)
            if self.carla_bridge:
                if self.carla_bridge.shutdown:
                    self.carla_bridge.shutdown.set()
                self.carla_bridge.destroy()
            self.carla_bridge = None
            logger.warning("[Shutdown] Brigde destroied")

        if self.dv:
            self.dv.disconnect()
            self.dv = None
            logger.warning("[Shutdown] Disconnected from Dreamview")

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
    # pdb.set_trace()
