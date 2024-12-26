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
import argparse
import signal
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
from MS_fuzz.common.result_saver import ResultSaver
# from MS_fuzz.ms_utils import predict_collision

import pdb


class SimulationTimeoutTimer:
  def __init__(self, timeout, callback):
    self.timeout = timeout
    self.callback = callback
    self.timer = None
    self.lock = threading.Lock()

  def _run(self):
    with self.lock:
      self.timer = None
      self.callback()

  def start(self):
    with self.lock:
      if self.timer is not None:
        self.timer.cancel()
      self.timer = threading.Timer(self.timeout, self._run)
      self.timer.start()

  def reset(self):
    self.start()

  def cancel(self):
    with self.lock:
      if self.timer is not None:
        self.timer.cancel()
        self.timer = None


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
      # 'Storytelling'  # ok
    ]

    self.carla_bridge = CarlaCyberBridge()

    self.dv: dreamview.Connection = None

    self.max_v_num = random.randint(0, 15)
    self.max_w_num = random.randint(0, 12)

    self.tm_thread: threading.Thread = None
    self.tm_close_event: threading.Event = threading.Event()

    self.close_event = threading.Event()

    self.recorder: ScenarioRecorder = None

    self.unsafe_detector: UnsafeDetector = None

    self.destination = None

    self.random_scenario: LocalScenario = None
    self.loading_random_scenario = False

    self.result_path = '/apollo/data/random_scenario/result'
    self.sim_result_path = ''

    self.result_saver = ResultSaver()

    self.sce_index = 0

    self.is_recording = False
    self.recorder_start_time = time.time()

    self.on_unsafe_lock = False

    self.max_reload = 20
    self.curr_reload = 0

    self.resent_dest = 0

    # Set a 10 minute timeout
    self.simulation_timeout = SimulationTimeoutTimer(10 * 60,
                                                     self.simulation_timeout_callback)

  def simulation_timeout_callback(self):

    self.close()
    sys.exit()

  def select_valid_dest(self, min_radius=100, max_radius=150) -> carla.Transform:
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
      if distance < min_radius or distance > max_radius:
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
    except KeyboardInterrupt as e:  # if keyboard signal is catched, this try should be deleted
      logger.error(f"[Bridge] Error: {e}")
    except Exception as e:  # pylint: disable=W0718
      logger.error(e)
    finally:
      if self.carla_client:
        self.close()
        sys.exit()

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
    self.dv.disable_apollo()
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
      print(f'curr speed : {int(curr_speed * 100) / 100}\r', end='')
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

    times = 0
    success = False
    # self.dv.disable_apollo()
    self.destination = self.select_valid_dest(50, 100)
    while times < 3:
      try:
        time.sleep(2)
        self.dv.set_hd_map(self.cfgs.dreamview_map)
        self.dv.set_vehicle(self.cfgs.dreamview_vehicle)
        self.dv.set_setup_mode('Mkz Standard Debug')

        self.result_saver.result_to_save['end_loc'] = {
          'x': self.destination.location.x,
          'y': self.destination.location.y,
          'z': self.destination.location.z
        }

        self.dv.enable_apollo(self.destination, self.modules)
        self.dv.set_destination_tranform(self.destination)
        success = True
        times += 1
      except:
        logger.warning(
          '[Simulator] Fail to spin up apollo, try again!')
        break
    if not success:
      raise RuntimeError('Fail to spin up apollo')
    logger.info(
      '[Simulator] Set Apollo (EGO) destination: '
      + str(self.destination.location.x)
      + ',' + str(self.destination.location.y))
    logger.info('[Simulator] Waiting for the vehicle to move')

    running = False
    for i in range(10):
      logger.info(f'[Simulator] Trying to set destination:{i}')
      self.dv.set_destination_tranform(self.destination)
      if self.wait_until_vehicle_moving(3):
        logger.info('[Simulator] Vehicle is started')
        running = True
        break
    if not running:
      logger.warning('[Simulator] Vehicle is not started')
      self.close()

    self.load_random_scenario()

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

    trigger_time = time.time()
    time_pass = trigger_time - self.result_saver.result_to_save['start_time']
    # time_pass_str = str(timedelta(seconds=time_pass))
    if time_pass < 5:
      self.on_unsafe_lock = False
      return

    if type in [UNSAFE_TYPE.ACCELERATION,
                UNSAFE_TYPE.LANE_CHANGE,
                UNSAFE_TYPE.CROSSING_SOLID_LANE]:
      self.result_saver.add_minor_unsafe(type, data, trigger_time)
      self.on_unsafe_lock = False
      return

    self.result_saver.result_to_save['unsafe'] = True
    self.result_saver.result_to_save['unsafe_type'] = UNSAFE_TYPE.type_str[type]
    logger.info('reload')

    self.stop_record_and_save(save_video=True)

    near_by_tf = self.carla_map.get_waypoint(
      self.ego_vehicle.get_location()).transform

    if type == UNSAFE_TYPE.STUCK and self.resent_dest < 3:
      self.resent_dest += 1
      if self.dv:
        self.dv.set_destination_tranform(self.destination)
        if self.wait_until_vehicle_moving(3):
          self.resent_dest = 0
      return

    if type in [UNSAFE_TYPE.COLLISION, UNSAFE_TYPE.STUCK]:
      # collision, or stuck, need to clear others
      self.loading_random_scenario = True
      self.random_scenario.scenario_end()
      self.random_scenario = None
      self.carla_world.wait_for_tick()
      # self.load_random_scenario()

      self.close()
      sys.exit()
    # self.reload_local(ego_initial_tf=near_by_tf)
    # self.sce_index += 1
    # self.start_record()

    self.on_unsafe_lock = False

  def start_record(self):
    sce_result_path = os.path.join(
      self.sim_result_path, f'Scenario_{self.sce_index}')

    if not os.path.exists(sce_result_path):
      os.makedirs(sce_result_path)

    self.sce_result_path = sce_result_path

    sce_video_path = os.path.join(sce_result_path, 'recorder.mp4')

    self.result_saver.set_save_path(sce_result_path)

    self.result_saver.clear_result()

    self.result_saver.result_to_save['video_path'] = sce_video_path

    curr_loc = self.ego_vehicle.get_location()
    self.result_saver.result_to_save['start_loc'] = {
      'x': curr_loc.x,
      'y': curr_loc.y,
      'z': curr_loc.z
    }
    self.result_saver.result_to_save['dest_loc'] = {
      'x': self.destination.location.x,
      'y': self.destination.location.y,
      'z': self.destination.location.z
    }

    self.result_saver.result_to_save['start_time'] = time.time()

    self.recorder.start_recording(save_path=sce_video_path)

    self.is_recording = True
    self.recorder_start_time = time.time()

  def stop_record_and_save(self, save_video=True):
    if save_video:
      time.sleep(1)
    self.recorder.stop_recording()
    curr_loc = self.ego_vehicle.get_location()
    self.result_saver.save_result(curr_loc, save_video)

  def main_loop(self):
    self.sim_status = True

    self.start_record()
    self.simulation_timeout.start()
    while self.sim_status:
      if self.check_if_ego_close_dest(10):
        if not self.ego_vehicle.get_control().brake > 0.65:
          world_ss = self.carla_world.wait_for_tick()
          if not self.sim_status:
            break
          if not self.loading_random_scenario and self.random_scenario:
            eval_frame = self.random_scenario.evaluate_snapshot_record(
              world_ss)
            self.result_saver.frames_record.append(eval_frame)
            # self.random_scenario.npc_refresh()
          continue
        print('\r')
        print('reach des')
        # Sce end
        self.stop_record_and_save(save_video=False)

        # Next sce
        self.destination = self.select_valid_dest(50, 100)

        time.sleep(5)
        if not self.sim_status:
          break
        self.dv.enable_apollo(self.destination, self.modules)
        success = False
        for i in range(5):
          self.dv.set_destination_tranform(self.destination)
          if self.wait_until_vehicle_moving(10):
            logger.info('[Simulator] Vehicle is started')
            success = True
            break
        if not success:
          break
        self.sce_index += 1
        self.start_record()
        self.simulation_timeout.reset()

      world_ss = self.carla_world.wait_for_tick()
      if not self.sim_status:
        break
      if not self.random_scenario:
        print('scenario not detected')
      if not self.loading_random_scenario and self.random_scenario:
        eval_frame = self.random_scenario.evaluate_snapshot_record(
          world_ss)
        self.result_saver.frames_record.append(eval_frame)
        # self.random_scenario.npc_refresh()

  def load_random_scenario(self):
    self.loading_random_scenario = True
    self.random_scenario = LocalScenario(self.carla_world,
                                         self.ego_vehicle,
                                         logger=logger, carla_map=self.carla_map)

    spawn_points = self.carla_world.get_map().get_spawn_points()

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
      self.random_scenario.add_npc_walker(sp_dic, ep_dic, 0, 0)

    self.random_scenario.spawn_all_npcs()

    for w in self.random_scenario.npc_walker_list:
      if w.ai_controller:
        w.ai_controller.start()

    for v in self.random_scenario.npc_vehicle_list:
      if v.vehicle:
        v.vehicle.set_autopilot(True)

    self.loading_random_scenario = False

  def close(self):
    logger.warning("Shutting down.")

    def exit_program():
      logger.warning("Force Exiting program due to timeout.")
      os.kill(os.getpid(), signal.SIGTERM)

    close_timeout_timer = SimulationTimeoutTimer(10, exit_program)

    if self.sim_status:
      self.sim_status = False

    close_timeout_timer.start()

    if self.dv:
      self.dv.disable_apollo()

    if self.recorder:
      logger.warning("[Shutdown] Recorder stopping")
      self.recorder.stop_recording()
      self.recorder = None
      logger.warning("[Shutdown] Recorder destroied")

    if self.unsafe_detector:
      logger.warning("[Shutdown] Unsafe detector stopping")
      self.unsafe_detector.cleanup()
      self.unsafe_detector = None
      logger.warning("[Shutdown] Unsafe detector destroied")

    if self.random_scenario:
      logger.warning("[Shutdown] Scenario stopping")
      # self.random_scenario.scenario_end()
      try:
        for vehicle in self.random_scenario.npc_vehicle_list:
          if vehicle.vehicle:
            vehicle.vehicle.destroy()
            vehicle.vehicle = None
          # self.npc_vehicle_list.remove(vehicle)
        self.random_scenario.npc_vehicle_list = []
        for walker in self.random_scenario.npc_walker_list:
          if walker.ai_controller:
            walker.ai_controller.destroy()
            walker.ai_controller = None
          if walker.walker:
            walker.walker.destroy()
            walker.walker = None
          # self.npc_walker_list.remove(walker)
        self.random_scenario.npc_walker_list = []
      except Exception as e:
        print(e)
      self.random_scenario = None

    if self.cfgs.load_bridge:
      if self.carla_client:
        self.carla_world = self.carla_client.get_world()
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla_world.apply_settings(settings)
      if self.carla_bridge:
        if self.carla_bridge.shutdown_event:
          self.carla_bridge.shutdown_event.set()
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
    close_timeout_timer.cancel()
    os.kill(os.getpid(), signal.SIGTERM)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description="Simulation configuration")
  parser.add_argument('-p', '--port', type=int, default=4000,
                      help='Set the simulation port (default: 4000)')
  parser.add_argument("--town", default=4, type=int,
                      help="Test on a specific town (e.g., '--town 3' forces Town03)")
  args = parser.parse_args()

  cfg = Config()
  cfg.sim_port = args.port

  town_index = args.town
  cfg.carla_map = cfg.town_name[str(town_index)]
  cfg.dreamview_map = cfg.dreamview_map_dic[cfg.carla_map]

  sim = RandomScenario(cfg)

  max_timer = SimulationTimeoutTimer(15 * 60, sim.close)
  max_timer.start()

  sim.initialization()
  sim.main_loop()
  sim.close()
