import os
import carla
import time
import numpy as np
import threading
import random
import math
# import pygame

from datetime import datetime
from loguru import logger

# from common import utils
# from common.frame import CaseRecorder, FrameElement, FrameEventType, CaseFaultType
from MS_fuzz.fuzz_config.Config import Config
from MS_fuzz.ms_utils import calc_relative_loc 
from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
from carla_bridge.utils.logurus import init_log
from carla_bridge.dreamview_carla import dreamview

from scenario import LocalScenario

import pdb

from agents.tools.misc import draw_waypoints


class Simulator(object):

    def __init__(self, cfgs: Config):
        self.carla_client = None
        self.carla_world = None
        self.carla_map = None
        self.ego_vehicle = None
        self.destination = None
        self.carla_bridge: CarlaCyberBridge = None
        self.carla_bridge_thread = None
        # self.async_flag = cfgs.sim_mode
        self.cfgs: Config = cfgs

        self.mutated_npc_list = []
        self.yellow_lines = []
        self.cross_lines = []
        self.edge_lines = []

        self.sim_status = False

        # self.connect_lgsvl()
        # self.load_map(self.lgsvl_map)

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

        self.local_scenario: LocalScenario = None

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
            self.carla_bridge.ego_spawn_point = ego_spawn_point
            self.carla_bridge.initialize_bridge(
                self.carla_world, parameters, logger)
        except (IOError, RuntimeError) as e:
            logger.error(f"[Bridge] Error: {e}")
        except KeyboardInterrupt as e:
            logger.error(f"[Bridge] Error: {e}")
        except Exception as e:  # pylint: disable=W0718
            logger.error(e)
        finally:
            if self.carla_client:
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

    def connect_carla(self):
        '''
        Connect to carla simualtor.
        '''
        if (self.carla_client != None and
                self.carla_world != None):
            logger.warning("Connection already exists")
            return
        try:
            logger.info(
                f"[Simulator] Connecting to Carla on {self.cfgs.sim_host} {self.cfgs.sim_port}")
            self.carla_client = carla.Client(
                host=self.cfgs.sim_host, port=self.cfgs.sim_port
            )
            self.carla_client.set_timeout(self.cfgs.load_world_timeout)
            if self.cfgs.load_bridge:
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

    def init_environment(self) -> bool:
        self.connect_carla()
        if self.cfgs.load_bridge:
            logger.info("Loading bridge")
            self.load_carla_bridge()
            time.sleep(2)
            retry_times = 0
            self.ego_vehicle = None
            while True:
                print("[*] Waiting for carla_bridge " +
                      "." * retry_times + "\r", end="")
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

    def run(self):

        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        logger.info(
            '[Simulator] === Simulation Start:  [' + date_time + '] ===')

        self.simulation_count += 1
        # time.sleep(1)

        if not self.init_environment():
            return

        # TODO: load initial senario
        self.local_scenario = LocalScenario(
            self.carla_world, self.ego_vehicle, logger)

        ego_tf = self.ego_vehicle.get_transform()

        carla_db = self.carla_world.debug
        carla_db.draw_arrow(begin=calc_relative_loc(ego_tf, 5, 0, 3),
                            end=calc_relative_loc(ego_tf, 7, 0, 3),
                            color=carla.Color(255, 0, 0))
        carla_db.draw_arrow(begin=calc_relative_loc(ego_tf, 5, 0, 3),
                            end=calc_relative_loc(ego_tf, 5, 2, 3),
                            color=carla.Color(0, 255, 0))

        self.local_scenario.add_npc_vehicle(start_loc={'x': calc_relative_loc(ego_tf, 5, -8).x,
                                                       'y': calc_relative_loc(ego_tf, 5, -8).y,
                                                       'z': 0},
                                            end_loc={'x': calc_relative_loc(ego_tf, 80, -8).x,
                                                     'y': calc_relative_loc(ego_tf, 80, -8).y,
                                                     'z': 0},
                                            start_time=5,
                                            behavior_type=0,
                                            agent_type='normal')
        self.local_scenario.add_npc_vehicle(start_loc={'x': calc_relative_loc(ego_tf, 12, 0).x,
                                                       'y': calc_relative_loc(ego_tf, 12, 0).y,
                                                       'z': 0},
                                            end_loc={'x': calc_relative_loc(ego_tf, 80, -1).x,
                                                     'y': calc_relative_loc(ego_tf, 80, -1).y,
                                                     'z': 0},
                                            start_time=6,
                                            behavior_type=0,
                                            agent_type='normal')
        self.local_scenario.add_npc_vehicle(start_loc={'x': calc_relative_loc(ego_tf, 22, -2).x,
                                                       'y': calc_relative_loc(ego_tf, 22, -2).y,
                                                       'z': 0},
                                            end_loc={'x': calc_relative_loc(ego_tf, 80, 1).x,
                                                     'y': calc_relative_loc(ego_tf, 80, 1).y,
                                                     'z': 0},
                                            start_time=5,
                                            behavior_type=0,
                                            agent_type='normal')

        self.local_scenario.add_npc_vehicle(start_loc={'x': calc_relative_loc(ego_tf, 8, 3).x,
                                                       'y': calc_relative_loc(ego_tf, 8, 3).y,
                                                       'z': 0},
                                            end_loc={'x': calc_relative_loc(ego_tf, 80, 1).x,
                                                     'y': calc_relative_loc(ego_tf, 80, 1).y,
                                                     'z': 0},
                                            start_time=4,
                                            behavior_type=1,
                                            agent_type='cautious')
        self.local_scenario.spawn_all_npcs()
        # pdb.set_trace()

        times = 0
        success = False
        dv: dreamview.Connection = None
        while times < 3:
            try:
                dv = dreamview.Connection(
                    self.ego_vehicle,
                    ip=self.cfgs.dreamview_ip,
                    port=str(self.cfgs.dreamview_port))
                time.sleep(2)
                dv.set_hd_map(self.cfgs.dreamview_map)
                dv.set_vehicle(self.cfgs.dreamview_vehicle)
                dv.set_setup_mode('Mkz Standard Debug')
                self.destination = self.select_valid_dest()
                dv.enable_apollo(self.destination, self.modules)
                success = True
                break
            except:
                logger.warning(
                    '[Simulator] Fail to spin up apollo, try again!')
                times += 1
        if not success:
            raise RuntimeError('Fail to spin up apollo')
        dv.set_destination_tranform(self.destination)
        logger.info(
            '[Simulator] Set Apollo (EGO) destination: ' +
            str(self.destination.location.x) +
            ',' + str(self.destination.location.y))

        self.sim_status = True
        synchronous_mode = self.carla_world.get_settings().synchronous_mode
        synchronous_mode_str = "Synchronous" if synchronous_mode else "Asynchronous"

        logger.info(f"[Simulator] World is set to {synchronous_mode_str} mode")
        logger.info(f"[Simulator] Running ...")

        self.local_scenario.scenario_start()
        while self.sim_status:
            self.carla_world.wait_for_tick()

            # TODO: Load scenario or refresh npcs
            self.local_scenario.npc_refresh()

            module_status = dv.get_module_status()
            for module, status in module_status.items():
                if (module not in self.modules or
                        status):
                    continue
                if module == "Prediction" or module == "Planning":
                    logger.warning('[Simulator] Module is closed: ' +
                                   module + '==> restrating')
                    self.carla_bridge.pause_event.set()
                    dv.enable_apollo(self.destination, self.modules)
                    dv.set_destination_tranform(self.destination)
                    if self.restart_module(dv, module):
                        logger.info('[Simulator] Module: ' +
                                    module + '==> restrated !')
                        self.carla_bridge.pause_event.clear()
                        continue
                    else:
                        logger.error('[Simulator] Module is closed: ' +
                                     module + '==> restrat timeout')
                        self.sim_status = False
                        break
                logger.warning('[Simulator] Module is closed: ' +
                               module + ' ==> maybe not affect')
        self.close()
        logger.info('[Simulator] === Simulation End === ')

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
            # distance = math.sqrt(
            #     (des_location.location.x - ego_curr_point.location.x) ** 2 +
            #     (des_location.location.y - ego_curr_point.location.y) ** 2
            # )
            if distance > radius:
                valid_destination = True
        return des_transform

    def restart_module(self, dv_remote: dreamview.Connection,
                       module, retry_times: int = 5) -> bool:
        module_status = None
        while module_status != True:
            dv_remote.enable_module(module)
            time.sleep(2)
            for module_i, status in dv_remote.get_module_status().items():
                if module_i == module:
                    module_status = status
                if module_status:
                    return True
            retry_times -= 1
            if retry_times <= 0:
                return False

    def close(self):
        if self.local_scenario:
            self.local_scenario.remove_all_npcs()
        if self.cfgs.load_bridge:
            if self.carla_client:
                self.carla_world = self.carla_client.get_world()
                settings = self.carla_world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.carla_world.apply_settings(settings)
                logger.warning("Shutting down.")
            if self.carla_bridge.shutdown:
                self.carla_bridge.shutdown.set()
            self.carla_bridge.destroy()
            logger.info("brigde destroied")
        if self.sim_status:
            self.sim_status = False
        if self.carla_world:
            del self.carla_world
            self.carla_world = None
        if self.carla_client:
            del self.carla_client
            self.carla_client = None


if __name__ == "__main__":
    cfg = Config()
    sim = Simulator(cfg)
    sim.run()
    pdb.set_trace()
