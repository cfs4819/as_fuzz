import os
import carla
import time
import numpy as np
import threading
import random
import sys
import queue
import signal
import pdb

from datetime import datetime
from typing import Dict
from loguru import logger
from multiprocessing import Queue

from MS_fuzz.fuzz_config.Config import Config
from MS_fuzz.ms_utils import calc_relative_loc, freeze_and_set_green_all_tls, select_valid_dest
from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
from carla_bridge.utils.transforms import carla_transform_to_cyber_pose
from carla_bridge.utils.logurus import init_log
from carla_bridge.dreamview_carla import dreamview

from MS_fuzz.common.scenario import LocalScenario
from MS_fuzz.common.camera_agent_imageio import ScenarioRecorder
from MS_fuzz.common.unsafe_detector import UNSAFE_TYPE, UnsafeDetector
from MS_fuzz.common.evaluate import Evaluate_Object, Evaluate_Transfer
from MS_fuzz.ga_engine.scene_segmentation import SceneSegment
from MS_fuzz.common.result_saver import ResultSaver
from MS_fuzz.planning.is_stuck import RoadBlockageChecker, is_vehicle_in_front, get_block_head_vehicle, is_vehicle_around
from MS_fuzz.planning.VehicleNavigation import VehicleNavigation


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


class Simulator(object):

    def __init__(self, cfgs: Config,
                 eva_req_queue: Queue = None,
                 eva_res_queue: Queue = None):
        self.carla_client = None
        self.carla_world = None
        self.carla_map = None
        self.ego_vehicle = None

        self.destination = None

        self.carla_bridge_thread = None
        self.cfgs: Config = cfgs

        self.sim_status = False

        self.ego_spawn_loc = None

        self.simulation_count = 0

        self.modules = [
            'Transform',  # ok
            'Routing',
            'Prediction',  # ok
            'Planning',  # ok
            'Control',
            'Storytelling'  # ok
        ]

        self.carla_bridge = CarlaCyberBridge()
        self.dv: dreamview.Connection = None

        self.curr_local_scenario: LocalScenario = None
        self.next_local_scenario: LocalScenario = None
        self.prev_local_scenario: LocalScenario = None
        self.scene_segmentation: SceneSegment = None

        # self.ga_lib: Dict[str, CEGA] = ga_lib
        self.eva_req_queue = eva_req_queue
        self.eva_res_queue = eva_res_queue

        self.close_event = threading.Event()
        self.closing = False
        self.closed = False

        self.recorder: ScenarioRecorder = None
        self.unsafe_detector: UnsafeDetector = None
        self.result_saver: ResultSaver = ResultSaver()
        self.road_blockage_Checker: RoadBlockageChecker = None
        self.is_recording = False

        self.on_unsafe_lock = False
        self.start_unsafe_callback = False

        self.simulation_start_time = None
        self.stuck_trigger_times = 0

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
            self.carla_bridge.initialize_bridge(self.carla_world,
                                                parameters,
                                                logger)

        except (IOError, RuntimeError) as e:
            logger.error(f"[Bridge] Error: {e}")
        except KeyboardInterrupt as e:
            logger.error(f"[Bridge] Error: {e}")
        except Exception as e:  # pylint: disable=W0718
            logger.error(e)
        finally:
            self.close()

    def load_carla_bridge(self, ego_spawn_loc: dict = None):
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
        logger.info("Loading bridge")
        sp_dic = None
        if ego_spawn_loc:
            sp_wp = self.carla_map.get_waypoint(carla.Location(x=ego_spawn_loc['x'],
                                                               y=-ego_spawn_loc['y'],
                                                               z=ego_spawn_loc['z']))
            sp_tf = sp_wp.transform
            sp_pose = carla_transform_to_cyber_pose(sp_tf)
            sp_dic = {"x": sp_pose.position.x,
                      "y": -sp_pose.position.y,
                      "z": sp_pose.position.z + 2,
                      "roll": sp_tf.rotation.roll,
                      "pitch": sp_tf.rotation.pitch,
                      "yaw": -sp_tf.rotation.yaw}
        if self.carla_world == None:
            logger.error("[Bridge] carla needed to be connected \
                    before loading bridge")
            return
        self.carla_bridge_thread = threading.Thread(target=self.carla_bridge_handler,
                                                    args=(sp_dic,))
        self.carla_bridge_thread.start()

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
            self.carla_client = carla.Client(host=self.cfgs.sim_host,
                                             port=self.cfgs.sim_port)
            self.carla_client.set_timeout(self.cfgs.load_world_timeout)
            self.carla_world = self.carla_client.get_world()
            curr_map_name = self.carla_world.get_map().name.split('/')[-1]
            curr_map_name = curr_map_name.lower()
            if reload_world or not curr_map_name in [self.cfgs.carla_map.lower(),
                                                     (self.cfgs.carla_map + '_Opt').lower()]:
                self.carla_world = self.carla_client.load_world(
                    self.cfgs.carla_map)
            self.carla_map = self.carla_world.get_map()

        except Exception as e:
            logger.error(f'[Simulator] Connect Carla wrong: {e}')
            self.close()
        except KeyboardInterrupt as e:
            logger.error('[Simulator]KeyboardInterrupt: {e.message}')
            self.close()
        logger.info(
            f'[Simulator] Connected {self.cfgs.sim_host}:{self.cfgs.sim_port}')

    def init_environment(self) -> bool:
        self.connect_carla()

        self.load_carla_bridge(ego_spawn_loc=self.ego_spawn_loc)
        time.sleep(2)
        retry_times = 0
        self.ego_vehicle = None
        logger.info("[Simulator] Waiting for carla_bridge")
        while True:
            print("[*] Waiting for carla_bridge "
                  + "." * retry_times + "\r", end="")
            vehicles = self.carla_world.get_actors().filter("*vehicle.*")
            for vehicle in vehicles:
                if vehicle.attributes["role_name"] == "ego_vehicle":
                    self.ego_vehicle = vehicle
                    break
            if self.ego_vehicle:
                logger.info("[Simulator] Ego vehicle found")
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
        time.sleep(2)
        return True

    def initialization(self):
        now = datetime.now()
        date_time = now.strftime("%Y%m%d_%H%M%S")
        logger.info('[Simulator] === Simulation Start:  \
                [' + date_time + '] ===')

        self.simulation_start_time = time.time()

        self.simulation_count += 1

        if not self.init_environment():
            sys.exit()

        freeze_and_set_green_all_tls(self.carla_world, logger)

        curr_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.result_path = os.path.join(self.cfgs.out_dir, curr_datetime)

        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.recorder = ScenarioRecorder(self.carla_world,
                                         self.ego_vehicle,
                                         self.result_path)

        self.unsafe_detector = UnsafeDetector(self.carla_world,
                                              self.ego_vehicle,
                                              try_relese_block=self.cfgs.try_relese_block)

        self.unsafe_detector.register_callback(self.on_unsafe)
        self.unsafe_detector.init_sensors()

        logger.info('[Simulator] Recorder and Detector initialized')

        self.scene_segmentation = SceneSegment(self.carla_world,
                                               self.ego_vehicle,
                                               logger=logger,
                                               debug=False)

        self.scene_segmentation.routing_listener.start()

        times = 0
        success = False
        self.destination = select_valid_dest(self.ego_vehicle, self.carla_map,
                                             min_radius=100, max_radius=9999)
        logger.info('[Simulator] setting up apollo')
        while times < 3:
            try:
                time.sleep(2)
                self.dv.set_hd_map(self.cfgs.dreamview_map)
                self.dv.set_vehicle(self.cfgs.dreamview_vehicle)
                self.dv.set_setup_mode('Mkz Standard Debug')
                self.dv.enable_apollo(self.destination, self.modules)
                success = True
                times += 1
                break
            except Exception as e:
                logger.warning(
                    '[Simulator] Fail to spin up apollo, try again!', e)
                times += 1
        if not success:
            self.close()
        self.dv.set_destination_tranform(self.destination)
        route_req_time = time.time()
        logger.info(
            '[Simulator] Set Apollo (EGO) destination: '
            + str(self.destination.location.x)
            + ',' + str(self.destination.location.y))

        self.sim_status = True
        synchronous_mode = self.carla_world.get_settings().synchronous_mode
        synchronous_mode_str = "Synchronous" if synchronous_mode else "Asynchronous"

        logger.info(f"[Simulator] World is set to {synchronous_mode_str} mode")
        logger.info(f"[Simulator] Running ...")

        # Define the timeout period and retry attempts
        timeout_period = 15
        retry_attempts = 6

        logger.info("[Simulator] Waiting for Apollo to find the route")

        time.sleep(2)
        self.dv.set_destination_tranform(self.destination)
        for attempt in range(retry_attempts):
            if not self.scene_segmentation.wait_for_route(
                    route_req_time, wait_from_req_time=True, timeout=timeout_period):
                logger.warning(
                    f"[Simulator] Apollo failed to find the route, retry {attempt + 1}")
                self.dv.set_destination_tranform(self.destination)
            else:
                logger.info("[Simulator] Apollo found the route")
                break
        else:
            logger.warning(
                "[Simulator] Apollo failed to find the route, give up")
            self.close()

        self.scene_segmentation.get_segments(self.cfgs.scenario_length,
                                             self.cfgs.scenario_width)
        logger.info('[Simulator] Scene Segmentation Initialized')
        logger.info(
            f'[Simulator] Gained {len(self.scene_segmentation.segments)} segs')
        self.scene_segmentation.routing_listener.stop()

        self.scene_segmentation.strat_vehicle_pos_listening()

        self.unsafe_detector.start_detection()

        logger.info('[Simulator] Simulation Initialized')

    def feedback_eva(self, eva_result: Evaluate_Object):
        eva_result_t = Evaluate_Transfer(eva_result.res_id,
                                         eva_result.id,
                                         eva_result.walker_ind,
                                         eva_result.vehicle_ind,
                                         eva_result.is_evaluated,
                                         eva_result.is_in_queue)
        req_dic = {
            'cmd': 'feedback',
            'eva_obj': eva_result_t
        }
        self.eva_req_queue.put(req_dic)

    def resolve_blockage(self, vehicles, condition_func) -> bool:
        """
        Abstract function to resolve stuck vehicles based on a given condition.

        :param vehicles: List of vehicle actors to evaluate.
        :param condition_func: A function that takes a vehicle as input and returns True if the vehicle should be resolved.
        """
        target_vehicles = [
            vehicle for vehicle in vehicles if condition_func(vehicle)]
        if not target_vehicles:
            logger.info("[ACTION] No block vehicles found")
            return False

        vehicle_to_resolve = get_block_head_vehicle(self.ego_vehicle,
                                                    target_vehicles)
        if not vehicle_to_resolve:
            logger.info(
                "[ACTION] No block head vehicle found, select a random one")
            # Randomly choose one vehicle from the filtered list
            vehicle_to_resolve = random.choice(target_vehicles)
        logger.info(
            f"[ACTION] Resolving stuck vehicle: Vehicle ID {vehicle_to_resolve.id}")

        scenario_vehicle = None
        search_list = []

        if self.curr_local_scenario and self.curr_local_scenario.npc_vehicle_list:
            search_list += self.curr_local_scenario.npc_vehicle_list

        if self.prev_local_scenario and self.prev_local_scenario.npc_vehicle_list:
            search_list += self.prev_local_scenario.npc_vehicle_list

        for NPC_v in search_list:
            if not NPC_v.vehicle:
                continue
            if not vehicle_to_resolve:
                return False
            if vehicle_to_resolve.id == NPC_v.vehicle.id:
                scenario_vehicle = NPC_v
                break
        if scenario_vehicle:
            # stucked vehicle belongs to curr or prev,  set new dest
            new_dest = select_valid_dest(scenario_vehicle.vehicle, self.carla_map,
                                         min_radius=100, max_radius=9999)
            new_dest_loc = new_dest.location
            # change for robust
            if scenario_vehicle.agent:
                scenario_vehicle.agent.set_destination(new_dest_loc)
                scenario_vehicle.end_loc = new_dest_loc
                logger.info(f"[ACTION] New destination: {new_dest_loc}")
        else:
            # check next scenario
            if not self.next_local_scenario:
                return False
            if self.next_local_scenario.running:
                # set new dest
                for NPC_v in self.next_local_scenario.npc_vehicle_list:
                    if not NPC_v.vehicle:
                        continue
                    if not vehicle_to_resolve:
                        return False
                    if vehicle_to_resolve.id == NPC_v.vehicle.id:
                        scenario_vehicle = NPC_v
                        break
                if scenario_vehicle:
                    new_dest = select_valid_dest(scenario_vehicle.vehicle, self.carla_map,
                                                 min_radius=100, max_radius=9999)
                    new_dest_loc = new_dest.location
                    # change for robust
                    if scenario_vehicle.agent:
                        scenario_vehicle.agent.set_destination(new_dest_loc)
                        scenario_vehicle.end_loc = new_dest_loc
                        logger.info(
                            f"[ACTION] New destination: {new_dest_loc}")
            else:
                self.next_local_scenario.scenario_start()
        return True

    def on_unsafe(self, type, message, data=None):
        if self.on_unsafe_lock:
            return
        self.on_unsafe_lock = True
        '''
        COLLISION
        CROSSING_SOLID_LANE
        LANE_CHANGE
        STUCK
        ACCELERATION
        ROAD_BLOCKED
    '''
        trigger_time = time.time()
        time_pass = trigger_time - \
            self.result_saver.result_to_save['start_time']
        if type == UNSAFE_TYPE.ROAD_BLOCKED:
            logger.info(f'[Unsafe Detected]: {message}')
            # move the blocked vehicle away
            blocked_vehicles = data  # [carla.Vehicle]

            def condition_func(vehicle):
                return (is_vehicle_in_front(self.ego_vehicle,
                                            vehicle)
                        or is_vehicle_around(self.ego_vehicle,
                                             vehicle)) and \
                    vehicle.get_velocity().length() < 1.0
            handle_result = self.resolve_blockage(blocked_vehicles,
                                                  condition_func)
            self.on_unsafe_lock = False
            return handle_result

        elif type == UNSAFE_TYPE.COLLISION:
            if time_pass > 5:
                logger.info(f'[Unsafe Detected]: {message}')
                self.result_saver.result_to_save['unsafe'] = True
                self.result_saver.result_to_save['unsafe_type'] = UNSAFE_TYPE.type_str[type]
                logger.info('reload')
                self.stop_record_and_save(save_video=True)
                if self.curr_local_scenario is not None:
                    if self.curr_local_scenario.running:
                        eva_result = self.curr_local_scenario.scenario_end()
                        if eva_result != None:
                            self.feedback_eva(eva_result)
                self.close()
                logger.info('closed')
        elif type in [UNSAFE_TYPE.LANE_CHANGE,
                      UNSAFE_TYPE.CROSSING_SOLID_LANE]:
            if self.start_unsafe_callback:
                logger.info(f'[Unsafe Detected]: {message}')
                if time_pass > 5:
                    self.result_saver.add_minor_unsafe(
                        type, data, trigger_time)
        elif type == UNSAFE_TYPE.ACCELERATION:
            if self.start_unsafe_callback:
                if time_pass > 5:
                    self.result_saver.add_minor_unsafe(
                        type, data, trigger_time)
        elif type == UNSAFE_TYPE.STUCK:
            if time_pass < 60:
                self.on_unsafe_lock = False
                return
            self.stuck_trigger_times += 1
            if not self.cfgs.try_relese_block and self.next_local_scenario != None:
                # try start next scenario
                logger.info('Stucked, try start next scenario')
                if not self.next_local_scenario.running:
                    self.next_local_scenario.scenario_start()
                    self.on_unsafe_lock = False
                    return
            logger.info(f'[Unsafe Detected]: {message}')
            self.result_saver.result_to_save['unsafe'] = True
            self.result_saver.result_to_save['unsafe_type'] = UNSAFE_TYPE.type_str[type]
            self.stop_record_and_save(save_video=True)
            logger.info(f"Analyzing stucked reason")
            if self.check_road_block_stuck():
                self.result_saver.result_to_save['unsafe_type'] = UNSAFE_TYPE.type_str[UNSAFE_TYPE.ROAD_BLOCKED]
            logger.info('reload')
            if self.curr_local_scenario is not None:
                if self.curr_local_scenario.running:
                    eva_result = self.curr_local_scenario.scenario_end()
                    if eva_result != None:
                        self.feedback_eva(eva_result)
            self.close()
        self.on_unsafe_lock = False
        return

    def check_road_block_stuck(self):
        def process_navigation(i, result_queue):
            """Thread processing logic"""
            vehicle_navigation = VehicleNavigation(self.carla_world,
                                                   self.ego_vehicle,
                                                   self.scene_segmentation.routing_listener.routing_wps,
                                                   self.carla_map)
            logger.info(f'[Check Road Block] Start navigation process {i}')
            vehicle_navigation.run()
            logger.info(
                f'[Check Road Block] Navigation process {i} initialized')
            planned_path = vehicle_navigation.perform_planning()
            logger.info(f'[Check Road Block] Navigation process {i} planning completed')
            # Save visualization image
            visualization_file_path = os.path.join(self.result_path,
                                                   f"grid_visualization_{int(time.time())}.png")
            vehicle_navigation.save_visualization(
                planned_path, visualization_file_path)
            logger.info(
                f"[Check Road Block] Saved visualization {i} to {visualization_file_path}")

            # Put the result into the queue
            result_queue.put(planned_path)

        # Create a thread-safe queue
        result_queue = queue.Queue()
        threads = []

        # Create and start threads
        for i in range(3):
            thread = threading.Thread(
                target=process_navigation, args=(i, result_queue,))
            threads.append(thread)
            thread.start()
            time.sleep(1)

        # Wait for all threads to finish
        for thread in threads:
            thread.join()

        check_road_block_result_path = os.path.join(
            self.result_path, f"check_road_block_result_{int(time.time())}.json")
        
        check_result = True
        while not result_queue.empty():
            if result_queue.get():
                check_result = False
                break
        with open(check_road_block_result_path, 'w') as f:
            f.write(f'{{"check_result": {check_result}}}')
        
        # Check results in the queue
        
        return check_result
    # def check_road_block_stuck(self):
    #     vehicle_navigation = VehicleNavigation(self.carla_world,
    #                                            self.ego_vehicle,
    #                                            self.scene_segmentation.routing_listener.routing_wps,
    #                                            self.carla_map)
    #     planned_path_his = []
    #     for i in range(3):
    #         logger.info(f'[Check Road Block] Try {i}')
    #         vehicle_navigation.run()
    #         # waypoints in planned_path
    #         planned_path = vehicle_navigation.perform_planning()
    #         planned_path_his.append(planned_path)
    #         # save the pic in grid_visualization_{int(time.time())}.png
    #         visualization_file_path = os.path.join(self.result_path,
    #                                             f"grid_visualization_{int(time.time())}.png")

    #         vehicle_navigation.save_visualization(planned_path,
    #                                             visualization_file_path)
    #         logger.info(
    #             f"[Check Road Block] Save visualization to {visualization_file_path}")
    #     for plan in planned_path_his:
    #         if plan:
    #             return False
    #     return True

    def start_record(self, id=None):
        self.result_saver.clear_result()
        if id == None:
            sce_result_path = os.path.join(self.result_path,
                                           f'Scenario_{self.scene_segmentation.curr_seg_index}')
        else:
            sce_result_path = os.path.join(self.result_path,
                                           f'Scenario_{id}')
        if not os.path.exists(sce_result_path):
            os.makedirs(sce_result_path)

        self.sce_result_path = sce_result_path

        sce_video_path = os.path.join(sce_result_path, 'recorder.mp4')

        self.result_saver.set_save_path(sce_result_path)

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

        self.start_unsafe_callback = True

    def stop_record_and_save(self, save_video=True):
        if not self.is_recording:
            return
        self.is_recording = False
        self.start_unsafe_callback = False
        if save_video:
            time.sleep(1)
        self.recorder.stop_recording()
        curr_loc = self.ego_vehicle.get_location()
        self.result_saver.result_to_save['stuck_time'] = self.unsafe_detector.total_stuck_time
        self.result_saver.result_to_save['block_time'] = self.unsafe_detector.total_block_time
        self.result_saver.result_to_save['stuck_trigger_times'] = self.stuck_trigger_times
        self.result_saver.result_to_save['total_simulation_time'] = time.time(
        ) - self.simulation_start_time
        self.result_saver.save_result(curr_loc, save_video)

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

    def ego_pos_listener_handler(self, close_event: threading.Event):
        while not close_event.isSet():
            if self.check_if_ego_close_dest():
                self.on_reach_destination()
        pass

    def on_reach_destination(self):
        pass

    def main_loop(self):
        self.handle_segs()

    def handle_segs(self):
        self.carla_world.set_pedestrians_cross_factor(0.1)
        logger.info('waiting until the vehicle reach the first segment')
        # wait until the vehicle reach first segment
        waitting_first_seg_timeout = 120
        start_waitting_first_seg_time = time.time()
        while self.scene_segmentation.curr_seg_index < 0:
            # logger.info('scene_segmentation.curr_seg_index: '
            #             + str(self.scene_segmentation.curr_seg_index))
            if self.close_event.is_set():
                return
            self.carla_world.wait_for_tick()
            if time.time() - start_waitting_first_seg_time > waitting_first_seg_timeout:
                logger.error(
                    'waiting for the vehicle to reach the first segment timeout.\n Startting another Simulation')
                self.close()
                break
        logger.info(
            f'scene_segmentation.curr_seg_index={self.scene_segmentation.curr_seg_index}')

        while self.sim_status and not self.close_event.is_set():
            curr_index = self.scene_segmentation.curr_seg_index

            if self.scene_segmentation.belongs_to_two_index == (False, False):
                # just move ego forward until reaching next seg
                self.carla_world.wait_for_tick()
                continue

            if curr_index == 0:
                # if it's the first seg, curr need to be loaded
                self.curr_local_scenario = LocalScenario(self.carla_world,
                                                         self.ego_vehicle,
                                                         logger, self.carla_map)
                self.curr_local_scenario.id = str(curr_index)
                if self.close_event.is_set():
                    return
                self.load_scenario(self.curr_local_scenario,
                                   curr_index)

                self.prev_local_scenario = None
            else:
                self.prev_local_scenario = self.curr_local_scenario
                self.curr_local_scenario = self.next_local_scenario

            if curr_index != (len(self.scene_segmentation.segments) - 1):
                # if not the final seg load next
                self.next_local_scenario = LocalScenario(
                    self.carla_world, self.ego_vehicle, logger, self.carla_map)
                self.next_local_scenario.id = str(curr_index + 1)

                if self.close_event.is_set():
                    return
                self.load_scenario(self.next_local_scenario,
                                   curr_index + 1)

                # run curr
                if self.curr_local_scenario != None:
                    if not self.curr_local_scenario.running:
                        self.curr_local_scenario.scenario_start()

                if self.curr_local_scenario != None and \
                        self.curr_local_scenario.evaluate_obj != None:
                    log_id = f'{curr_index}_{self.curr_local_scenario.evaluate_obj.id}'
                else:
                    log_id = f'{curr_index}'
                self.start_record(id=log_id)

                while curr_index != self.scene_segmentation.finished_index:
                    world_ss = self.carla_world.wait_for_tick()
                    if not self.sim_status or self.close_event.is_set():
                        return
                    if self.prev_local_scenario != None:
                        if self.prev_local_scenario.running:
                            self.prev_local_scenario.npc_refresh()
                    if self.curr_local_scenario != None:
                        if self.curr_local_scenario.running:
                            self.curr_local_scenario.npc_refresh()
                    if self.next_local_scenario != None:
                        if self.next_local_scenario.running:
                            self.next_local_scenario.npc_refresh()
                    frame_record = self.curr_local_scenario.evaluate_snapshot_record(
                        world_ss)
                    self.result_saver.frames_record.append(frame_record)
                    # self.check_modules()
                    if self.scene_segmentation.belongs_to_two_index == (True, True):
                        if not self.next_local_scenario.running:
                            self.next_local_scenario.scenario_start()
                self.stop_record_and_save(save_video=False)
                eva_result = self.curr_local_scenario.scenario_end()
                if eva_result != None:
                    self.feedback_eva(eva_result)
            else:
                # aleady the final seg
                if self.close_event.is_set():
                    return
                if not self.curr_local_scenario.running:
                    self.curr_local_scenario.scenario_start()
                log_id = f'{self.curr_local_scenario.evaluate_obj.id}'
                self.start_record(id=log_id)
                while self.sim_status and not self.close_event.isSet():
                    world_ss = self.carla_world.wait_for_tick()
                    if not self.sim_status or self.close_event.isSet():
                        break
                    if self.check_if_ego_close_dest(10):
                        if self.ego_vehicle.get_control().brake > 0.5:
                            print('\r')
                            print('reach des')
                            self.stop_record_and_save(save_video=False)
                            eva_result = self.curr_local_scenario.scenario_end()
                            if eva_result != None:
                                self.feedback_eva(eva_result)
                            self.close()
                    if self.prev_local_scenario != None:
                        if self.prev_local_scenario.running:
                            self.prev_local_scenario.npc_refresh()
                    if self.curr_local_scenario != None:
                        if self.curr_local_scenario.running:
                            self.curr_local_scenario.npc_refresh()
                    frame_record = self.curr_local_scenario.evaluate_snapshot_record(
                        world_ss)
                    self.result_saver.frames_record.append(frame_record)

        self.close()
        logger.info('[Simulator] === Simulation End === ')

    def load_scenario(self,
                      scenario_2_load: LocalScenario,
                      seg_index,
                      wait_timeout=5.0):
        seg_type = self.scene_segmentation.get_seg_type(
            self.scene_segmentation.segments[seg_index],
            (self.cfgs.scenario_width, self.cfgs.scenario_length))

        obj_2_evaluate = None
        start_wait_time = time.time()
        while time.time() - start_wait_time < wait_timeout:
            if self.eva_res_queue.empty():
                req_dic = {
                    'cmd': 'get_obj',
                    'type_str': seg_type
                }
                self.eva_req_queue.put(req_dic)
            try:
                eva_res_dict = self.eva_res_queue.get(timeout=wait_timeout)
                if eva_res_dict['type_str'] == seg_type:
                    obj_2_eva_t: Evaluate_Transfer = eva_res_dict['obj']
                    obj_2_evaluate = None
                    if obj_2_eva_t != None:
                        obj_2_evaluate = Evaluate_Object(obj_2_eva_t.walker_ind,
                                                         obj_2_eva_t.vehicle_ind,
                                                         id=(obj_2_eva_t.walker_ind.id,
                                                             obj_2_eva_t.vehicle_ind.id))
                        obj_2_evaluate.res_id = obj_2_eva_t.uid
                    break
            except queue.Empty:
                # get timeout
                obj_2_evaluate = None
                break
        if obj_2_evaluate == None:
            logger.info('No individual to evaluate, Vehicles roam freely')
            return False

        scenario_2_load.attach_segment(
            self.scene_segmentation.segments[seg_index])
        scenario_2_load.add_npcs_from_evaluate_obj(obj_2_evaluate)
        scenario_2_load.spawn_all_npcs()
        return True

    def check_modules(self):
        module_status = self.dv.get_module_status()
        for module, status in module_status.items():
            if (module not in self.modules
                    or status):
                continue
            if module == "Prediction" or module == "Planning":
                logger.warning('[Simulator] Module is closed: '
                               + module + '==> restrating')
                self.pause_world()
                self.dv.enable_apollo(self.destination, self.modules)
                self.dv.set_destination_tranform(self.destination)
                if self.restart_module(self.dv, module):
                    logger.info('[Simulator] Module: '
                                + module + '==> restrated !')
                    self.pause_world(False)
                    continue
                else:
                    logger.error('[Simulator] Module is closed: '
                                 + module + '==> restrat timeout')
                    self.sim_status = False
                    break
            logger.warning('[Simulator] Module is closed: '
                           + module + ' ==> maybe not affect')

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

    def pause_world(self, pause: bool = True):
        if self.carla_bridge.pause_event == None:
            return
        if pause:
            self.carla_bridge.pause_event.set()
        else:
            self.carla_bridge.pause_event.clear()

    def close(self):
        logger.warning("Simulation Shutting down.")
        if self.closing:
            logger.warning('Other thread closing simulation, waiting')
            timeout = 20
            while self.closing and timeout > 0:
                time.sleep(0.01)
                timeout -= 0.01
            if not self.closed:
                logger.warning("Force Killing")
                sys.exit()
        self.closing = True
        logger.warning("Into Simulation Shutting down Program.")

        def exit_program():
            logger.warning("Force Exiting program due to timeout.")
            os.kill(os.getpid(), signal.SIGKILL)

        close_timeout_timer = SimulationTimeoutTimer(15, exit_program)
        close_timeout_timer.start()

        self.close_event.set()
        self.sim_status = False

        if self.is_recording:
            self.stop_record_and_save()

        if self.recorder:
            self.recorder.rm_cams()
            self.recorder = None

        if self.unsafe_detector:
            self.unsafe_detector.cleanup()
            self.unsafe_detector = None

        if self.scene_segmentation:
            if self.scene_segmentation.routing_listener.running:
                self.scene_segmentation.routing_listener.stop()
                logger.warning("[Shutdown] Routing listener stoped")

            self.scene_segmentation.stop_vehicle_listening()
            logger.warning("[Shutdown] Vehicle listening stoped")

        logger.warning(f'[Shutdown] cls, {self.curr_local_scenario}')
        if self.curr_local_scenario != None:
            if self.curr_local_scenario.running:
                self.curr_local_scenario.scenario_end()
            logger.warning("[Shutdown] Current scenario unloaded")
            self.curr_local_scenario = None

        logger.warning(f'[Shutdown] nls, {self.next_local_scenario}')
        if self.next_local_scenario:
            if self.next_local_scenario.running:
                self.next_local_scenario.scenario_end()
            logger.warning("[Shutdown] Next scenario unloaded")
            self.next_local_scenario = None

        logger.warning(f'[Shutdown] dv, {self.dv}')
        if self.dv:
            self.dv.disable_apollo()
            self.dv.disconnect()
            self.dv = None
            logger.warning("[Shutdown] Disconnected from Dreamview")

        logger.warning(f'[Shutdown] cb, {self.carla_bridge}')
        if self.carla_bridge != None:
            logger.warning("[Shutdown] Shutting down bridge")
            if self.carla_bridge.shutdown_event:
                self.carla_bridge.shutdown_event.set()
            self.carla_bridge.destroy()
            # if self.carla_bridge_thread:
            #     # can be ignore, auto clear by os.kill(os.getpid(), signal.SIGKILL)
            #     # self.carla_bridge_thread.
            #     self.carla_bridge_thread.join()
            self.carla_bridge = None
            logger.warning("[Shutdown] Brigde destroied")

        logger.warning(f'[Shutdown] cw, {self.carla_world}')
        if self.carla_world:
            del self.carla_world
            self.carla_world = None
            logger.warning("[Shutdown] Carla world destroied")

        if self.carla_client:
            del self.carla_client
            self.carla_client = None
            logger.warning("[Shutdown] Carla client destroied")
        close_timeout_timer.cancel()
        self.closed = True
        self.closing = False
        logger.warning("All cleared, Force Killing")
        sys.exit()


if __name__ == "__main__":
    cfg = Config()
    sim = Simulator(cfg)
    sim.initialization()
    sim.main_loop()
    sim.close()
    pdb.set_trace()
