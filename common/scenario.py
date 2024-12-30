import copy
import random
import threading
import numpy as np
import math
import carla
import time
from typing import List, Dict

import pdb

from loguru import logger
from typing import List

from MS_fuzz.ga_engine.gene import *

from agents.navigation.behavior_agent import BehaviorAgent
from MS_fuzz.ga_engine.scene_segmentation import Segment
from MS_fuzz.ms_utils import calc_relative_loc, calc_relative_loc_dict
from MS_fuzz.ms_utils import get_crosswalk_list, is_point_in_any_crosswalk
from MS_fuzz.ga_engine.gene import GeneNpcWalkerList, GeneNpcVehicleList
from MS_fuzz.common.evaluate import Evaluate_Object


class NpcBase(object):
    def __init__(self,
                 start_loc,
                 end_loc,
                 blueprint: carla.ActorBlueprint,
                 start_time,
                 free_roam=False):
        self.start_loc: carla.Transform = start_loc
        self.end_loc: carla.Transform = end_loc
        self.blueprint: carla.ActorBlueprint = blueprint
        self.waypoints = []
        self.start_time = start_time

        self.spawned = False
        self.is_running = False
        self.reached_destination = False
        self.control_thread: threading.Thread = None
        self.close_event: threading.Event = None
        self.free_roam = free_roam


class NpcVehicle(NpcBase):
    '''
        choose your blueprints by random.choice(Scenario.vehicle_blueprint)
    '''

    def __init__(self,
                 start_loc: carla.Transform,
                 end_loc: carla.Transform,
                 blueprint: carla.ActorBlueprint,
                 start_time,
                 behavior_type: int,
                 agent_type: str,
                 start_speed: float = 0.0,
                 vehicle_id=None):
        super(NpcVehicle, self).__init__(
            start_loc=start_loc,
            end_loc=end_loc,
            blueprint=blueprint,
            start_time=start_time
        )
        self.behavior_type: int = behavior_type  # 0: driving, 1: starting, 2: parked
        self.agent_type: str = agent_type
        self.agent: BehaviorAgent = None
        self.vehicle: carla.Vehicle = None  # the actor object
        self.vehicle_id: str = vehicle_id
        self.start_speed: float = start_speed


class NpcWalker(NpcBase):
    def __init__(self,
                 start_loc: carla.Transform,
                 end_loc: carla.Transform,
                 blueprint: carla.ActorBlueprint,
                 start_time,
                 max_speed: float,
                 behavior_type: int = 0,
                 walker_id=None):
        super(NpcWalker, self).__init__(
            start_loc=start_loc,
            end_loc=end_loc,
            blueprint=blueprint,
            start_time=start_time
        )
        self.walker: carla.Walker = None  # the actor object
        self.max_speed = max_speed
        self.behavior_type: int = behavior_type  # 0: walking, 1: stoped
        self.ai_controller: carla.WalkerAIController = None
        self.walker_id: str = walker_id


class LocalScenario(object):
    '''
        # Run as follow step:
            1. add npcs into npc list by `add_npc_vehicle()` or `add_npc_walker()`
            2. spawn npcs by `spawn_all_npcs()`
            3. start running by `scenario_start()`
            4. tick the scenario repeatedly by `npc_refresh()`
            5. stop all running walkers & vehicles by `stop_all_npcs()`
            6. remove all of them from the scenario by `remove_all_npcs()`


    '''

    def __init__(self,
                 carla_world: carla.World,
                 ego_vhicle: carla.Vehicle,
                 logger=logger, carla_map=None):
        self.id = ''
        self.scen_seg: Segment = None
        self.logger = logger
        self.ego: carla.Vehicle = ego_vhicle
        self.carla_world: carla.World = carla_world
        self.carla_map: carla.Map = carla_map

        self.scenario_start_time: carla.Timestamp = self.carla_world.get_snapshot().timestamp

        # init npc para
        self.npc_vehicle_list: List[NpcVehicle] = []
        self.npc_walker_list: List[NpcWalker] = []

        self.sce_obj_2_carla_actor_id: Dict[str, int] = {
            'ego_vehicle': self.ego.id}

        # environment
        self.environment = None

        self.world_blueprint: carla.BlueprintLibrary = None
        self.vehicle_blueprint: List[carla.BlueprintLibrary] = None
        self.walker_blueprint: carla.BlueprintLibrary = None

        self.vehicle_car_bps: List[carla.BlueprintLibrary] = None
        self.vehicle_truck_bps: List[carla.BlueprintLibrary] = None
        self.vehicle_van_bps: List[carla.BlueprintLibrary] = None
        self.vehicle_motorcycle_bps: List[carla.BlueprintLibrary] = None
        self.vehicle_bycicle_bps: List[carla.BlueprintLibrary] = None

        self.refresh_blueprint(self.carla_world)

        self.refresh_condition: threading.Condition = None

        self.vehicle_count = 0
        self.walker_count = 0
        self.running: bool = False

        self.evaluate_obj: Evaluate_Object = None
        self.crosswalk_list = get_crosswalk_list(
            self.carla_map.get_crosswalks())

    def attach_segment(self, segment: Segment):
        self.scen_seg = segment

    def add_npcs_from_evaluate_obj(self, obj: Evaluate_Object):
        if self.scen_seg is None:
            self.logger.error('attach segment before adding npcs')
            return
        self.add_npc_vehicles_form_gene(obj.vehicle_ind)
        self.add_npc_walkers_form_gene(obj.walker_ind)
        self.evaluate_obj = obj

    def add_npc_walkers_form_gene(self, gene: GeneNpcWalkerList):
        base_ref = carla.Transform(
            self.scen_seg.location, self.scen_seg.rotation)
        for index, walker in enumerate(gene.list):
            start_loc = calc_relative_loc_dict(base_ref, walker.start['x'],
                                               walker.start['y'], walker.start['z'])
            end_loc = calc_relative_loc_dict(base_ref, walker.end['x'],
                                             walker.end['y'], walker.end['z'])
            self.add_npc_walker(start_loc=start_loc,
                                end_loc=end_loc,
                                start_time=walker.start_time,
                                behavior_type=walker.status,
                                max_speed=walker.max_speed,
                                walker_id=f'sce_{self.id}_walker_{index}')

    def add_npc_vehicles_form_gene(self, gene: GeneNpcVehicleList):
        base_ref = carla.Transform(
            self.scen_seg.location, self.scen_seg.rotation)
        for index, vehicle in enumerate(gene.list):
            start_loc = calc_relative_loc_dict(base_ref, vehicle.start['x'],
                                               vehicle.start['y'], vehicle.start['z'])
            end_loc = calc_relative_loc_dict(base_ref, vehicle.end['x'],
                                             vehicle.end['y'], vehicle.end['z'])
            self.add_npc_vehicle(start_loc=start_loc,
                                 end_loc=end_loc,
                                 start_time=vehicle.start_time,
                                 behavior_type=vehicle.status,
                                 agent_type=vehicle.agent_type,
                                 start_speed=vehicle.initial_speed,
                                 bp_type=vehicle.vehicle_type,
                                 vehicle_id=f'sce_{self.id}_vehicle_{index}')

    # def renew_gene(self, gene_vehicle: GeneNpcVehicleList, gene_walker: GeneNpcWalkerList):
    #     pass

    def add_npc_vehicle(self,
                        start_loc: dict,
                        end_loc: dict,
                        start_time,
                        behavior_type: int,
                        agent_type: int,
                        start_speed=0.0,
                        blueprint: carla.ActorBlueprint = None,
                        bp_type: int = 0,
                        vehicle_id=None,
                        free_roam=False):
        '''
        Add a specified vehicle into vehicle list, but have not spawned it

            Parameters:
                start_loc       :   The starting location of the NPC vehicle,
                                    specified as a dictionary with keys 'x' and 'y'.
                end_loc         :   The ending location of the NPC vehicle,
                                    specified as a dictionary with keys 'x' and 'y'.
                start_time      :   The starting time of the NPC vehicle's movement.
                                    In seconds. Max 2s.
                behavior_type   :   Indicate the working status of the vehicle.
                                    0: driving,
                                    1: starting,
                                    2: parked.
                agent_type      :   The type of agent controlling the NPC vehicle,
                                    0: normal,
                                    1: cautious,
                                    2: aggressive.
                start_speed     :   The initial speed of the NPC vehicle,
                                    defaults to 0.0 if not specified.
                blueprint       :   The blueprint of the NPC vehicle,
                                    if not provided, a random blueprint will be used.
                bp_type         :   The type of blueprint used for the NPC vehicle.
                                    0: Car,
                                    1: Truck,
                                    2: Van,
                                    3: Motorcycle,
                                    4: Bicycle.
                vehicle_id      :   Optional identifier for the NPC vehicle.

            Returns             :   None. This function doesn't return anything.


        '''

        if behavior_type not in range(0, 3):
            behavior_type = 2

        if behavior_type != 0:
            # not drving vehicle
            start_speed = 0.0

        start_waypoint = self.carla_map.get_waypoint(
            carla.Location(x=start_loc['x'],
                           y=start_loc['y'], z=start_loc['z']),
            project_to_road=True,
            lane_type=(carla.LaneType.Driving if behavior_type
                       == 0 else carla.LaneType.Shoulder)
        )

        start_waypoint_tf = start_waypoint.transform
        start_waypoint_tf.location.z += 1

        end_waypoint = self.carla_map.get_waypoint(
            carla.Location(x=end_loc['x'],
                           y=end_loc['y'], z=end_loc['z']),
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        end_waypoint_tf = end_waypoint.transform
        # carla_db = self.carla_world.debug
        # carla_db.draw_point(location=end_waypoint_tf.location+carla.Location(0,0,3),
        #                     size=0.1)
        # end_waypoint.transform.location.z += 2.5

        agent_type_str = 'normal'
        if agent_type == 1:
            agent_type_str = 'cautious'
        elif agent_type == 2:
            agent_type_str = 'aggressive'

        if blueprint == None:
            if bp_type == 0:
                blueprint = random.choice(self.vehicle_car_bps)
            elif bp_type == 1:
                blueprint = random.choice(self.vehicle_truck_bps)
            elif bp_type == 2:
                blueprint = random.choice(self.vehicle_van_bps)
            elif bp_type == 3:
                blueprint = random.choice(self.vehicle_motorcycle_bps)
            elif bp_type == 4:
                blueprint = random.choice(self.vehicle_bycicle_bps)
            else:
                blueprint = random.choice(self.vehicle_blueprint)

        if vehicle_id == None:
            self.vehicle_count = self.vehicle_count + 1
            vehicle_id = f'npc_vehicle_{self.vehicle_count}'

        npc_vehicle = NpcVehicle(start_loc=start_waypoint_tf,
                                 end_loc=end_waypoint_tf,
                                 blueprint=blueprint,
                                 start_time=start_time,
                                 behavior_type=behavior_type,
                                 agent_type=agent_type_str,
                                 start_speed=start_speed,
                                 vehicle_id=vehicle_id)
        npc_vehicle.free_roam = free_roam
        self.npc_vehicle_list.append(npc_vehicle)

    def add_npc_walker(self,
                       start_loc: dict,
                       end_loc: dict,
                       start_time,
                       behavior_type,
                       max_speed=1.4,
                       blueprint: carla.ActorBlueprint = None,
                       walker_id=None):
        """
        Add a specified walker into the walker list, but have not spawned it.

            Parameters:
                start_loc       :   The starting location of the NPC walker,
                                    specified as a dictionary with keys 'x' and 'y'.
                end_loc         :   The ending location of the NPC walker,
                                    specified as a dictionary with keys 'x' and 'y'.
                start_time      :   The starting time of the NPC walker's movement.
                                    In seconds.
                behavior_type   :   Indicate the behavior type of the walker.
                                    0: walking,
                                    1: stopped.
                max_speed       :   The maximum speed of the NPC walker,
                                    defaults to 1.4 if not specified.
                blueprint       :   The blueprint of the NPC walker,
                                    if not provided, a random blueprint will be used.
                walker_id       :   Optional identifier for the NPC walker.

            Returns:
                None. This function doesn't return anything.
        """
        if behavior_type not in range(0, 1):
            behavior_type = 0

        if behavior_type == 1:
            max_speed = 0

        start_waypoint = self.carla_map.get_waypoint(
            carla.Location(x=start_loc['x'],
                           y=start_loc['y'], z=start_loc['z']),
            project_to_road=True,
            lane_type=carla.LaneType.Sidewalk
        )

        start_waypoint_tf = start_waypoint.transform
        start_waypoint_tf.location.z += 1

        end_waypoint = self.carla_map.get_waypoint(
            carla.Location(x=end_loc['x'],
                           y=end_loc['y'], z=end_loc['z']),
            project_to_road=True,
            lane_type=carla.LaneType.Sidewalk
        )

        if blueprint == None:
            blueprint = random.choice(self.walker_blueprint)

        if walker_id == None:
            self.walker_count = self.walker_count + 1
            walker_id = f'npc_walker_{self.walker_count}'

        self.npc_walker_list.append(NpcWalker(start_loc=start_waypoint_tf,
                                              end_loc=end_waypoint.transform,
                                              blueprint=blueprint,
                                              start_time=start_time,
                                              max_speed=max_speed,
                                              behavior_type=behavior_type,
                                              walker_id=walker_id))

    def spawn_a_vehicle_handler(self, vehicle: NpcVehicle):
        vehicle.vehicle = self.carla_world.try_spawn_actor(vehicle.blueprint,
                                                           vehicle.start_loc)
        self.carla_world.wait_for_tick()
        if vehicle.vehicle == None:
            logger.warning(
                f"Vehicle spawned failed: id is {vehicle.vehicle_id}")
            self.sce_obj_2_carla_actor_id[vehicle.vehicle_id] = -1
            return
        vehicle.spawned = True
        # logger.info(f"Vehicle spawned: id is {vehicle.vehicle_id}")
        self.sce_obj_2_carla_actor_id[vehicle.vehicle_id] = vehicle.vehicle.id
        if vehicle.behavior_type == 2:
            # is a parked vehicle
            return

        vehicle.agent = BehaviorAgent(vehicle.vehicle,
                                      behavior=vehicle.agent_type)
        vehicle.agent.set_destination(vehicle.end_loc.location)
        return

    def spawn_a_walker_handler(self, walker: NpcWalker):
        walker.walker = self.carla_world.try_spawn_actor(walker.blueprint,
                                                         walker.start_loc)
        self.carla_world.wait_for_tick()
        if walker.walker == None:
            logger.warning(
                f"Walker spawned failed: id is {walker.walker_id}")
            self.sce_obj_2_carla_actor_id[walker.walker_id] = -1
            return
        walker.spawned = True
        self.sce_obj_2_carla_actor_id[walker.walker_id] = walker.walker.id
        if walker.behavior_type == 1:
            return
        walker_controller_bp = self.world_blueprint.find(
            'controller.ai.walker')
        walker.ai_controller = self.carla_world.spawn_actor(walker_controller_bp,
                                                            walker.start_loc,
                                                            attach_to=walker.walker)
        walker.ai_controller.go_to_location(walker.end_loc.location)
        walker.ai_controller.set_max_speed(walker.max_speed)
        return

    def spawn_all_npcs(self):
        # call when loading scenarios

        vehicle_spawn_threads: List[threading.Thread] = []
        walker_spawn_threads: List[threading.Thread] = []

        for vehicle in self.npc_vehicle_list:
            this_spawn_thread = threading.Thread(
                target=self.spawn_a_vehicle_handler, args=(vehicle,))
            vehicle_spawn_threads.append(this_spawn_thread)
            this_spawn_thread.start()

        for walker in self.npc_walker_list:
            this_spawn_thread = threading.Thread(
                target=self.spawn_a_walker_handler, args=(walker,))
            walker_spawn_threads.append(this_spawn_thread)
            this_spawn_thread.start()

        for spawn_thread in vehicle_spawn_threads:
            spawn_thread.join()

        for spawn_thread in walker_spawn_threads:
            spawn_thread.join()

    def scenario_start(self):
        self.scenario_start_time = self.carla_world.get_snapshot().timestamp

        self.refresh_condition = threading.Condition()

        for vehicle in self.npc_vehicle_list:
            if vehicle.behavior_type == 2:
                continue
            if vehicle.vehicle == None:
                continue
            vehicle.close_event = threading.Event()
            vehicle.control_thread = threading.Thread(
                target=self.vehicle_control_handler,
                args=(vehicle,),
                name=f"thread_{vehicle.vehicle_id}")
            vehicle.control_thread.start()
        for walker in self.npc_walker_list:
            if walker.behavior_type == 1:
                continue
            walker.close_event = threading.Event()
            walker.control_thread = threading.Thread(
                target=self.walker_control_handler,
                args=(walker,),
                name=f"thread_{walker.walker_id}")
            walker.control_thread.start()

        self.running = True

    def scenario_end(self) -> Evaluate_Object:
        self.remove_all_npcs()
        if self.evaluate_obj:
            self.evaluate_obj.evaluate()

        self.running = False
        if self.evaluate_obj:
            return self.evaluate_obj

    def vehicle_control_handler(self, vehicle: NpcVehicle):
        while not vehicle.close_event.is_set():
            with self.refresh_condition:
                self.refresh_condition.wait()
                if vehicle.close_event.is_set():
                    break
                # 1. wait until vehicle can run
                if (not vehicle.is_running
                            and not vehicle.close_event.is_set()
                        ):
                    # Check if it's time for this vehicle to run
                    curr_time = self.carla_world.get_snapshot().timestamp
                    time_passed = curr_time.elapsed_seconds - \
                                  self.scenario_start_time.elapsed_seconds
                    if time_passed >= vehicle.start_time:
                        if vehicle.behavior_type == 0:
                            forward_vector = vehicle.vehicle.get_transform().rotation.get_forward_vector()
                            start_velocity = forward_vector * vehicle.start_speed
                            vehicle.vehicle.set_target_velocity(start_velocity)
                        vehicle.is_running = True
                if vehicle.close_event.is_set():
                    break

                # 2. Once the vehicle starts running, wait for refresh signal from main thread
                elif (vehicle.is_running
                      and not vehicle.close_event.is_set()):

                    # Apply control to the vehicle
                    ctrl = vehicle.agent.run_step(debug=True)
                    vehicle.vehicle.apply_control(ctrl)

                    # print(f"Controlling {vehicle.vehicle_id}, at {vehicle.agent}")

                    if vehicle.agent.done():
                        if not vehicle.free_roam:
                            # finished, close the thread
                            vehicle.reached_destination = True
                            vehicle.is_running = False
                            return
                        else:
                            new_dest = random.choice(
                                self.carla_map.get_spawn_points())
                            vehicle.agent.set_destination(new_dest.location)

        # If close_event is set, stop the vehicle
        vehicle.vehicle.apply_control(vehicle.agent.emergency_stop())
        vehicle.is_running = False

    def walker_control_handler(self, walker: NpcWalker):
        while not walker.close_event.is_set():
            with self.refresh_condition:
                self.refresh_condition.wait()
                if walker.close_event.is_set():
                    break
                # 1. wait until walker can run
                if (not walker.is_running and not walker.close_event.is_set()):
                    # Check if it's time for this walker to run
                    curr_time = self.carla_world.get_snapshot().timestamp
                    time_passed = curr_time.elapsed_seconds - \
                                  self.scenario_start_time.elapsed_seconds
                    if time_passed < walker.start_time:
                        continue
                    # is time that this walker can run
                    if walker.behavior_type == 0 and walker.ai_controller != None:
                        walker.ai_controller.start()
                        walker.ai_controller.go_to_location(
                            walker.end_loc.location)
                        walker.ai_controller.set_max_speed(walker.max_speed)
                        walker.is_running = True

                # 2.running, nothing we can do
                else:
                    pass

        # time to close
        if walker.ai_controller:
            walker.is_running = False
            walker.ai_controller.stop()

    def remove_all_npcs(self):
        # call when unloading scenarios
        # stop all running agents
        for agent_list in [self.npc_vehicle_list, self.npc_walker_list]:
            for agent in agent_list:
                if agent.control_thread and agent.close_event:
                    agent.close_event.set()
                if self.refresh_condition:
                    with self.refresh_condition:
                        self.refresh_condition.notify_all()
                        self.refresh_condition.notify_all()

        # make sure all agent threads can finish
        if self.refresh_condition:
            with self.refresh_condition:
                self.refresh_condition.notify_all()
                self.refresh_condition.notify_all()

        for agent_list in [self.npc_vehicle_list, self.npc_walker_list]:
            for agent in agent_list:
                if agent.control_thread:
                    agent.control_thread.join()

        # delete both parked vehicle and driving vehicle
        try:
            for vehicle in self.npc_vehicle_list:
                if vehicle.vehicle:
                    vehicle.vehicle.destroy()
                # self.npc_vehicle_list.remove(vehicle)
            self.npc_vehicle_list = []
            for walker in self.npc_walker_list:
                if walker.ai_controller:
                    walker.ai_controller.destroy()
                if walker.walker:
                    walker.walker.destroy()
                # self.npc_walker_list.remove(walker)
            self.npc_walker_list = []
            self.carla_world.wait_for_tick()
        except:
            pass

    def npc_refresh(self):
        '''
            refresh all npc control, called every time you tick the world
        '''
        with self.refresh_condition:
            self.refresh_condition.notify_all()

    def refresh_blueprint(self, world: carla.World):
        self.world_blueprint = world.get_blueprint_library()
        self.vehicle_blueprint = self.world_blueprint.filter('vehicle')
        self.walker_blueprint = self.world_blueprint.filter('walker')

        self.vehicle_car_bps = [bp for bp in self.vehicle_blueprint if bp.get_attribute(
            'base_type') == "car"]
        self.vehicle_truck_bps = [bp for bp in self.vehicle_blueprint if bp.get_attribute(
            'base_type') == "truck"]
        self.vehicle_van_bps = [bp for bp in self.vehicle_blueprint if bp.get_attribute(
            'base_type') == "van"]
        self.vehicle_motorcycle_bps = [bp for bp in self.vehicle_blueprint if bp.get_attribute(
            'base_type') == "motorcycle"]
        self.vehicle_bycicle_bps = [bp for bp in self.vehicle_blueprint if bp.get_attribute(
            'base_type') == "bycicle"]

    def evaluate_snapshot_record(self, world_snapshot: carla.WorldSnapshot):
        '''
            1. for all individuals:
                f_distance: the minimum distance between ego and other npcs during the simulation time t
                f_smooth : represents the ego vehicle's acceleration during a scene
                f_diversity: diversity of the scene
            2. for npc_walkers:
                f_crossing_time : Time taken to cross the road
            3. for npc_vehicles:s
                f_interaction_rate: the rate at which vehicles interact with the ego vehicle
        '''
        frame = world_snapshot.frame
        ego_ss = world_snapshot.find(self.ego.id)
        if not ego_ss:
            return None
        npc_vehicles_ss = []
        for vehicle in self.npc_vehicle_list:
            if not vehicle.vehicle:
                continue
            npc_vehicles_ss.append(world_snapshot.find(vehicle.vehicle.id))
        npc_walkers_ss = []
        for walker in self.npc_walker_list:
            if not walker.walker:
                continue
            npc_walkers_ss.append(world_snapshot.find(walker.walker.id))

        min_dis = 9999
        for npc in npc_vehicles_ss + npc_walkers_ss:
            if ego_ss == None:
                return None
            if npc == None:
                continue
            dis = ego_ss.get_transform().location.distance(npc.get_transform().location)
            if dis < min_dis:
                min_dis = dis

        unsmooth_acc = 0 if (abs(ego_ss.get_acceleration().x) < 4) else (
            abs(ego_ss.get_acceleration().x) - 4)

        walker_in_road = 0
        for walker in npc_walkers_ss:
            if walker == None:
                continue
            if is_point_in_any_crosswalk(walker.get_transform().location, self.crosswalk_list):
                walker_in_road += 1

        frame_record = {
            'timestamp': time.time(),
            'frame': frame,
            'min_dis': min_dis,
            'unsmooth_acc': unsmooth_acc,
            'walker_in_road': walker_in_road,
            'npc_vehicles_ss': npc_vehicles_ss,
            'ego_ss': ego_ss
        }
        if self.evaluate_obj:
            self.evaluate_obj.frame_recorded.append(frame_record)
        return frame_record
        # caculate later
        # vehicle_may_collide = 0
        # for vehicle in npc_vehicles_ss:
        #     collision, c_t = self.predict_collision(vehicle, ego_ss)
        #     if collision:
        #         vehicle_may_collide += 1
