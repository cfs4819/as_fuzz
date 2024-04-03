import copy
import random
import threading
import numpy as np
import carla
import time
from typing import List

import pdb

from loguru import logger
from typing import List

from MS_fuzz.ga_engine.gene import *

from agents.navigation.behavior_agent import BehaviorAgent


class NpcBase(object):
    def __init__(self,
                 start_loc,
                 end_loc,
                 blueprint: carla.ActorBlueprint,
                 start_time):

        self.start_loc: carla.Transform = start_loc
        self.end_loc: carla.Transform = end_loc
        self.blueprint: carla.ActorBlueprint = blueprint
        self.waypoints = []
        self.start_time = start_time

        self.is_running = False
        self.reached_destination = False
        self.control_thread: threading.Thread = None
        self.close_event: threading.Event = None


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
        self.vehicle = None  # the actor object
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
        self.walker = None  # the actor object
        self.max_speed = max_speed
        self.behavior_type: int = behavior_type  # 0: walking, 1: stoped
        self.ai_controller: carla.WalkerAIController = None
        self.walker_id: str = walker_id


class LocalScenario(object):

    '''

        # Run as follow step:
            1. add npcs into npc list by add_npc_vehicle() or add_npc_walker()
            2. spawn npcs by spawn_all_npcs()
            3. start running by scenario_start()
            4. tick the scenario repeatedly by npc_refresh()
            5. stop all running walkers & vehicles by stop_all_npcs()
            6. remove all of them from the scenario by remove_all_npcs()


    '''

    def __init__(self,
                 carla_world: carla.World,
                 ego_vhicle: carla.Vehicle,
                 logger=logger):
        self.logger = logger
        self.ego = ego_vhicle  # dict
        self.carla_world: carla.World = carla_world
        self.carla_map: carla.Map = self.carla_world.get_map()

        self.scenario_start_time: carla.Timestamp = self.carla_world.get_snapshot().timestamp

        # init npc para
        self.npc_vehicle_list: List[NpcVehicle] = []
        self.npc_walker_list: List[NpcWalker] = []

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

    def add_npc_vehicle(self,
                        start_loc: dict,
                        end_loc: dict,
                        start_time,
                        behavior_type,
                        agent_type,
                        start_speed=0.0,
                        blueprint: carla.ActorBlueprint = None,
                        bp_type=0,
                        vehicle_id=None):
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
                                    chosen from ['cautious', 'normal', 'aggressive'].
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
            lane_type=(carla.LaneType.Driving if behavior_type ==
                       0 else carla.LaneType.Shoulder)
        )

        start_waypoint_tf = start_waypoint.transform
        start_waypoint_tf.location.z += 2.5

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

        valid_agent_types = ['cautious', 'normal', 'aggressive']
        if agent_type not in valid_agent_types:
            agent_type = 'normal'

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

        self.npc_vehicle_list.append(NpcVehicle(start_loc=start_waypoint_tf,
                                                end_loc=end_waypoint_tf,
                                                blueprint=blueprint,
                                                start_time=start_time,
                                                behavior_type=behavior_type,
                                                agent_type=agent_type,
                                                start_speed=start_speed,
                                                vehicle_id=vehicle_id))

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
        start_waypoint.transform.location.z += 1.5

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

        self.npc_walker_list.append(NpcWalker(start_loc=start_waypoint.transform,
                                              end_loc=end_waypoint.transform,
                                              blueprint=blueprint,
                                              start_time=start_time,
                                              max_speed=max_speed,
                                              behavior_type=behavior_type,
                                              walker_id=walker_id))

    def spawn_all_npcs(self):
        # call when loading scenarios

        for vehicle in self.npc_vehicle_list:
            vehicle.vehicle = self.carla_world.try_spawn_actor(vehicle.blueprint,
                                                               vehicle.start_loc)
            self.carla_world.wait_for_tick()
            if vehicle.vehicle == None:
                logger.warning(
                    f"Vehicle spawned failed: id is {vehicle.vehicle_id}")
                continue

            logger.info(
                f"Vehicle spawned: id is {vehicle.vehicle_id}")
            if vehicle.behavior_type == 2:
                # is a parked vehicle
                continue

            # carla_db = self.carla_world.debug
            # carla_db.draw_point(vehicle.vehicle.get_transform().location+carla.Location(0,0,3),
            #                     size=0.1, color=carla.Color(238,233,191))

            vehicle.agent = BehaviorAgent(
                vehicle.vehicle, behavior=vehicle.agent_type)
            vehicle.agent.set_destination(vehicle.end_loc.location)

        for walker in self.npc_walker_list:
            walker.walker = self.carla_world.try_spawn_actor(
                walker.blueprint, walker.start_loc)

            if walker.walker == None:
                logger.warning(
                    f"Walker spawned failed: id is {walker.walker_id}")
                continue
            if walker.behavior_type == 1:
                continue
            walker_controller_bp = self.world_blueprint.find(
                'controller.ai.walker')
            walker.ai_controller = self.carla_world.spawn_actor(
                walker_controller_bp, walker.start_loc, attach_to=walker.walker)
            walker.ai_controller.go_to_location(
                walker.end_loc.location)
            walker.ai_controller.set_max_speed(walker.max_speed)

        # self.carla_world.wait_for_tick()

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
                args=(vehicle, ),
                name=f"thread_{vehicle.vehicle_id}")
            vehicle.control_thread.start()
        for walker in self.npc_walker_list:
            if walker.behavior_type == 1:
                continue
            walker.close_event = threading.Thread()
            walker.control_thread = threading.Thread(
                target=self.walker_control_handler,
                args=(walker, ),
                name=f"thread_{walker.walker_id}")
            walker.control_thread.start()

    def vehicle_control_handler(self, vehicle: NpcVehicle):
        while not vehicle.close_event.is_set():
            with self.refresh_condition:
                # 1. wait until vehicle can run
                while (not vehicle.is_running and
                       not vehicle.reached_destination and
                       not vehicle.close_event.is_set()
                       ):
                    self.refresh_condition.wait()
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
                while (vehicle.is_running and
                       not vehicle.reached_destination and
                       not vehicle.close_event.is_set()):
                    self.refresh_condition.wait()

                    # Apply control to the vehicle
                    ctrl = vehicle.agent.run_step(debug=True)
                    vehicle.vehicle.apply_control(ctrl)

                    # print(f"Controlling {vehicle.vehicle_id}, at {vehicle.agent}")

                    if vehicle.agent.done():
                        vehicle.reached_destination = True
                        vehicle.is_running = False
                        break
                if vehicle.close_event.is_set():
                    break

        # If close_event is set, stop the vehicle
        vehicle.vehicle.apply_control(vehicle.agent.emergency_stop())
        vehicle.is_running = False

    def walker_control_handler(self, walker: NpcWalker):
        while not walker.close_event.is_set():
            with self.refresh_condition:
                # 1. wait until walker can run
                while (not walker.is_running and not walker.close_event.is_set()):
                    self.refresh_condition.wait()
                    if walker.close_event.is_set():
                        break
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

        # time to close
        if walker.ai_controller:
            walker.is_running = False
            walker.ai_controller.stop()

        while not walker.close_event.is_set():
            with self.refresh_condition:
                self.refresh_condition.wait()
                if walker.is_running:
                    # check if walker has been running
                    pass
                curr_time = self.carla_world.get_snapshot().timestamp
                time_passed = curr_time.elapsed_seconds - \
                    self.scenario_start_time.elapsed_seconds
                if time_passed >= walker.start_time:
                    # is time that this walker can run
                    if walker.behavior_type == 0 and walker.ai_controller != None:
                        walker.ai_controller.start()
                        walker.ai_controller.go_to_location(
                            walker.end_loc.location)
                        walker.ai_controller.set_max_speed(walker.max_speed)
                    walker.is_running = True
                continue

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

        # make sure all agent threads can finish
        with self.refresh_condition:
            self.refresh_condition.notify_all()
            self.refresh_condition.notify_all()

        for agent_list in [self.npc_vehicle_list, self.npc_walker_list]:
            for agent in agent_list:
                if agent.control_thread:
                    agent.control_thread.join()

        # delete both parked vehicle and driving vehicle
        for vehicle in self.npc_vehicle_list:
            if vehicle.vehicle:
                vehicle.vehicle.destroy()
            self.npc_vehicle_list.remove(vehicle)

        for walker in self.npc_walker_list:
            if walker.ai_controller:
                walker.ai_controller.destroy()
            if walker.walker:
                walker.walker.destroy()
            self.npc_walker_list.remove(walker)
        self.carla_world.wait_for_tick()

    def npc_refresh(self):
        '''
            refresh all npc control, called every time you tick the world
        '''
        with self.refresh_condition:
            self.refresh_condition.notify_all()
        pass

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