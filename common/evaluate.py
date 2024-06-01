import carla

from typing import List, Dict
import math
import numpy as np
from loguru import logger

from MS_fuzz.ga_engine.gene import GeneNpcWalkerList, GeneNpcVehicleList


class Evaluate_Transfer:
    def __init__(self, uid:str,
                 id,
                 walker_ind: GeneNpcWalkerList,
                 vehicle_ind: GeneNpcVehicleList,
                 is_evaluated,
                 is_in_queue):
        self.uid: str = uid # for transfer between processes
        self.id = id
        self.walker_ind: GeneNpcWalkerList = walker_ind
        self.vehicle_ind: GeneNpcVehicleList = vehicle_ind
        
        self.is_evaluated = is_evaluated
        self.is_in_queue = is_in_queue


class Evaluate_Object:
    def __init__(self,
                 walker_ind: GeneNpcWalkerList,
                 vehicle_ind: GeneNpcVehicleList,
                 id=('', '')):
        self.walker_ind: GeneNpcWalkerList = walker_ind
        self.vehicle_ind: GeneNpcVehicleList = vehicle_ind
        if id[0] == id[1]:
            self.id = id[0]
        else:
            self.id = id[0] + '_' + id[1]

        self.is_evaluated = False
        self.is_in_queue = False
        self.res_id = ''  # for transfer between processes

        self.f_crossing_time = 0
        self.f_distance = 0
        self.f_smooth = 0
        self.f_diversity = 0
        self.f_interaction_rate = 0

        # f_crossing_time, f_distance, f_smooth, f_diversity, f_interaction_rate
        self.fitness = (self.f_crossing_time,
                        self.f_distance,
                        self.f_smooth,
                        self.f_diversity,
                        self.f_interaction_rate)

        self.frame_recorded: List[Dict] = []
        '''
            frame_record = [{
                'timestamp ': world_snapshot.timestamp,
                'frame': frame,
                'min_dis': min_dis,
                'unsmooth_acc': unsmooth_acc,
                'walker_in_road': walker_in_road,
                'npc_vehicles_ss': npc_vehicles_ss,
                'ego_ss': ego_ss
            }]
        '''

    def evaluate(self):
        # frame_start = self.frame_recorded[0]['frame']
        # frame_end = self.frame_recorded[-1]['frame']
        # frame_duration = frame_end - frame_start + 1
        if not self.frame_recorded or len(self.frame_recorded) == 0:

            self.walker_ind.fitness.values = (0, 0, 0, 0)
            self.vehicle_ind.fitness.values = (0, 0, 0, 0)
            self.is_evaluated = False
            logger.info(f'{self.id}, core = {(0, 0, 0, 0)}')
            return
        f_distance = self.frame_recorded[0]['min_dis']
        f_unsmooth_acc = self.frame_recorded[0]['unsmooth_acc']

        walker_in_road_count = 0
        vehicle_may_collide = 0

        for frame in self.frame_recorded:
            if frame == None:
                continue
            if frame['min_dis'] < f_distance:
                f_distance = frame['min_dis']

            if frame['unsmooth_acc'] > f_unsmooth_acc:
                f_unsmooth_acc = frame['unsmooth_acc']

            walker_in_road_count += frame['walker_in_road']

            ego_ss = frame['ego_ss']
            for vehicle_ss in frame['npc_vehicles_ss']:
                collision, c_t = self.predict_collision(vehicle_ss, ego_ss)
                if collision:
                    vehicle_may_collide += 1

        self.f_distance = f_distance
        self.f_smooth = f_unsmooth_acc
        self.f_crossing_time = walker_in_road_count / len(self.frame_recorded)
        self.f_interaction_rate = vehicle_may_collide / \
            len(self.frame_recorded)

        self.walker_ind.fitness.values = (self.f_distance,
                                          self.f_smooth,
                                          self.f_diversity,
                                          self.f_crossing_time)
        self.vehicle_ind.fitness.values = (self.f_distance,
                                           self.f_smooth,
                                           self.f_diversity,
                                           self.f_interaction_rate)
        logger.info(f'{self.id}, core = {(self.f_distance, self.f_smooth, self.f_diversity, self.f_crossing_time)}')
        self.is_evaluated = True

    def predict_collision(self,
                          actor1: carla.ActorSnapshot,
                          actor2: carla.ActorSnapshot,
                          prediction_time=1,
                          time_step=0.05,
                          collision_distance=2.0):
        if actor1 is None or actor2 is None:
            return False, None
        # Initial conditions
        position1 = actor1.get_transform().location
        velocity1 = actor1.get_velocity()
        acceleration1 = actor1.get_acceleration()

        position2 = actor2.get_transform().location
        velocity2 = actor2.get_velocity()
        acceleration2 = actor2.get_acceleration()

        for t in np.arange(0, prediction_time, time_step):
            # Update positions
            future_pos1 = carla.Location(
                x=position1.x + (velocity1.x * t) + 0.5
                * acceleration1.x * t**2,
                y=position1.y + (velocity1.y * t) + 0.5
                * acceleration1.y * t**2,
                z=position1.z + (velocity1.z * t) + 0.5
                * acceleration1.z * t**2
            )

            future_pos2 = carla.Location(
                x=position2.x + (velocity2.x * t) + 0.5
                * acceleration2.x * t**2,
                y=position2.y + (velocity2.y * t) + 0.5
                * acceleration2.y * t**2,
                z=position2.z + (velocity2.z * t) + 0.5
                * acceleration2.z * t**2
            )

            # Calculate distance between future positions
            dist = math.sqrt((future_pos1.x - future_pos2.x)**2
                             + (future_pos1.y - future_pos2.y)**2
                             + (future_pos1.z - future_pos2.z)**2)

            # Check for potential collision
            if dist < collision_distance:
                return True, t

        return False, None
