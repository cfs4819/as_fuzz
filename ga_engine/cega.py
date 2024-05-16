import random
from deap import base, creator, tools, algorithms
from typing import List

from copy import deepcopy

from MS_fuzz.common.evaluate import Evaluate_Object
from MS_fuzz.ga_engine.gene import *

import threading
import queue
import time


class CEGA:
    def __init__(self,
                 scenario_length,
                 scenario_width,
                 logger=None):

        self.max_generation = 100
        self.num_individuals = 10

        self.logger = logger

        self.scene_length = scenario_length
        self.scene_width = scenario_width

        self.type_str = 'straight'  # straight, junctio
        self.road_type = 'straight'
        self.way_num = 2
        self.lane_num = 2
        self.junction_size = 'small'
        self.junction_dir_num = 3

        self.ind_vehicle_max_count = 3
        self.ind_walker_max_count = 3

        self.evaluate_list: List[Evaluate_Object] = []

        self.running = False

        self.main_thread: threading.Thread = None
        self.stop_event: threading.Event = None

    def prase_road_type(self, type_str: str):
        str_seg = type_str.split('_')

        if str_seg[0] == 'junction':
            # for example 'junction_medium_3_dir', by default small_3_dir
            self.road_type = str_seg[0]
            self.junction_size = str_seg[1]
            self.junction_dir_num = int(
                str_seg[2]) if str_seg[2] != '-1' else 3

            if self.junction_size == 'small':
                self.ind_vehicle_max_count = 3
                self.ind_walker_max_count = 4
            elif self.junction_size == 'medium':
                self.ind_vehicle_max_count = 4
                self.ind_walker_max_count = 5
            elif self.junction_size == 'large':
                self.ind_vehicle_max_count = 5
                self.ind_walker_max_count = 6

        elif str_seg[0] == 'straight':
            # for example 'straight_2_way_8_lane', by default 2_way_2_lane
            self.road_type = str_seg[0]
            self.way_num = int(str_seg[1]) if str_seg[1] != '-1' else 2
            self.lane_num = int(str_seg[3]) if str_seg[3] != '-1' else 2

            self.ind_vehicle_max_count = 1.5*self.lane_num

    def evaluate(self, walker_ind: GeneNpcWalkerList, vehicle_ind: GeneNpcVehicleList):
        '''
            1. Target:
                wait for evaluate result
                then return <walker_fitness, vehicle_fitness>
            2. for all individuals:
                f_distance: the minimum distance between ego and other npcs during the simulation time t
                f_smooth : represents the ego vehicle's acceleration during a scene
                f_diversity: diversity of the scene
            3. for npc_walkers:
                f_crossing_time : Time taken to cross the road
            4. for npc_vehicles:s
                f_interaction_rate: the rate at which vehicles interact with the ego vehicle
        '''
        evaluate_obj = Evaluate_Object(walker_ind, vehicle_ind)

        self.evaluate_list.append(evaluate_obj)

        while not self.stop_event.is_set():
            # wait for result
            if evaluate_obj.is_evaluated:
                return evaluate_obj.fitness
        return -1, -1, -1, -1, -1

    def mate_walkers(self, ind1: GeneNpcWalkerList, ind2: GeneNpcWalkerList):
        offspring = GeneNpcWalkerList(max_count=self.ind_walker_max_count)

        for index in range(min(len(ind1.list), len(ind2.list))):
            parent1 = ind1.list[index]
            parent2 = ind2.list[index]

            walker = GeneNpcWalker()
            walker.start = random.choice([parent1.start, parent2.start])
            walker.end = random.choice([parent1.end, parent2.end])
            walker.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            walker.status = random.choice([parent1.status, parent2.status])
            walker.max_speed = random.choice(
                [parent1.max_speed, parent2.max_speed])

            offspring.list.append(walker)

        for index in range(min(len(ind1.list), len(ind2.list)),
                           max(len(ind1.list), len(ind2.list))):
            if len(ind1.list) > len(ind2.list):
                offspring.list.append(deepcopy(ind1.list[index]))
            else:
                offspring.list.append(deepcopy(ind2.list[index]))

        return offspring

    def mate_vehicles(self, ind1: GeneNpcVehicleList, ind2: GeneNpcVehicleList):
        offspring = GeneNpcVehicleList(max_count=self.ind_vehicle_max_count)

        for index in range(min(len(ind1.list), len(ind2.list))):
            parent1 = ind1.list[index]
            parent2 = ind2.list[index]

            vehicle = GeneNpcVehicle()

            vehicle.start = random.choice([parent1.start, parent2.start])
            vehicle.end = random.choice([parent1.end, parent2.end])
            vehicle.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            vehicle.agent_type = random.choice(
                [parent1.agent_type, parent2.agent_type])
            vehicle.status = random.choice([parent1.status, parent2.status])

            if vehicle.status == 0:
                vehicle.initial_speed = random.choice(
                    [parent1.initial_speed, parent2.initial_speed])

            offspring.list.append(vehicle)

        for index in range(min(len(ind1.list), len(ind2.list)),
                           max(len(ind1.list), len(ind2.list))):
            if len(ind1.list) > len(ind2.list):
                offspring.list.append(deepcopy(ind1.list[index]))
            else:
                offspring.list.append(deepcopy(ind2.list[index]))

        return offspring

    def mutate_walkers(self, ind: GeneNpcWalkerList):
        mut_pb = random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return ind

        # add a random agent, p = 0.3
        elif ((mut_pb <= 0.2 + 0.3 and len(ind.list) < ind.max_walker_count) or
              len(ind.list) < 1):
            ind.list.append(ind.get_a_new_agent(
                self.scene_width, self.scene_length))
            return ind

        # mutate a random agent, p = 0.5
        else:
            ind.list.remove(random.choice(ind.list))
            start_x = random.uniform(-self.scene_length/2, self.scene_length/2)
            start_y = random.uniform(-self.scene_width/2, self.scene_width/2)
            end_x = random.uniform(-self.scene_length/2, self.scene_length/2*3)
            end_y = random.uniform(-self.scene_width/2, self.scene_width/2)

            new_walker = GeneNpcWalker()
            new_walker.start = {'x': start_x, 'y': start_y, 'z': 0}
            new_walker.end = {'x': end_x, 'y': end_y, 'z': 0}

            new_walker.start_time = random.uniform(0, 2)

            new_walker.status = random.choices([0, 1],
                                               weights=[0.7, 0.3], k=1)[0]

            if new_walker.status == 0:
                new_walker.max_speed = random.uniform(0, 3)

            ind.list.append(new_walker)
            return ind

    def mutate_vehicles(self, ind: GeneNpcVehicleList):
        mut_pb = random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return ind

        # add a random vehicle, p = 0.3
        elif ((mut_pb <= 0.2 + 0.3 and len(ind.list) < ind.max_walker_count) or
              len(ind.list) < 1):
            ind.list.append(ind.get_a_new_agent(self.scene_width,
                                                self.scene_length))
            return ind

        # mutate a random agent, p = 0.5
        else:
            ind_2_mutate = random.choice(ind.list)
            parameters_2_mutate_list = ['start', 'end', 'start_time',
                                        'vehicle_type', 'initial_speed',
                                        'status', 'agent_type']
            mutete_weight = [0.2, 0.2, 0.1,
                             0.2, 0.1,
                             0.0, 0.2]
            # select 3 parameters to mutate
            parameters_2_mutate = random.choices(parameters_2_mutate_list,
                                                 weights=mutete_weight,
                                                 k=3)
            if 'start' in parameters_2_mutate:
                start_x = random.uniform(-self.scene_length/2,
                                         self.scene_length/2)
                start_y = random.uniform(-self.scene_width/2,
                                         self.scene_width/2)
                ind_2_mutate.start = {'x': start_x, 'y': start_y, 'z': 0}
            if 'end' in parameters_2_mutate:
                end_x = random.uniform(-self.scene_length/2,
                                       self.scene_length/2*3)
                end_y = random.uniform(-self.scene_width/2, self.scene_width/2)
                while abs(ind_2_mutate.start['y'] - end_y) <= 5:
                    end_y = random.uniform(-self.scene_width/2,
                                           self.scene_width/2)
                ind_2_mutate.end = {'x': end_x, 'y': end_y, 'z': 0}
            if 'start_time' in parameters_2_mutate:
                ind_2_mutate.start_time = random.uniform(0, 2)
            if 'vehicle_type' in parameters_2_mutate:
                ind_2_mutate.vehicle_type = random.choices([0, 1, 2, 3],
                                                           weights=[
                                                               0.4, 0.3, 0.2, 0.1],
                                                           k=1)[0]
            if 'status' in parameters_2_mutate:
                ind_2_mutate.status = random.choices([0, 1, 2],
                                                     weights=[0.6, 0.3, 0.1], k=1)[0]
            if 'initial_speed' in parameters_2_mutate and ind_2_mutate.status == 0:
                ind_2_mutate.initial_speed = random.uniform(0, 20)
            if 'agent_type' in parameters_2_mutate:
                ind_2_mutate.agent_type = random.choices([0, 1, 2],
                                                         weights=[0.6, 0.2, 0.2], k=1)[0]

            return ind

    def start(self):
        self.main_thread = threading.Thread(target=self.main_progress)
        self.stop_event = threading.Event()
        self.stop_event.clear()
        self.main_thread.start()
        self.running = True

    def stop(self):
        self.stop_event.set()
        self.main_thread.join()
        self.running = False

    def main_progress(self):
        if self.logger:
            self.logger.info(f"Start GA {self.type_str}:")

        # GA Hyperparameters
        # POP_SIZE = 10   # number of population
        # OFF_SIZE = 10   # number of offspring to produce
        # CXPB = 0.6      # crossover probability
        # MUTPB = 0.4     # mutation probability

        # number of population
        POP_SIZE = 8 if self.road_type == 'straight' else 4
        # number of offspring to produce
        OFF_SIZE = 8 if self.road_type == 'straight' else 4
        # crossover probability
        CXPB = 0.6
        # mutation probability
        MUTPB = 0.4

        # Co-evolutionary Genetic Algorithm
        tb_walkers = base.Toolbox()
        tb_vehicles = base.Toolbox()

        tb_walkers.register('mate', self.mate_walkers)
        tb_walkers.register('mutate', self.mutate_walkers)
        tb_walkers.register('select', tools.selNSGA2)

        tb_vehicles.register('mate', self.mate_vehicles)
        tb_vehicles.register('mutate', self.mutate_vehicles)
        tb_vehicles.register('select', tools.selNSGA2)

        pop_walkers: List[GeneNpcWalkerList] = [
            get_new_walker_ind(max_count=self.ind_walker_max_count) for _ in range(POP_SIZE)]
        pop_vehicles: List[GeneNpcVehicleList] = [
            get_new_vehicle_ind(max_count=self.ind_vehicle_max_count) for _ in range(POP_SIZE)]

        for index, c in enumerate(pop_walkers):
            c.id = f'gen_0_ind_{index}'
            # print(f'{c.id}, {c.max_walker_count}, {len(c.list)}')
        for index, c in enumerate(pop_vehicles):
            c.id = f'gen_0_ind_{index}'
            # print(f'{c.id}, {c.max_vehicle_count}, {len(c.list)}')

        hof_walkers = tools.ParetoFront()
        hof_vehicles = tools.ParetoFront()

        # Evaluate Initial Population
        if self.logger:
            self.logger.info(f' ====== Analyzing Initial Population ====== ')

        self.evaluate_pop(pop_walkers, pop_vehicles)

        hof_walkers.update(pop_walkers)
        hof_vehicles.update(pop_vehicles)

        for gen in range(1, self.max_generation):
            if self.stop_event.is_set():
                return
            if self.logger:
                self.logger.info(f"Generation #{gen}. Start:")

            # Vary the population
            offspring_walkers: List[GeneNpcWalkerList] = algorithms.varOr(
                pop_walkers, tb_walkers, OFF_SIZE, CXPB, MUTPB)
            offspring_vehicles: List[GeneNpcVehicleList] = algorithms.varOr(
                pop_vehicles, tb_vehicles, OFF_SIZE, CXPB, MUTPB)

            for index, c in enumerate(offspring_walkers):
                c.id = f'gen_{gen}_ind_{index}'
            for index, c in enumerate(offspring_vehicles):
                c.id = f'gen_{gen}_ind_{index}'

            # Evaluate the individuals with an invalid fitness
            self.evaluate_pop(offspring_walkers, offspring_vehicles)

            hof_walkers.update(offspring_walkers)
            hof_vehicles.update(offspring_vehicles)

            # population update
            pop_walkers[:] = tb_walkers.select(
                pop_walkers + offspring_walkers, POP_SIZE)
            pop_vehicles[:] = tb_vehicles.select(
                pop_vehicles + offspring_vehicles, POP_SIZE)

    def evaluate_pop(self, pop_walkers: List[GeneNpcWalkerList],
                     pop_vehicles: List[GeneNpcVehicleList]):
        
        pop_size = len(pop_walkers) if len(pop_walkers) < len(
            pop_vehicles) else len(pop_vehicles)

        for index in range(pop_size):
            walker_ind = pop_walkers[index]
            vehicle_ind = pop_vehicles[index]

            # only those individuals with invalid fitness need to be evaluated
            if walker_ind.fitness.valid and vehicle_ind.fitness.valid:
                continue

            # or add them to evaluate list
            if self.stop_event.is_set():
                # save if needed
                return

            evaluate_obj = Evaluate_Object(walker_ind, 
                                           vehicle_ind, 
                                           id=(walker_ind.id, vehicle_ind.id))
            self.evaluate_list.append(evaluate_obj)

        # wait until all evaluate_obj are evaluated
        all_evaluated = False
        while not all_evaluated:
            all_evaluated = True
            for obj in self.evaluate_list:
                if not obj.is_evaluated:
                    all_evaluated = False
                    break
            if not all_evaluated:
                time.sleep(0.01)
            if self.stop_event.is_set():
                break

        # reset self.evaluate_list after all evaluated
        self.evaluate_list.clear()

    def get_an_unevaluated_obj(self):
        while len(self.evaluate_list) == 0:
            if self.stop_event.is_set():
                return None
            time.sleep(0.01)
        for obj in self.evaluate_list:
            if not obj.is_evaluated:
                return obj
        return None

    def save(self):
        pass
