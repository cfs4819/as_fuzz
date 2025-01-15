import random
from deap import base, creator, tools, algorithms
from typing import List

from copy import deepcopy

from MS_fuzz.common.evaluate import Evaluate_Object
from MS_fuzz.ga_engine.gene import *

import threading
import queue
import time
import pickle
import os
import json
import copy
import datetime


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

        self.type_str = 'straight'
        self.road_type = 'straight'  # straight, junction
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

        self.pop_walkers = []
        self.pop_vehicles = []
        self.generation = 0

        self.gen_history = {}

        self.generation_file = None

    def set_generation_file(self, file_path):
        self.generation_file = file_path

    def save_generation(self):
        if self.generation_file == None or self.generation_file == '':
            return

        if not os.path.exists(self.generation_file):
            os.makedirs(self.generation_file)

        datetime_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.generation_file,
                            f'cega_{self.type_str}_his_{datetime_str}.json')

        result_dic = {}

        result_dic['type'] = self.type_str
        gen_his = {}
        for gen_name, gen_dic in self.gen_history.items():
            # geneartion
            pop_w_dic = []
            for ind_w in gen_dic['pop_w']:
                ind_w_dic = {}
                ind_fitness_value = ind_w.fitness.values
                ind_w_dic['fitness'] = {
                    'min_dis': ind_fitness_value[0],
                    'smooth': ind_fitness_value[1],
                    'diversity': ind_fitness_value[2],
                    'crossing_time': ind_fitness_value[3]
                }
                ind_w_dic['walkers'] = [
                    {
                        'start_p': w.start,
                        'start_t': w.start_time,
                        'max_speed': w.max_speed,
                        'status': w.status
                    } for w in ind_w.list
                ]

                pop_w_dic.append(ind_w_dic)

            pop_v_dic = []
            for ind_v in gen_dic['pop_v']:
                ind_v_dic = {}
                ind_fitness_value = ind_v.fitness.values
                ind_v_dic['fitness'] = {
                    'min_dis': ind_fitness_value[0],
                    'smooth': ind_fitness_value[1],
                    'diversity': ind_fitness_value[2],
                    'interaction_rate': ind_fitness_value[3]
                }
                ind_v_dic['vehicles'] = [
                    {
                        'start_p': v.start,
                        'start_t': v.start_time,
                        'end_p': v.end,
                        'vehicle_type': v.vehicle_type,
                        'status': v.status,
                        'agnet_type': v.agent_type
                    } for v in ind_v.list
                ]
                pop_v_dic.append(ind_v_dic)
            this_gen_dic = {
                # 'gen':gen_name,
                'pop_v': pop_v_dic,
                'pop_w': pop_w_dic
            }
            gen_his[gen_name] = this_gen_dic
        result_dic['gen_his'] = gen_his
        with open(path, 'w') as f:
            json.dump(result_dic, f, indent=4)

    def prase_road_type(self, type_str: str, traffic_density_coefficient=1):
        self.type_str = type_str
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

            self.ind_vehicle_max_count = 1.5 * self.lane_num

        self.ind_vehicle_max_count = self.ind_vehicle_max_count * \
            traffic_density_coefficient
        self.ind_walker_max_count = self.ind_walker_max_count * \
            traffic_density_coefficient

    def mate_walkers(self, ind1: GeneNpcWalkerList, ind2: GeneNpcWalkerList):
        offspring1 = GeneNpcWalkerList(max_count=self.ind_walker_max_count)
        offspring2 = GeneNpcWalkerList(max_count=self.ind_walker_max_count)

        for index in range(min(len(ind1.list), len(ind2.list))):
            if len(offspring1.list) >= self.ind_walker_max_count:
                break
            parent1 = ind1.list[index]
            parent2 = ind2.list[index]

            walker1 = GeneNpcWalker()
            walker2 = GeneNpcWalker()

            walker1.start = random.choice([parent1.start, parent2.start])
            walker2.start = random.choice([parent1.start, parent2.start])
            walker1.end = random.choice([parent1.end, parent2.end])
            walker2.end = random.choice([parent1.end, parent2.end])
            walker1.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            walker2.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            walker1.status = random.choice([parent1.status, parent2.status])
            walker2.status = random.choice([parent1.status, parent2.status])
            walker1.max_speed = random.choice(
                [parent1.max_speed, parent2.max_speed])
            walker2.max_speed = random.choice(
                [parent1.max_speed, parent2.max_speed])

            offspring1.list.append(walker1)
            offspring2.list.append(walker2)

        for index in range(min(len(ind1.list), len(ind2.list)), max(len(ind1.list), len(ind2.list))):
            if len(offspring1.list) >= self.ind_walker_max_count:
                break
            if len(ind1.list) > len(ind2.list):
                offspring1.list.append(deepcopy(ind1.list[index]))
                offspring2.list.append(deepcopy(ind1.list[index]))
            else:
                offspring1.list.append(deepcopy(ind2.list[index]))
                offspring2.list.append(deepcopy(ind2.list[index]))

        return offspring1, offspring2

    def mate_vehicles(self, ind1: GeneNpcVehicleList, ind2: GeneNpcVehicleList):
        offspring1 = GeneNpcVehicleList(max_count=self.ind_vehicle_max_count)
        offspring2 = GeneNpcVehicleList(max_count=self.ind_vehicle_max_count)

        for index in range(min(len(ind1.list), len(ind2.list))):
            if len(offspring1.list) >= self.ind_vehicle_max_count:
                break
            parent1 = ind1.list[index]
            parent2 = ind2.list[index]

            vehicle1 = GeneNpcVehicle()
            vehicle2 = GeneNpcVehicle()

            vehicle1.start = random.choice([parent1.start, parent2.start])
            vehicle2.start = random.choice([parent1.start, parent2.start])
            vehicle1.end = random.choice([parent1.end, parent2.end])
            vehicle2.end = random.choice([parent1.end, parent2.end])
            vehicle1.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            vehicle2.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            vehicle1.agent_type = random.choice(
                [parent1.agent_type, parent2.agent_type])
            vehicle2.agent_type = random.choice(
                [parent1.agent_type, parent2.agent_type])
            vehicle1.status = random.choice([parent1.status, parent2.status])
            vehicle2.status = random.choice([parent1.status, parent2.status])

            if vehicle1.status == 0:
                vehicle1.initial_speed = random.choice(
                    [parent1.initial_speed, parent2.initial_speed])
            if vehicle2.status == 0:
                vehicle2.initial_speed = random.choice(
                    [parent1.initial_speed, parent2.initial_speed])

            offspring1.list.append(vehicle1)
            offspring2.list.append(vehicle2)

        for index in range(min(len(ind1.list), len(ind2.list)), max(len(ind1.list), len(ind2.list))):
            if len(offspring1.list) >= self.ind_vehicle_max_count:
                break
            if len(ind1.list) > len(ind2.list):
                offspring1.list.append(deepcopy(ind1.list[index]))
                offspring2.list.append(deepcopy(ind1.list[index]))
            else:
                offspring1.list.append(deepcopy(ind2.list[index]))
                offspring2.list.append(deepcopy(ind2.list[index]))

        return offspring1, offspring2

    def mutate_walkers(self, ind: GeneNpcWalkerList):
        mut_pb = random.random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return (ind,)

        # add a random agent, p = 0.3
        elif ((mut_pb <= 0.2 + 0.3 and len(ind.list) < ind.max_walker_count)
              or len(ind.list) < 1):
            ind.list.append(ind.get_a_new_agent(
                self.scene_width, self.scene_length))
            return (ind,)

        # mutate a random agent, p = 0.5
        else:
            ind.list.remove(random.choice(ind.list))
            start_x = random.uniform(-self.scene_length
                                     / 2, self.scene_length / 2)
            start_y = random.uniform(-self.scene_width
                                     / 2, self.scene_width / 2)
            end_x = random.uniform(-self.scene_length / 2,
                                   self.scene_length / 2 * 3)
            end_y = random.uniform(-self.scene_width / 2, self.scene_width / 2)

            new_walker = GeneNpcWalker()
            new_walker.start = {'x': start_x, 'y': start_y, 'z': 0}
            new_walker.end = {'x': end_x, 'y': end_y, 'z': 0}

            new_walker.start_time = random.uniform(0, 2)

            new_walker.status = random.choices([0, 1],
                                               weights=[0.7, 0.3], k=1)[0]

            if new_walker.status == 0:
                new_walker.max_speed = random.uniform(0, 3)

            ind.list.append(new_walker)
            return (ind,)

    def mutate_vehicles(self, ind: GeneNpcVehicleList):
        mut_pb = random.random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return (ind,)

        # add a random vehicle, p = 0.3
        elif ((mut_pb <= 0.2 + 0.3 and len(ind.list) < ind.max_vehicle_count)
              or len(ind.list) < 1):
            ind.list.append(ind.get_a_new_agent(self.scene_width,
                                                self.scene_length))
            return (ind,)

        # mutate a random agent, p = 0.5
        else:
            ind_2_mutate = random.choice(ind.list)
            parameters_2_mutate_list = ['start', 'end', 'start_time',
                                        'vehicle_type', 'initial_speed',
                                        'status', 'agent_type']
            mutate_weight = [0.2, 0.2, 0.1,
                             0.2, 0.1,
                             0.0, 0.2]
            # select 3 parameters to mutate
            parameters_2_mutate = random.choices(parameters_2_mutate_list,
                                                 weights=mutate_weight,
                                                 k=3)
            if 'start' in parameters_2_mutate:
                start_x = random.uniform(-self.scene_length / 2,
                                         self.scene_length / 2)
                start_y = random.uniform(-self.scene_width / 2,
                                         self.scene_width / 2)
                ind_2_mutate.start = {'x': start_x, 'y': start_y, 'z': 0}
            if 'end' in parameters_2_mutate:
                end_x = random.uniform(-self.scene_length / 2,
                                       self.scene_length / 2 * 3)
                end_y = random.uniform(-self.scene_width
                                       / 2, self.scene_width / 2)
                while abs(ind_2_mutate.start['y'] - end_y) <= 5:
                    end_y = random.uniform(-self.scene_width / 2,
                                           self.scene_width / 2)
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

            return (ind,)

    def start(self):
        self.main_thread = threading.Thread(target=self.main_progress)
        self.stop_event = threading.Event()
        self.stop_event.clear()
        self.main_thread.start()
        self.running = True

    def stop(self):
        if not self.running:
            return
        self.stop_event.set()
        self.main_thread.join()
        # self.save(f'CEGA_checkpoint_{self.type_str}.pkl')
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

        if self.generation == 0:
            self.pop_walkers: List[GeneNpcWalkerList] = [
                get_new_walker_ind(max_count=self.ind_walker_max_count) for _ in range(POP_SIZE)]
            self.pop_vehicles: List[GeneNpcVehicleList] = [
                get_new_vehicle_ind(max_count=self.ind_vehicle_max_count) for _ in range(POP_SIZE)]

        for index, c in enumerate(self.pop_walkers):
            c.id = f'gen_{self.generation}_ind_{index}'
        for index, c in enumerate(self.pop_vehicles):
            c.id = f'gen_{self.generation}_ind_{index}'

        hof_walkers = tools.ParetoFront()
        hof_vehicles = tools.ParetoFront()

        # Evaluate Initial Population
        if self.generation == 0:
            if self.logger:
                self.logger.info(
                    f' ====== Analyzing Initial Population ====== ')

            self.evaluate_pop(self.pop_walkers, self.pop_vehicles)

            hof_walkers.update(self.pop_walkers)
            hof_vehicles.update(self.pop_vehicles)

        for gen in range(self.generation, self.max_generation):
            self.generation = gen
            if self.stop_event.is_set():
                return
            if self.logger:
                self.logger.info(f"Generation #{gen}. Start:")

            # Vary the population
            offspring_walkers: List[GeneNpcWalkerList] = algorithms.varOr(
                self.pop_walkers, tb_walkers, OFF_SIZE, CXPB, MUTPB)
            offspring_vehicles: List[GeneNpcVehicleList] = algorithms.varOr(
                self.pop_vehicles, tb_vehicles, OFF_SIZE, CXPB, MUTPB)

            for index, c in enumerate(offspring_walkers):
                c.id = f'gen_{gen}_ind_{index}'
            for index, c in enumerate(offspring_vehicles):
                c.id = f'gen_{gen}_ind_{index}'

            self.evaluate_pop(offspring_walkers, offspring_vehicles)

            hof_walkers.update(offspring_walkers)
            hof_vehicles.update(offspring_vehicles)

            # save generation
            last_gen_w = copy.deepcopy(self.pop_walkers)
            last_gen_v = copy.deepcopy(self.pop_vehicles)
            self.gen_history[f'gen_{gen}'] = {
                'pop_w': last_gen_w,
                'pop_v': last_gen_v
            }

            if self.generation_file:
                self.logger.info(
                    f'Saving generation {gen} to {self.generation_file}')
                self.save_generation()

            # population update
            self.pop_walkers[:] = tb_walkers.select(self.pop_walkers + offspring_walkers,
                                                    POP_SIZE)
            self.pop_vehicles[:] = tb_vehicles.select(self.pop_vehicles + offspring_vehicles,
                                                      POP_SIZE)

    def evaluate_pop(self, pop_walkers: List[GeneNpcWalkerList],
                     pop_vehicles: List[GeneNpcVehicleList]):

        pop_size = min(len(pop_walkers), len(pop_vehicles))

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
        print('[evaluate_pop] evaluate_list:', len(self.evaluate_list))
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
            if not obj.is_evaluated and not obj.is_in_queue:
                obj.is_in_queue = True
                return obj
        # if all objects are in queue, check again
        for obj in self.evaluate_list:
            if not obj.is_evaluated and obj.is_in_queue:
                return obj
        return None

    def save(self, filename):
        print(f'cega {self.type_str} saving at {filename}')
        with open(filename, 'wb') as f:
            state = {
                'max_generation': self.max_generation,
                'num_individuals': self.num_individuals,
                'scene_length': self.scene_length,
                'scene_width': self.scene_width,
                'type_str': self.type_str,
                'road_type': self.road_type,
                'way_num': self.way_num,
                'lane_num': self.lane_num,
                'junction_size': self.junction_size,
                'junction_dir_num': self.junction_dir_num,
                'ind_vehicle_max_count': self.ind_vehicle_max_count,
                'ind_walker_max_count': self.ind_walker_max_count,
                'evaluate_list': self.evaluate_list,
                'pop_walkers': self.pop_walkers,
                'pop_vehicles': self.pop_vehicles,
                'generation': self.generation,
                'gen_history': self.gen_history
            }
            pickle.dump(state, f)

    def load_from_file(self, filename):
        try:
            with open(filename, 'rb') as f:
                state = pickle.load(f)
                self.max_generation = state['max_generation']
                self.num_individuals = state['num_individuals']
                self.scene_length = state['scene_length']
                self.scene_width = state['scene_width']
                self.type_str = state['type_str']
                self.road_type = state['road_type']
                self.way_num = state['way_num']
                self.lane_num = state['lane_num']
                self.junction_size = state['junction_size']
                self.junction_dir_num = state['junction_dir_num']
                self.ind_vehicle_max_count = state['ind_vehicle_max_count']
                self.ind_walker_max_count = state['ind_walker_max_count']
                self.evaluate_list = state['evaluate_list']
                self.pop_walkers = state['pop_walkers']
                self.pop_vehicles = state['pop_vehicles']
                self.generation = state['generation']
                self.gen_history = state['gen_history']

            if self.logger:
                self.logger.info('Loaded checkpoint successfully.')
        except Exception as e:
            if self.logger:
                self.logger.info(
                    'No checkpoint found or failed to load checkpoint.')
