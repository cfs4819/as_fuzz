import random
import threading

from deap import base
from typing import List


class GeneNpcWalker:
    def __init__(self):
        self.start: dict = {'x': 0, 'y': 0, 'z': 0}  # "x": 0.0, "y": 0.0
        self.end: dict = {'x': 0, 'y': 0, 'z': 0}  # "x": 0.0, "y": 0.0
        self.start_time: float = 0.0
        self.max_speed: float = 1.4
        self.status: int = 0  # 0: walking, 1: stopped


class GeneNpcVehicle:
    def __init__(self):
        self.start: dict = {'x': 0, 'y': 0, 'z': 0}  # "x": 0.0, "y": 0.0
        self.end: dict = {'x': 0, 'y': 0, 'z': 0}  # "x": 0.0, "y": 0.0
        self.start_time: float = 0.0

        self.vehicle_type: int = 0
        # 0: Car, 1: Truck, 2: Van, 3: Motorcycle, 4: Bicycle.

        self.initial_speed: float = 0.0
        self.status: int = 0  # 0: driving, 1: starting, 2: parked.
        self.agent_type: int = 0  # 0: normal, 1: cautious, 2: aggressive.
        # self.max_speed:float = 10.0
        # self.status:int = 0     # 0: walking, 1: stopped


class WalkerListFitness(base.Fitness):
    """
    Class to represent weight of each fitness function
    (f_distance, f_smooth, f_diversity, f_crossing_time)
    """
    # minimize closest distance between pair of ADC
    # maximize number of unique decisions being made
    # maximize pairs of conflict trajectory
    # maximize unique violation
    weights = (-1.0, 1.0, 1.0, 1.0)


class VehicleListFitness(base.Fitness):
    """
    Class to represent weight of each fitness function
    (f_distance, f_smooth, f_diversity, f_interaction_rate)
    """
    # minimize closest distance between pair of ADC
    # maximize number of unique decisions being made
    # maximize pairs of conflict trajectory
    # maximize unique violation
    weights = (-1.0, 1.0, 1.0, 1.0)


class GeneNpcWalkerList:
    def __init__(self, id='', list: List[GeneNpcWalker] = [], max_count: int = 5):
        self.id = id  # gen_{}
        self.list: List[GeneNpcWalker] = list

        self.max_walker_count = max_count
        self.fitness: base.Fitness = WalkerListFitness()

    def get_a_new_agent(self, scene_width=30, scene_length=30) -> GeneNpcWalker:
        # generate a random start position and end position
        start_x = random.uniform(-scene_length / 2, scene_length / 2)
        start_y = random.uniform(-scene_width / 2, scene_width / 2)
        end_x = random.uniform(-scene_length / 2, scene_length / 2 * 3)
        end_y = random.uniform(-scene_width / 2, scene_width / 2)

        new_walker = GeneNpcWalker()
        new_walker.start = {'x': start_x, 'y': start_y, 'z': 0}
        new_walker.end = {'x': end_x, 'y': end_y, 'z': 0}
        new_walker.start_time = random.uniform(0, 2)

        new_walker.status = random.choices([0, 1],
                                           weights=[0.7, 0.3], k=1)[0]

        if new_walker.status == 0:
            new_walker.max_speed = random.uniform(0, 3)

        return new_walker


class GeneNpcVehicleList:
    def __init__(self, id='', list: List[GeneNpcVehicle] = [], max_count: int = 5):
        self.id = id
        self.list: List[GeneNpcVehicle] = list

        self.max_vehicle_count = max_count
        self.fitness: base.Fitness = VehicleListFitness()

    def get_a_new_agent(self, scene_width=30, scene_length=30) -> GeneNpcVehicle:
        # generate a random start position and end position
        start_x = random.uniform(-scene_length / 2, scene_length / 2)
        start_y = random.uniform(-scene_width / 2, scene_width / 2)
        end_x = random.uniform(-scene_length / 2, scene_length / 2 * 3)
        end_y = random.uniform(-scene_width / 2, scene_width / 2)
        while abs(start_y - end_y) <= 5:
            end_y = random.uniform(-scene_width / 2, scene_width / 2)

        new_vehicle = GeneNpcVehicle()

        new_vehicle.start = {'x': start_x, 'y': start_y, 'z': 0}
        new_vehicle.end = {'x': end_x, 'y': end_y, 'z': 0}
        new_vehicle.start_time = random.uniform(0, 1)

        new_vehicle.vehicle_type = random.choices([0, 1, 2, 3],
                                                  weights=[0.4, 0.3, 0.2, 0.1], k=1)[0]

        new_vehicle.status = random.choices([0, 1, 2],
                                            weights=[0.6, 0.3, 0.1], k=1)[0]

        new_vehicle.agent_type = random.choices([0, 1, 2],
                                                weights=[0.6, 0.2, 0.2], k=1)[0]
        if new_vehicle.status == 0:
            new_vehicle.initial_speed = random.uniform(0, 20)

        return new_vehicle


def get_new_walker_ind(max_count: int = 5) -> GeneNpcWalkerList:
    ind = GeneNpcWalkerList(max_count=max_count)
    ind.list = []
    if max_count <= 1:
        ind.list.append(ind.get_a_new_agent())
        return ind
    for i in range(random.randint(1, int(max_count))):
        ind.list.append(ind.get_a_new_agent())
    return ind


def get_new_vehicle_ind(max_count: int = 5) -> GeneNpcVehicleList:
    ind = GeneNpcVehicleList(max_count=max_count)
    ind.list = []
    if max_count <= 1:
        ind.list.append(ind.get_a_new_agent())
        return ind
    for i in range(random.randint(1, int(max_count))):
        ind.list.append(ind.get_a_new_agent())
    return ind


def save_pop(vehicle_pop: List[GeneNpcVehicleList],
             walker_pop: List[GeneNpcWalkerList],
             file_path):
    pass


def load_pop(file_path):
    pass
