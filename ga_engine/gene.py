import carla
import threading
from agents.navigation.behavior_agent import BehaviorAgent

from typing import List


class GeneNpcWalker:
    def __init__(self):
        self.start: dict = {'x': 0, 'y': 0, 'z': 0}    # "x": 0.0, "y": 0.0
        self.end: dict = {'x': 0, 'y': 0, 'z': 0}      # "x": 0.0, "y": 0.0
        self.start_time: float = 0.0
        self.max_speed: float = 1.4
        self.status: int = 0     # 0: walking, 1: stopped


class GeneNpcVehicle:
    def __init__(self):
        self.start: dict = {'x': 0, 'y': 0, 'z': 0}    # "x": 0.0, "y": 0.0
        self.end: dict = {'x': 0, 'y': 0, 'z': 0}      # "x": 0.0, "y": 0.0
        self.start_time: float = 0.0
        # 0: Car, 1: Truck, 2: Van, 3: Motorcycle, 4: Bicycle.
        
        self.vehicle_type: int = 0
        self.initial_speed: float = 0.0
        self.status: int = 0     # 0: driving, 1: starting, 2: parked.
        self.agent_type: int = 0  # 0: normal, 1: cautious, 2: aggressive.
        # self.max_speed:float = 10.0
        # self.status:int = 0     # 0: walking, 1: stopped


class GeneNpcWalkerList:
    def __init__(self, id, list: List[GeneNpcWalker], max_count: int = 20):
        self.id = id
        self.list = list

        self.max_walker_count = max_count


class GeneNpcVehicleList:
    def __init__(self, id, list: List[GeneNpcVehicle], max_count: int = 20):
        self.id = id
        self.list = list

        self.max_vehicle_count = max_count
