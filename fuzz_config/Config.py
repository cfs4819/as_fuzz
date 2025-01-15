import os
import sys


def get_proj_root():
    config_path = os.path.abspath(__file__)
    src_dir = os.path.dirname(config_path)
    proj_root = os.path.dirname(src_dir)
    return proj_root


class Config:
    """
    A class defining fuzzing configuration and helper methods.
    An instance of this class should be created by the main module (fuzzer.py)
    and then be shared across other modules as a context handler.
    """

    def __init__(self):
        self.debug = True

        # simulator config
        self.sim_host = '172.17.0.1'
        self.sim_port = 4000
        self.load_world_timeout = 10

        self.carla_map = "Town10hd"
        # self.carla_map = "Town04"

        # carla bridge config
        self.load_bridge = True

        # dreamview config
        self.dreamview_map = "Carla Town10hd"
        # self.dreamview_map = "Carla Town04"
        self.dreamview_vehicle = "Lincoln2017MKZ LGSVL"
        self.dreamview_ip = "localhost"
        self.dreamview_port = 8888
        self.dreamview_map_dic = {
            'town01': 'Carla Town01',
            'Town01': 'Carla Town01',

            'town02': 'Carla Town02',
            'Town02': 'Carla Town02',

            'town04': 'Carla Town04',
            'Town04': 'Carla Town04',

            'town07': 'Carla Town07',
            'Town07': 'Carla Town07',

            'town10hd': 'Carla Town10hd',
            'Town10hd': 'Carla Town10hd'
        }

        # Fuzzer config
        self.scenario_length = 30
        self.scenario_width = 30
        self.no_traffic_lights = False
        self.try_relese_block = True
        
        # GA config        
        self.density = 1

        # Fuzzing metadata
        self.town = None
        self.town_name = {
            "1": "Town01",
            "2": "Town02",
            "4": "Town04",
            "7": "Town07",
            "10": "Town10hd",
        }
        self.out_dir = '/apollo/data/MS_fuzz/result'
        self.seed_dir = None

        # Target config
        # self.agent_type = c.AUTOWARE  # c.AUTOWARE

        # Enable/disable Various Checks
        self.check_dict = {
            "speed": True,
            "lane": True,
            "crash": True,
            "stuck": True,
            "red": True,
            "other": True,
        }


