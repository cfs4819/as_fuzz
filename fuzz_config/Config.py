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
        self.score_dir = None
        self.rosbag_dir = None
        self.cam_dir = None
        self.trace_dir = None
        self.meta_file = None
        self.error_dir = None
        self.queue_dir = None
        self.debug = True

        # simulator config
        self.sim_host = '172.17.0.1'
        self.sim_port = 5000
        self.load_world_timeout = 20
        self.frame_rate = 10

        # self.carla_map = "Town10hd"
        self.carla_map = "Town04"

        # carla bridge config
        self.load_bridge = True

        # dreamview config
        # self.dreamview_map = "Carla Town10hd"
        self.dreamview_map = "Carla Town04"
        self.dreamview_vehicle = "Lincoln2017MKZ LGSVL"
        self.dreamview_ip  = "localhost"
        self.dreamview_port  = 8888

        # Fuzzer config
        self.topo_k = 2
        self.immobile_percentage = 0  # the percentage of the actors is immobile forever
        self.max_cycles = 0
        self.max_mutation = 0
        self.num_dry_runs = 1
        self.density = 1
        self.num_mutation_car = 1
        self.density = 1
        self.no_traffic_lights = False

        # Fuzzing metadata
        self.town = None
        self.cur_time = None
        self.determ_seed = None
        self.out_dir = None
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

        # Functional testing
        self.function = "general"

    def set_paths(self):
        self.queue_dir = os.path.join(self.out_dir, "queue")
        self.error_dir = os.path.join(self.out_dir, "errors")
        self.meta_file = os.path.join(self.out_dir, "meta")
        self.cam_dir = os.path.join(self.out_dir, "camera")
        self.trace_dir = os.path.join(self.out_dir, "trace")
        self.rosbag_dir = os.path.join(self.out_dir, "rosbags")

    # def enqueue_seed_scenarios(self):
    #     try:
    #         seed_scenarios = os.listdir(self.seed_dir)
    #     except:
    #         print("[-] Error - cannot find seed directory ({})".format(self.seed_dir))
    #         sys.exit(-1)
    #
    #     queue = [seed for seed in seed_scenarios if not seed.startswith(".")
    #              and seed.endswith(".json")]
    #
    #     return queue
