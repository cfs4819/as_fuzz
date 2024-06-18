import os, sys, glob
import constants as c


def get_proj_root():
    config_path = os.path.abspath(__file__)
    src_dir = os.path.dirname(config_path)
    proj_root = os.path.dirname(src_dir)

    return proj_root


def set_carla_api_path():
    pass
    return
    proj_root = get_proj_root()

    dist_path = os.path.join(proj_root, "carla/PythonAPI/carla/dist")
    glob_path = os.path.join(dist_path, "carla-*%d.%d-%s.egg" % (
        sys.version_info.major,
        sys.version_info.minor,
        "win-amd64" if os.name == "nt" else "linux-x86_64"
    ))

    try:
        api_path = glob.glob(glob_path)[0]
    except IndexError:
        print("Couldn't set Carla API path.")
        exit(-1)

    if api_path not in sys.path:
        sys.path.append(api_path)
        print(f"API: {api_path}")


class Config:
    """
    A class defining fuzzing configuration and helper methods.
    An instance of this class should be created by the main module (fuzzer.py)
    and then be shared across other modules as a context handler.
    """

    def __init__(self):
        self.debug = False

        # reload carla port 
        self.reload_carla_port = 7691
        
        # simulator config
        self.sim_host = "localhost"
        self.sim_port = 2000
        self.sim_tm_port = 8000
        self.town = 3

        # Fuzzer config
        self.max_cycles = 0         # Maximum number of loops, default=10
        self.max_mutations = 0       # Size of the mutated population per cycle, default=8
        self.num_dry_runs = 1
        self.num_param_mutations = 1
        self.timeout = 60
        # self.initial_quota = 10

        # Fuzzing metadata
        self.cur_time = None
        self.determ_seed = None     # Set seed num for deterministic mutation
        self.out_dir = None         # Directory to save fuzzing logs, default="output"
        self.seed_dir = None        # Seed directory, default="seed"

        # Apollo config
        self.dreamview_ip = "localhost"
        self.dreamview_port = 8888
        self.dreamview_vehicle = "Lincoln2017MKZ LGSVL"
        self.dreamview_map = {
            'town01':   'Carla Town01',
            'Town01':   'Carla Town01',
            
            'town02':   'Carla Town02',
            'Town02':   'Carla Town02',
            
            'town04':   'Carla Town04',
            'Town04':   'Carla Town04',
            
            'town07':   'Carla Town07',
            'Town07':   'Carla Town07',
            
            'town10hd': 'Carla Town10hd',
            'Town10hd': 'Carla Town10hd'
        }
        self.apollo_modules =  [
            # 'Localization',  # ok
            'Transform',  # ok
            'Routing',
            'Prediction',  # ok
            'Planning',  # ok
            'Control',
            'Storytelling'  # ok
        ]
        
        # Target config
        self.agent_type = c.APOLLO  # c.APOLLO

        # Enable/disable Various Checks
        self.check_dict = {
            "speed": False,         # no_speed_check
            "lane": False,
            "crash": True,
            "stuck": True,
            "red": False,           # no_red_check
            "other": False,         # no_other_check
        }

        # Functional testing
        self.function = "general"   # Functionality to test (general / collision / traction), default="general"

        self.strategy = c.ALL       # Input mutation strategy (all / congestion / entropy / instability / trajectory)
        
        # Sim-debug settings
        self.view = c.BIRDSEYE

    def set_paths(self):
        self.queue_dir = os.path.join(self.out_dir, "queue")
        self.error_dir = os.path.join(self.out_dir, "errors")
        self.cov_dir = os.path.join(self.out_dir, "cov")
        self.meta_file = os.path.join(self.out_dir, "meta")
        self.cam_dir = os.path.join(self.out_dir, "camera")
        self.rosbag_dir = os.path.join(self.out_dir, "rosbags")
        self.score_dir = os.path.join(self.out_dir, "scores")

    def enqueue_seed_scenarios(self):
        try:
            seed_scenarios = os.listdir(self.seed_dir)
        except:
            print("[-] Error - cannot find seed directory ({})".format(self.seed_dir))
            sys.exit(-1)

        queue = [seed for seed in seed_scenarios if not seed.startswith(".")
                and seed.endswith(".json")]

        return queue

