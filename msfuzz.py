from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
import argparse
from MS_fuzz.fuzz_config.Config import Config
from MS_fuzz.common.simulator_v2 import Simulator
def set_args():
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("--debug", action="store_true", default=False)
    argument_parser.add_argument("-o", "--out-dir", default="output", type=str,
                                 help="Directory to save fuzzing logs")
    argument_parser.add_argument("-s", "--seed-dir", default="seed-artifact", type=str,
                                 help="Seed directory")
    argument_parser.add_argument("-m", "--max-mutations", default=5, type=int,
                                 help="Size of the mutated population per cycle")
    argument_parser.add_argument("-d", "--determ-seed", type=float,
                                 help="Set seed num for deterministic mutation (e.g., for replaying)")
    argument_parser.add_argument("-u", "--sim-host", default="localhost", type=str,
                                 help="Hostname of Carla simulation server")
    argument_parser.add_argument("-p", "--sim-port", default=2000, type=int,
                                 help="RPC port of Carla simulation server")
    argument_parser.add_argument("-t", "--target", default="behavior", type=str,
                                 help="Target autonomous driving system (behavior/Autoware)")
    argument_parser.add_argument("-f", "--function", default="general", type=str,
                                 choices=["general", "collision", "traction", "eval-os", "eval-us",
                                          "figure", "sens1", "sens2", "lat", "rear"],
                                 help="Functionality to test (general / collision / traction)")
    argument_parser.add_argument("-k", "--num_mutation_car", default=3, type=int,
                                 help="Number of max weight vehicles to mutation per cycle, default=1,negative means "
                                      "random")
    argument_parser.add_argument("--density", default=1, type=float,
                                 help="density of vehicles,1.0 means add 1 bg vehicle per 1 sec")
    argument_parser.add_argument("--town", default=3, type=int,
                                 help="Test on a specific town (e.g., '--town 3' forces Town03)")
    argument_parser.add_argument("--timeout", default="60", type=int,
                                 help="Seconds to timeout if vehicle is not moving")
    argument_parser.add_argument("--no-speed-check", action="store_true")
    argument_parser.add_argument("--no-lane-check", action="store_true")
    argument_parser.add_argument("--no-crash-check", action="store_true")
    argument_parser.add_argument("--no-stuck-check", action="store_true")
    argument_parser.add_argument("--no-red-check", action="store_true")
    argument_parser.add_argument("--no-other-check", action="store_true")
    argument_parser.add_argument("--no-traffic-lights", action="store_true")
    return argument_parser


def main():
    conf = Config()
    argument_parser = set_args()
    args = argument_parser.parse_args()
    conf.sim_port = args.port
    sim = Simulator(conf)
    pass

if __name__ == "__main__":
    main()
