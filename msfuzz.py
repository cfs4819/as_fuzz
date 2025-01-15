import os
import multiprocessing
import subprocess
import threading
import argparse
import signal
import time
import pdb
import sys

import carla

from carla_bridge.apollo_carla_bridge import CarlaCyberBridge
from loguru import logger

from MS_fuzz.fuzz_config.Config import Config
from MS_fuzz.common.simulator import Simulator
from MS_fuzz.ga_engine.ga_lib import GA_LIB


def set_args():
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("-o", "--out-dir", default="/apollo/data/AS_fuzz/result", type=str,
                                 help="Directory to save fuzzing logs")
    argument_parser.add_argument("--ga-dir", default="/apollo/data/AS_fuzz/ga_lib", type=str,
                                 help="Directory to save GALib progress files")
    # argument_parser.add_argument("-m", "--max-mutations", default=5, type=int,
    #                              help="Size of the mutated population per cycle")
    argument_parser.add_argument("-u", "--sim-host", default="172.17.0.1", type=str,
                                 help="Hostname of Carla simulation server")
    argument_parser.add_argument("-p", "--sim-port", default=4000, type=int,
                                 help="RPC port of Carla simulation server")
    argument_parser.add_argument("--town", default=10, type=int,
                                 help="Test on a specific town (e.g., '--town 3' forces Town03)")
    argument_parser.add_argument("-d","--traffic-density", default=1, type=int,
                                 help="Traffic density of the simulation. Avaliable only when results/ga_lib is empty")
    argument_parser.add_argument("-ntrb", "--not-trying-release-block", action="store_true",
                                 help="Disable releasing road blockage, only log the blockage time")
    return argument_parser


class MS_FUZZ(object):
    def __init__(self, args):
        self.conf = Config()

        town_index = args.town
        self.conf.carla_map = self.conf.town_name[str(town_index)]
        self.conf.dreamview_map = self.conf.dreamview_map_dic[self.conf.carla_map]
        self.conf.sim_port = args.sim_port
        self.conf.sim_host = args.sim_host
        self.conf.out_dir = args.out_dir
        self.conf.try_relese_block = not args.not_trying_release_block
        self.conf.density = args.traffic_density
        
        self.eva_req_queue = multiprocessing.Queue()
        self.eva_res_queue = multiprocessing.Queue()

        self.sim_stop_queue = multiprocessing.Queue()

        self.ga_path = args.ga_dir
        if not os.path.exists(self.ga_path):
            os.makedirs(self.ga_path)

        self.ga_lib: GA_LIB = None

        # self.ga_process = multiprocessing.Process(target=self.ga_lib.run)
        # self.ga_process.start()
        self.ga_process = multiprocessing.Process(
            target=self.ga_lib_progress_handler)
        self.ga_process.start()

        self.sim: Simulator = None
        self.sim_process = None

        signal.signal(signal.SIGINT, self.close)
        # signal.signal(signal.SIGTERM, self.close)

        # pdb.set_trace()
        if not self.check_carla():
            logger.error("Carla Dieded. Please Reload Carla")
            self.close()
        time.sleep(1)

        while True:
            # check twice
            if not self.check_carla():
                logger.error("Carla Dieded. Please Reload Carla")
                self.close()
                break

            self.sim_process = multiprocessing.Process(
                target=self.sim_progress_handler, args=(self.sim_stop_queue,))
            self.sim_process.start()

            while True:
                if not self.sim_process.is_alive():
                    logger.warning(
                        "Simulator process has exited. Restarting...")
                    self.sim_process.terminate()
                    self.sim_process.join()
                    self.sim = None
                    break
                if not self.ga_process.is_alive():
                    self.close()
                    sys.exit()

                time.sleep(1)

            time.sleep(1)

    def check_carla(self):
        client = carla.Client(self.conf.sim_host, self.conf.sim_port)
        print(self.conf.sim_port)
        client.set_timeout(5)
        world = None
        try:
            world = client.get_world()
        except RuntimeError:
            return False
        if world:
            return True

    def ga_lib_progress_handler(self):
        self.ga_lib = GA_LIB(self.conf.scenario_length,
                             self.conf.scenario_width,
                             self.eva_req_queue,
                             self.eva_res_queue,
                             logger,
                             self.ga_path)

        def sigint_handler(signum, frame):
            logger.warning("GA_LIB Process SIGINT received. Saving...")
            self.ga_lib.close_and_save()
            os.kill(os.getpid(), signal.SIGKILL)

        signal.signal(signal.SIGINT, sigint_handler)

        self.ga_lib.continue_ga()
        self.ga_lib.run()
        logger.warning("GA_LIB has exited. Saving...")
        self.ga_lib.close_and_save()

    def sim_progress_handler(self, stop_queue: multiprocessing.Queue):
        sim = Simulator(self.conf,
                        self.eva_req_queue,
                        self.eva_res_queue)

        def terminate_listener_handler():
            while True:
                msg = stop_queue.get()
                if msg == 'stop':
                    logger.warning(
                        'Simulator Process stop signal received. Closing...')
                    sim.close()
                    return

        terminate_listener_t = threading.Thread(
            target=terminate_listener_handler)
        terminate_listener_t.start()

        def close_handler(signal_received, frame):
            logger.warning("Simulator Process SIGINT received. Closing...")
            sim.close()

        signal.signal(signal.SIGINT, close_handler)

        sim.initialization()
        sim.main_loop()
        sim.close()

    def close(self, signal_received=None, frame=None):
        if self.ga_process is not None and self.ga_process.is_alive():
            logger.warning("Closing GA_LIB")
            self.eva_req_queue.put({
                'cmd': 'close'
            })
            self.ga_process.join()

        if self.sim_process is not None and self.sim_process.is_alive():
            logger.warning("Closing Simulator")
            self.sim_stop_queue.put('stop')
            self.sim_process.join()

        # Terminate the processes
        processes = [self.ga_process, self.sim_process]
        # for process in processes:
        #     if process is not None and process.is_alive():
        #         process.terminate()

        # Wait for processes to exit
        timeout = 20
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(process is None or not process.is_alive() for process in processes):
                break
            time.sleep(1)

        # If processes did not exit in time, forcefully kill the remaining processes
        for process in processes:
            if process:
                if process.is_alive():
                    logger.error(
                        f"Process {process.pid} did not terminate in time. Forcefully killing it.")
                    os.kill(process.pid, signal.SIGKILL)

        logger.info("All processes terminated gracefully.")
        os._exit(0)


def main():
    # conf = Config()
    argument_parser = set_args()
    args = argument_parser.parse_args()

    msfuzz = MS_FUZZ(args)


if __name__ == "__main__":
    main()
