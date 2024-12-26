import time
import threading
import os
import json
import queue
from typing import Dict, List


from multiprocessing import Process, Queue
from MS_fuzz.common.evaluate import Evaluate_Object, Evaluate_Transfer
from MS_fuzz.ga_engine.cega import CEGA


class GA_LIB():
    def __init__(self,
                 scenario_length,
                 scenario_width,
                 eva_req_queue: Queue,
                 eva_res_queue: Queue,
                 logger,
                 ga_lib_floder_path=None):

        self.ga_lib: Dict[str, CEGA] = {}

        self.scenario_length = scenario_length
        self.scenario_width = scenario_width
        self.ga_lib_floder_path = ga_lib_floder_path
        self.logger = logger

        self.close_event = threading.Event()

        self.req_queue = eva_req_queue
        self.res_queue = eva_res_queue

        self.saved = False
        self.closing = False
        self.lock = threading.Lock()

        self.eva_res_list: Dict[str, Evaluate_Object] = {}

    def load_from_path(self):
        if self.ga_lib_floder_path is None:
            self.logger.error('No folder path provided for GA_LIB.')
            return

        lib_path = os.path.join(self.ga_lib_floder_path, 'cega_lib.json')

        try:
            with open(lib_path, 'r') as f:
                save_dict = json.load(f)

            for type_str, cega_path in save_dict.items():
                cega = CEGA(self.scenario_length,
                            self.scenario_width,
                            logger=self.logger)
                cega.load_from_file(cega_path)
                cega_his_dir = os.path.join(self.ga_lib_floder_path,
                                            f'gen_his_{cega.type_str}')
                cega.set_generation_file(cega_his_dir)
                cega.start()
                self.ga_lib[type_str] = cega

            self.logger.info(
                'Loaded and started all CEGA processes successfully.')

        except Exception as e:
            self.logger.error(f'Failed to load from path: {e}')

    def get_an_unevaluated_obj(self, type_str: str) -> Evaluate_Object:
        with self.lock:
            cega = self.ga_lib.get(type_str)
            if cega is None:
                cega = CEGA(self.scenario_length,
                            self.scenario_width, logger=self.logger)
                cega.type_str = type_str
                cega.prase_road_type(type_str)
                cega_his_dir = os.path.join(
                    self.ga_lib_floder_path, f'gen_his_{cega.type_str}')
                cega.set_generation_file(cega_his_dir)
                cega.start()
                self.ga_lib[type_str] = cega
                timeout = 2
                if len(cega.evaluate_list) == 0:
                    self.logger.info('Waiting for first individual')
                    while len(cega.evaluate_list) == 0:
                        time.sleep(0.01)
                        timeout -= 0.01
                        if self.close_event.is_set():
                            return None
                        if timeout <= 0:
                            self.logger.error('Waiting Timeout')
                            self.close()
                            return None
                self.logger.info(f'CEGA {type_str} individual gained')
            obj_2_evaluate = cega.get_an_unevaluated_obj()

        return obj_2_evaluate

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.logger.info('Closing CEGA Lib')
        self.close_event.set()
        for cega in self.ga_lib.values():
            cega.stop()
        self.closing = False

    def close_and_save(self):
        if self.closing:
            return
        self.closing = True

        self.logger.info('Closing and saving CEGA Lib')
        print(self.ga_lib)
        self.close_event.set()
        for cega in self.ga_lib.values():
            cega.stop()
        print(f'{time.time()}, saved:{self.saved}')
        if not self.saved:
            save_dict = {}
            with self.lock:
                for type, cega in self.ga_lib.items():
                    cega_path = os.path.join(
                        self.ga_lib_floder_path, 'cega_' + type + '.pkl')
                    cega.save(cega_path)
                    save_dict[type] = cega_path

            if save_dict:
                lib_path = os.path.join(
                    self.ga_lib_floder_path, 'cega_lib.json')
                with open(lib_path, 'w') as f:
                    json.dump(save_dict, f)
                self.saved = True
            else:
                self.logger.error('No CEGA objects to save.')

        self.closing = False

    def check_required_files(self):
        if self.ga_lib_floder_path is None:
            self.logger.error('No folder path provided for GA_LIB.')
            return False

        required_files = 'cega_lib.json'
        file_path = os.path.join(self.ga_lib_floder_path, required_files)
        if not os.path.isfile(file_path):
            self.logger.error(f'Missing required file: {required_files}')
            return False
         # Check the paths inside cega_lib.json
        lib_path = os.path.join(self.ga_lib_floder_path, 'cega_lib.json')
        try:
            with open(lib_path, 'r') as f:
                save_dict = json.load(f)

            missing_paths = []
            for type_str, cega_path in save_dict.items():
                if not os.path.isfile(cega_path):
                    missing_paths.append(cega_path)

            if missing_paths:
                for path in missing_paths:
                    self.logger.error(f'Missing required file: {path}')
                return False

        except Exception as e:
            self.logger.error(f'Error reading or parsing cega_lib.json: {e}')
            return False

        self.logger.info('All required cega paths are present.')
        return True

    def continue_ga(self):
        if self.check_required_files():
            self.load_from_path()

    def run(self):
        # Main loop to handle requests
        while not self.close_event.is_set():
            try:
                # always wait for a request
                req_dic = self.req_queue.get()
                cmd = req_dic.get('cmd')
                if cmd == "close":
                    self.logger.warning('Received close request')
                    self.close_event.set()
                    break
                elif cmd == 'get_obj':
                    type_str = req_dic.get('type_str')
                    obj_2_evaluate = self.get_an_unevaluated_obj(type_str)
                    res_id = f'{type_str}_{obj_2_evaluate.id}'
                    if obj_2_evaluate is None:
                        res_dict = {
                            'type_str': type_str,
                            'obj': None
                        }
                        self.res_queue.put(res_dict)
                        continue
                    eva_t = Evaluate_Transfer(res_id,
                                              obj_2_evaluate.id,
                                              obj_2_evaluate.walker_ind,
                                              obj_2_evaluate.vehicle_ind,
                                              obj_2_evaluate.is_evaluated,
                                              obj_2_evaluate.is_in_queue)
                    res_dict = {
                        'type_str': type_str,
                        'obj': eva_t
                    }
                    self.res_queue.put(res_dict)
                    self.eva_res_list[res_id] = obj_2_evaluate
                elif cmd == 'feedback':
                    print('get a feedback')
                    eva_obj:Evaluate_Transfer = req_dic.get('eva_obj')
                    res_id = eva_obj.uid
                    target_eva_obj = self.eva_res_list.get(res_id)
                    target_eva_obj.walker_ind.fitness.values = eva_obj.walker_ind.fitness.values
                    target_eva_obj.vehicle_ind.fitness.values = eva_obj.vehicle_ind.fitness.values
                    target_eva_obj.is_evaluated = eva_obj.is_evaluated
                    target_eva_obj.is_in_queue = False
            except queue.Empty:
                continue
            except KeyboardInterrupt:
                break
            except Exception as e:
                print('[Ga_lib]',e)
                continue
