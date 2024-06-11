import carla
import time
import os
import json

from datetime import timedelta

from MS_fuzz.common.unsafe_detector import *
from MS_fuzz.ms_utils import predict_collision


class ResultSaver(object):
    def __init__(self, save_floder_path=''):

        self.result_to_save = {}
        
        self.frames_record = []
        '''
            frame_record = {
                'timestamp': time.time(),
                'frame': frame:int,
                'min_dis': min_dis:float,
                'unsmooth_acc': unsmooth_acc,
                'walker_in_road': walker_in_road:int,
                'npc_vehicles_ss': npc_vehicles_ss:List[carla.ActorSnapshot],
                'ego_ss': ego_ss:carla.ActorSnapshot
            }
        '''
        
        self.sce_result_path = save_floder_path
        
        self.clear_result()

    def set_save_path(self, save_floder_path):
        self.sce_result_path = save_floder_path
        
    def clear_result(self):
        self.result_to_save = {}
        self.result_to_save = {
            'unsafe': False,
            'unsafe_type': None,
            'minor_unsafe': [],
            'start_loc': None,
            'dest_loc': None,
            'end_loc': None,
            'start_time': time.time(),
            'end_time': None,
            'run_time': None,
            'interaction': None,
            'video_path': None,
            'Odometer': None,
        }
        
        self.frames_record = []
    
    def add_frame(self, frame: dict):
        self.frames_record.append(frame)
        
    def add_minor_unsafe(self,
                          unsafe_type: UNSAFE_TYPE,
                          data,
                          trigger_time):
        time_pass = trigger_time - self.result_to_save['start_time']
        time_pass_str = str(timedelta(seconds=time_pass))

        if unsafe_type == UNSAFE_TYPE.ACCELERATION:
            if self.result_to_save['minor_unsafe'] and \
                    'unsafe_acc' in self.result_to_save['minor_unsafe']:
                if data == self.result_to_save['minor_unsafe']['unsafe_acc'][-1]['acc']:
                    return
                self.result_to_save['minor_unsafe']['unsafe_acc'].append({
                    'time': time_pass,
                    'time_str': time_pass_str,
                    'acc': data
                })
            elif self.result_to_save['minor_unsafe'] and \
                    'unsafe_acc' not in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['unsafe_acc'] = [{
                    'time': time_pass,
                    'time_str': time_pass_str,
                    'acc': data
                }]
            else:
                self.result_to_save['minor_unsafe'] = {
                    'unsafe_acc': [{
                        'time': time_pass,
                        'time_str': time_pass_str,
                        'acc': data
                    }]
                }

        elif unsafe_type == UNSAFE_TYPE.LANE_CHANGE:
            if self.result_to_save['minor_unsafe'] and \
                    'lanechange_timeout' in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['lanechange_timeout'].append({
                    'time': time_pass,
                    'time_str': time_pass_str
                })
            elif self.result_to_save['minor_unsafe'] and \
                    'lanechange_timeout' not in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['lanechange_timeout'] = [{
                    'time': time_pass,
                    'time_str': time_pass_str
                }]
            else:
                self.result_to_save['minor_unsafe'] = {
                    'lanechange_timeout': [{
                        'time': time_pass,
                        'time_str': time_pass_str
                    }]
                }

        elif unsafe_type == UNSAFE_TYPE.CROSSING_SOLID_LANE:
            lane_type_str = 'Other'
            if data == 1:
                lane_type_str = 'Solid Lane'
            if data == 2:
                lane_type_str = 'Solid Solid Lane'
            if self.result_to_save['minor_unsafe'] and \
                    'crossing_solid_lane' in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['crossing_solid_lane'].append({
                    'time': time_pass,
                    'time_str': time_pass_str,
                    'crossing': lane_type_str
                })
            elif self.result_to_save['minor_unsafe'] and \
                    'crossing_solid_lane' not in self.result_to_save['minor_unsafe']:
                self.result_to_save['minor_unsafe']['crossing_solid_lane'] = [{
                    'time': time_pass,
                    'time_str': time_pass_str,
                    'crossing': lane_type_str
                }]
            else:
                self.result_to_save['minor_unsafe'] = {
                    'crossing_solid_lane': [{
                        'time': time_pass,
                        'time_str': time_pass_str,
                        'crossing': lane_type_str
                    }]
                }

    def save_result(self, ego_curr_loc:carla.Location, save_video=True):
        self.result_to_save['end_loc'] = {
            'x': ego_curr_loc.x,
            'y': ego_curr_loc.y,
            'z': ego_curr_loc.z
        }

        now = time.time()
        self.result_to_save['end_time'] = now
        self.result_to_save['run_time'] = now - \
            self.result_to_save['start_time']

        if not self.frames_record or len(self.frames_record) == 0:
            self.result_to_save['interaction'] = None
            self.result_to_save['Odometer'] = None
        else:
            ego_pos_l = []
            run_distance = 0.0
            interaction_per_frame = []
            total_frame_num = len(self.frames_record)

            will_collide_frame_cnt = 0

            pre_frame = None

            # final_frame_time = self.frames_record[-1]['timestamp']
            for frame in self.frames_record:
                ego_ss = frame['ego_ss']
                npcs_ss = frame['npc_vehicles_ss']
                ego_pos_l.append({
                    'timestamp': frame['timestamp'],
                    'frame_num': frame['frame'],
                    'x': ego_ss.get_transform().location.x,
                    'y': ego_ss.get_transform().location.y,
                    'z': ego_ss.get_transform().location.z,
                    'yaw': ego_ss.get_transform().rotation.yaw,
                    'pitch': ego_ss.get_transform().rotation.pitch,
                    'roll': ego_ss.get_transform().rotation.roll,
                    'speed': ego_ss.get_velocity().length(),
                    'run_distance': round(run_distance, 2)
                })

                if pre_frame:
                    pre_ego_ss = pre_frame['ego_ss']
                    delta_dis = abs(ego_ss.get_transform().location.distance(
                        pre_ego_ss.get_transform().location))
                    run_distance += delta_dis

                pre_colli_cunt = 0
                this_frame_has_colli = False
                npcs_pos = []
                for npc_ss in npcs_ss:
                    if npc_ss == None:
                        continue
                    will_collide, t = predict_collision(ego_ss, npc_ss)
                    pre_colli_cunt += 1 if will_collide else 0
                    if will_collide:
                        this_frame_has_colli = True
                    npc_loc = npc_ss.get_transform().location
                    npcs_pos.append({
                        'x': npc_loc.x,
                        'y': npc_loc.y,
                        'z': npc_loc.z,
                        'will_collide': will_collide
                    })

                will_collide_frame_cnt += 1 if this_frame_has_colli else 0
                will_collide_rate = pre_colli_cunt / \
                    len(npcs_ss) if npcs_ss else 0

                interaction_per_frame.append({
                    'timestamp': frame['timestamp'],
                    'frame_num': frame['frame'],
                    'interaction_rate': will_collide_rate,
                    'npcs_pos':npcs_pos
                })

                pre_frame = frame

            self.result_to_save['Odometer'] = {
                'total_distance': round(run_distance, 2),
                'pos_per_frame': ego_pos_l
            }
            self.result_to_save['interaction'] = {
                'interact_frame_rate': will_collide_frame_cnt / total_frame_num,
                'per_frame': interaction_per_frame
            }
        try:
            resule_str = json.dumps(self.result_to_save, indent=4)
            result_path = os.path.join(self.sce_result_path, 'result.json')
            with open(result_path, 'w') as f:
                f.write(resule_str)
            if not save_video:
                if os.path.isfile(self.result_to_save['video_path']):
                    os.remove(self.result_to_save['video_path'])
                    self.result_to_save['video_path'] = 'deleted'
        except Exception as e:
            print(e)
