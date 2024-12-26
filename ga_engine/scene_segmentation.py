import carla
import math
from typing import List
import time
import threading
import signal
import pdb
import xml.etree.ElementTree as ET


from MS_fuzz.ms_utils.apollo_routing_listener import ApolloRoutingListener
from MS_fuzz.ms_utils import rotate_point


class Segment(object):
    def __init__(self, location: carla.Location,
                 rotation: carla.Rotation,
                 length: float, width: float,
                 is_junction=False):
        self.location: carla.Location = location
        self.rotation: carla.Rotation = rotation
        self.length: float = length
        self.width: float = width

        self.is_junction = is_junction

        self.belongs_to_roadid = ''
        self.type = ('straight', -1)

        self.bbox: carla.BoundingBox = carla.BoundingBox(
            location, carla.Vector3D(self.length/2, self.width/2, 2))


class SceneSegment(object):
    def __init__(self, world: carla.World,
                 vehicle: carla.Vehicle,
                 logger=None,
                 debug=False):
        self.carla_world: carla.World = world
        self.ego_vehicle: carla.Vehicle = vehicle
        self.carla_map: carla.Map = self.carla_world.get_map()
        self.xodr_str = self.carla_map.to_opendrive()
        self.xodr_root = ET.fromstring(self.xodr_str)
        self.logger = logger
        self.debug = debug

        self.map_roads = {}
        self.map_junctions = {}
        self.map_road_to_junctions = {}

        self.finished_index = -1
        self.curr_seg_index = -1
        self.belongs_to_two_index = (True, False)
        # | normal: [ *] [ ];     | between: [ [*] ];       | neither: [ ]*[ ]         |
        # |         ↑             |          ↑              |          ↑               |
        # |         curr_index    |          curr_index     |          curr_index      |
        # |    (True, False)      |       (True, True)      |     (False, False)       |

        self.phrase_xodr()
        # print(self.map_roads)
        # print(self.map_junctions)

        self.routing_listener = ApolloRoutingListener(self.carla_world,
                                                      ego_vehicle=self.ego_vehicle,
                                                      logger=self.logger,
                                                      debug=self.debug)
        # self.routing_listener = ApolloRoutingListener(self.carla_world,
        #                                               ego_vehicle=self.ego_vehicle,
        #                                               logger=self.logger,
        #                                               debug=False)

        self.segments: List[Segment] = []

        self.vehicle_pos_listener_thread: threading.Thread = None

        self.stop_vehicle_pos_listening = False

    def phrase_xodr(self):
        for road in self.xodr_root.findall('road'):
            if road.get('junction') != '-1':
                self.map_road_to_junctions[road.get(
                    'id')] = road.get('junction')
            # pdb.set_trace()
            left_lane = [lan.get('id') for lan in (road.find('.//laneSection/left') or []).findall(
                'lane') if lan.get('type') == 'driving'] if road.find('.//laneSection/left') is not None else []
            right_lane = [lan.get('id') for lan in (road.find('.//laneSection/right') or []).findall(
                'lane') if lan.get('type') == 'driving'] if road.find('.//laneSection/right') is not None else []

            lane_count = len(left_lane) + len(right_lane)
            self.map_roads[road.get('id')] = {'lane_num': lane_count, 'left_lane': left_lane,
                                              'right_lane': right_lane}
        for junction in self.xodr_root.findall('junction'):
            connections = junction.findall('.//connection')
            incoming_roads = set()
            for connection in connections:
                incoming_roads.add(connection.get('incomingRoad'))
            self.map_junctions[junction.get('id')] = {'dir_count': len(
                incoming_roads), 'incoming_roads': list(incoming_roads)}

    def strat_vehicle_pos_listening(self):
        self.vehicle_pos_listener_thread = threading.Thread(
            target=self.listening_thread)
        self.stop_vehicle_pos_listening = False
        self.vehicle_pos_listener_thread.start()

    def stop_vehicle_listening(self):
        self.stop_vehicle_pos_listening = True
        if self.vehicle_pos_listener_thread:
            self.vehicle_pos_listener_thread.join()

    def listening_thread(self):
        last_in_index = self.curr_seg_index - 1
        while not self.stop_vehicle_pos_listening:
            pos = self.ego_vehicle.get_location()

            # have not yet gain segments from routing listener
            if len(self.segments) == 0:
                self.finished_index = -1
                continue

            # have finished all segments
            if self.finished_index == len(self.segments) - 1:
                self.curr_seg_index = len(self.segments) - 1
                break

            # have not yet reach the first segment
            if self.curr_seg_index < 0:
                self.finished_index = -1
                # wait until reach the first segment
                while not self.check_if_loc_in_segment(pos, self.segments[0]):
                    pos = self.ego_vehicle.get_location()
                    if self.stop_vehicle_pos_listening:
                        break
                # now reached the first segment
                self.curr_seg_index = 0
                last_in_index = self.curr_seg_index
                continue

            is_in_list = []
            already_in_list = []
            for index, seg in enumerate(self.segments):
                is_in = self.check_if_loc_in_segment(pos, seg)
                is_in_list.append(is_in)
                if is_in:
                    already_in_list.append(index)

            if already_in_list != []:
                self.curr_seg_index = already_in_list[0]
                if self.curr_seg_index != last_in_index:
                    self.finished_index = last_in_index
                    last_in_index = self.curr_seg_index
                if len(already_in_list) > 1:
                    self.belongs_to_two_index = (True, True)
                else:
                    self.belongs_to_two_index = (True, False)
            else:
                self.belongs_to_two_index = (False, False)
                if last_in_index == self.curr_seg_index:
                    self.finished_index = self.curr_seg_index

            # in_curr = self.check_if_loc_in_segment(
            #     pos, self.segments[self.curr_seg_index])
            # in_next = False
            # if self.finished_index < len(self.segments) - 2:
            #     in_next = self.check_if_loc_in_segment(
            #         pos, self.segments[self.curr_seg_index + 1])
            # self.belongs_to_two_index = (in_curr, in_next)
            # if in_curr == last_in_curr:
            #     if in_next and not in_curr:
            #         # [ ] *[ ] ->[ ] [* ]
            #         self.finished_index = self.curr_seg_index
            #         self.curr_seg_index += 1
            #         last_in_curr = in_next
            #     continue
            # else:
            #     if in_curr and in_next:
            #         # [ [*] ] -> [ []* ]
            #         self.finished_index = self.curr_seg_index
            #         self.curr_seg_index += 1
            #         last_in_curr = in_next
            #     if not in_curr and not in_next:
            #         # [ *] [ ] -> [ ]* [ ]
            #         self.finished_index = self.curr_seg_index
            #     last_in_curr = in_curr
            #     continue

    def check_if_loc_in_segment(self, pos: carla.Location, seg: Segment):
        dx = pos.x - seg.location.x
        dy = pos.y - seg.location.y
        local_x, local_y = rotate_point(dx, dy, -seg.rotation.yaw)
        half_width = seg.width / 2
        half_length = seg.length / 2
        return (-half_length <= local_x <= half_length) and (-half_width <= local_y <= half_width)

    def interpolate_location(start, end, fraction):
        x = start.x + (end.x - start.x) * fraction
        y = start.y + (end.y - start.y) * fraction
        z = start.z + (end.z - start.z) * fraction
        return carla.Location(x, y, z)

    def get_seg_type(self, seg: Segment, commmon_size=(30, 30)):
        road_id = seg.belongs_to_roadid.split('_')[1]
        if seg.is_junction:
            size = 'large'
            if seg.width <= commmon_size[0] and seg.length <= commmon_size[1]:
                size = 'small'
            elif seg.width <= 2*commmon_size[0] and seg.length <= 2*commmon_size[1]:
                size = 'medium'

            if road_id not in self.map_road_to_junctions:
                return f'junction_{size}_-1_dir'

            jun_index = self.map_road_to_junctions[road_id]
            if jun_index not in self.map_junctions:
                return f'junction_{size}_-1_dir'

            # for example 'junction_medium_3_dir'
            return f"junction_{size}_{self.map_junctions[jun_index]['dir_count']}_dir"

        else:
            if road_id in self.map_roads:
                lan_dir = '2_way'
                if len(self.map_roads[road_id]['left_lane']) == 0 or \
                        len(self.map_roads[road_id]['right_lane']) == 0:
                    lan_dir = '1_way'

                # for example 'straight_2_way_8_lane'
                return (f"straight_{lan_dir}_{self.map_roads[road_id]['lane_num']}_lane")
            else:
                return ('straight_-1_way_-1_lane')

    def wait_for_route(self, req_time, interval=-2, wait_from_req_time=False,
                       timeout=10):
        if wait_from_req_time:
            while self.routing_listener.req_time == None or \
                    (self.routing_listener.req_time - req_time) <= interval:
                if not self.routing_listener.running or timeout <= 0:
                    return False
                time.sleep(0.1)
                timeout -= 0.1

        else:
            while self.routing_listener.recv_time == None or \
                    (self.routing_listener.recv_time - req_time) <= interval:
                if not self.routing_listener.running or timeout <= 0:
                    return False
                time.sleep(0.1)
                timeout -= 0.1
                
        return True

    def get_seg_from_junction_wp(self, wp: carla.Waypoint, length: float, width: float):
        junction = wp.get_junction()
        junction_center_loc = junction.bounding_box.location
        junction_length = junction.bounding_box.extent.x * 2
        junction_width = junction.bounding_box.extent.y * 2
        seg_length = length if length > abs(
            junction_length) else junction_length
        seg_width = width if width > abs(junction_width) else junction_width
        seg = Segment(junction_center_loc,
                      wp.transform.rotation, seg_length, seg_width)
        seg.is_junction = True
        return [seg]

    def get_seg_from_straight_to_end_wp(self, wp: carla.Waypoint,
                                        length: float, width: float):
        segs = []
        if wp == None:
            return segs
        among_wps = wp.previous_until_lane_start(length)
        for among_wp in among_wps:
            segs.append(Segment(among_wp.transform.location,
                                among_wp.transform.rotation, length, width))
        segs.reverse()
        return segs

    def get_seg_from_straight_from_start_wp(self, wp: carla.Waypoint,
                                           length: float, width: float):
        segs = []
        among_wps = wp.next_until_lane_end(length)
        for among_wp in among_wps:
            segs.append(Segment(among_wp.transform.location,
                                among_wp.transform.rotation, length, width))
            
    def get_seg_between_two_wp(self, wp_s: carla.Waypoint,
                               wp_e: carla.Waypoint,
                               length: float, width: float):
        segs = []
        if wp_s == None:
            segs = self.get_seg_from_straight_to_end_wp(wp_e, length, width)
            return segs
        if wp_e == None:
            return segs
        wp = wp_s
        max = int(wp_s.transform.location.distance(
            wp_e.transform.location)/length) + 3
        # Ensures precise division even on winding roads
        while wp.transform.location.distance(wp_e.transform.location) > length:
            segs.append(Segment(wp.transform.location,
                                wp.transform.rotation, length, width))
            wp = wp.next(length)[0]
            # Prevent falling into an infinite loop
            max -= 1
            if max <= 0:
                break
        return segs

    def get_seg_from_full_road(self, wp_touple,
                               length: float, width: float):
        segs = []
        among_wps = []
        start_wp = wp_touple[0]
        if self.segments != [] and self.segments[-1] != None:
            last_seg = self.segments[-1]
            last_trans = carla.Transform(
                carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
            # print(last_seg.bbox.contains(start_wp.transform.location, last_trans))
            if last_seg.bbox.contains(start_wp.transform.location, last_trans):
                start_wp = start_wp.next(length/2)[0]  # forward half a length
            pass
        try:
            next_wps = start_wp.next_until_lane_end(length)
            among_wps = [start_wp] + next_wps[:-1]
        except RuntimeError as e:
            try:
                prev_wps = wp_touple[1].previous_until_lane_start(length)
                prev_wps.reverse()
                among_wps = prev_wps
            except RuntimeError as e:
                if self.debug and self.logger != None:
                    self.logger.warning(f'{wp_touple}: fail {e}')
                among_wps.append(start_wp)
        for among_wp in among_wps:
            segs.append(Segment(among_wp.transform.location,
                                among_wp.transform.rotation, length, width))
        return segs

    def get_segments(self, length: float, width: float):
        self.segments = []
        routting_road = []
        with self.routing_listener.lock:
            routting_road = self.routing_listener.routing
            routing_wps = self.routing_listener.routing_wps
        # pdb.set_trace()
        if routing_wps[0][0] == None:
            # we assume that the vehicle is stopped at the beginning of its planning road
            if self.ego_vehicle != None:
                start_loc = self.ego_vehicle.get_transform().location
                routing_wps[0][0] = self.carla_world.get_map(
                ).get_waypoint(start_loc)

        for i, route_wp in enumerate(routing_wps):
            if route_wp[0] is None:
                continue
            if route_wp[1] is not None:
                if route_wp[0].is_junction and route_wp[1].is_junction:
                    if i == 0:
                        continue
                    junction_segs = self.get_seg_from_junction_wp(route_wp[0],
                                                                length, width)
                    for seg in junction_segs:
                        seg.belongs_to_roadid = routting_road[i]
                    self.segments += junction_segs
                    continue
                elif route_wp[0].is_junction and not route_wp[1].is_junction:
                    first_segs = self.get_seg_from_straight_to_end_wp(route_wp[1],
                                                                    length, width)
                    for seg in first_segs:
                        seg.belongs_to_roadid = routting_road[i]
                    self.segments += first_segs
                    continue
            else:
                if route_wp[0].is_junction:
                    if i == 0:
                        continue
                    junction_segs = self.get_seg_from_junction_wp(route_wp[0],
                                                                length, width)
                    for seg in junction_segs:
                        seg.belongs_to_roadid = routting_road[i]
                    self.segments += junction_segs
                    continue

            if i == len(routing_wps) - 1:
                # handle the final segment
                final_segs = self.get_seg_between_two_wp(route_wp[0],
                                                         route_wp[1],
                                                         length, width)
                for seg in final_segs:
                    seg.belongs_to_roadid = routting_road[i]
                self.segments += final_segs
                continue
                
            
            # elif not route_wp[0].is_junction and route_wp[1].is_junction:
            #     last_segs = self.get_seg_from_straight_to_end_wp(route_wp[0],
            #                                                      length, width)
            #     for seg in last_segs:
            #         seg.belongs_to_roadid = routting_road[i]
            #     self.segments +=last_segs

            between_segs = self.get_seg_from_full_road(route_wp, length, width)
            for seg in between_segs:
                seg.belongs_to_roadid = routting_road[i]

            self.segments += between_segs

        # delete those segs that are too close to each other
        segs_temp = self.segments
        for index in range(1, len(self.segments)):
            if self.segments[index-1] == None:
                continue
            if self.segments[index].location.distance(
                    self.segments[index - 1].location) < length*2/3:
                segs_temp[index] = None
        self.segments = [seg for seg in segs_temp if seg != None]

        if self.debug:
            carla_db = self.carla_world.debug
            for i, seg in enumerate(self.segments):
                if seg.bbox == None:
                    continue
                carla_db.draw_string(seg.location,
                                     f"segmentation #{i} junction: {seg.is_junction}",
                                     color=carla.Color(0, 191, 255),
                                     life_time=60)
                carla_db.draw_box(seg.bbox, seg.rotation, thickness=0.1,
                                  color=carla.Color(0, 191, 255), life_time=60)


if __name__ == '__main__':
    from loguru import logger
    client = carla.Client('172.17.0.1', 5000)
    world = client.get_world()

    SS = SceneSegment(world, vehicle=None, debug=True, logger=logger)

    def signal_handler(sig, frame):
        SS.routing_listener.stop()
        exit()

    signal.signal(signal.SIGINT, signal_handler)

    SS.routing_listener.start()

    SS.wait_for_route(time.time())
    SS.get_segments(30, 30)
    SS.routing_listener.stop()
