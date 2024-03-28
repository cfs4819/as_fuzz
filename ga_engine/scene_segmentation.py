import carla
import math
from typing import List
import time
import signal
import pdb

from MS_fuzz.ms_utils.apollo_routing_listener import ApolloRoutingListener


class Segment(object):
    def __init__(self, location: carla.Location,
                 rotation: carla.Rotation,
                 length: float, width: float):
        self.location = location
        self.rotation = rotation
        self.length = length
        self.width = width

        self.is_junction = False

        self.bbox: carla.BoundingBox = carla.BoundingBox(
            location, carla.Vector3D(self.length/2, self.width/2, 2))


class SceneSegment(object):
    def __init__(self, world: carla.World,
                 vehicle: carla.Vehicle,
                 logger=None,
                 debug=False):
        self.carla_world = world
        self.ego_vehicle = vehicle
        self.logger = logger
        self.debug = debug

        # self.routing_listener = ApolloRoutingListener(self.carla_world,
        #                                               ego_vehicle=self.ego_vehicle,
        #                                               logger=self.logger,
        #                                               debug=self.debug)
        self.routing_listener = ApolloRoutingListener(self.carla_world,
                                                      ego_vehicle=self.ego_vehicle,
                                                      logger=self.logger,
                                                      debug=self.debug)

        self.segments = []

        self.stop_listening = False

    def interpolate_location(start, end, fraction):
        x = start.x + (end.x - start.x) * fraction
        y = start.y + (end.y - start.y) * fraction
        z = start.z + (end.z - start.z) * fraction
        return carla.Location(x, y, z)

    def get_curr_seg():
        pass

    def get_next_seg():
        pass

    def wait_for_route(self, req_time, interval=-2):
        self.stop_listening = False
        while self.routing_listener.recv_time == None or \
                (self.routing_listener.recv_time - req_time) <= interval:
            if self.stop_listening == True:
                break
            time.sleep(0.1)

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
        among_wps = wp.previous_until_lane_start(length)
        for among_wp in among_wps:
            segs.append(Segment(among_wp.transform.location,
                                among_wp.transform.rotation, length, width))
        segs.reverse()
        return segs

    def get_seg_between_two_wp(self, wp_s: carla.Waypoint,
                               wp_e: carla.Waypoint,
                               length: float, width: float):
        segs = []
        if wp_s == None:
            segs = self.get_seg_from_straight_to_end_wp(wp_e, length, width)
            return segs
        wp = wp_s
        max = int(wp_s.transform.location.distance(
            wp_e.transform.location)/length) + 3
        # Ensure accurate division even in curves
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
        try:
            next_wps = wp_touple[0].next_until_lane_end(length)
            among_wps = [wp_touple[0]] + next_wps[:-1]
        except RuntimeError as e:
            try:
                prev_wps = wp_touple[1].previous_until_lane_start(length)
                prev_wps.reverse()
                among_wps = prev_wps
            except RuntimeError as e:
                if self.debug and self.logger != None:
                    self.logger.warning(f'{wp_touple}: fail {e}')
                among_wps.append(wp_touple[0])
        for among_wp in among_wps:
            segs.append(Segment(among_wp.transform.location,
                                among_wp.transform.rotation, length, width))
        return segs

    def get_segments(self, length: float, width: float):
        segs = []
        with self.routing_listener.lock:
            routing_wps = self.routing_listener.routing_wps
        pdb.set_trace()
        for i, route_wp in enumerate(routing_wps):
            if route_wp[1].is_junction:
                segs += self.get_seg_from_junction_wp(route_wp[0],
                                                      length, width)
                continue
            if i == len(routing_wps) - 1:
                # handle the final segment
                segs += self.get_seg_between_two_wp(route_wp[0],
                                                    route_wp[1],
                                                    length, width)
                continue
            segs += self.get_seg_from_full_road(route_wp, length, width)

        if self.debug:
            carla_db = self.carla_world.debug
            for i, seg in enumerate(segs):
                if seg.bbox == None:
                    continue
                carla_db.draw_string(seg.location,
                                     f"segmentation #{i} junction: {seg.is_junction}",
                                     color=carla.Color(0, 191, 255),
                                     life_time=60)
                carla_db.draw_box(seg.bbox, seg.rotation, thickness=0.1,
                                  color=carla.Color(0, 191, 255), life_time=60)

        return segs


if __name__ == '__main__':
    from loguru import logger
    client = carla.Client('172.17.0.1', 5000)
    world = client.get_world()

    SS = SceneSegment(world, vehicle=None, debug=True, logger=logger)

    def signal_handler(sig, frame):
        SS.routing_listener.stop()
        SS.stop_listening = True
        exit()

    signal.signal(signal.SIGINT, signal_handler)

    SS.routing_listener.start()

    SS.wait_for_route(time.time())
    SS.get_segments(30, 30)
    SS.routing_listener.stop()
