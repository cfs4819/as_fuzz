from cyber.python.cyber_py3 import cyber
from modules.common_msgs.routing_msgs.routing_pb2 import RoutingResponse
import carla
import time
import threading
import signal

# from agents.tools.misc import draw_waypoints
import math


class ApolloRoutingListener:
    def __init__(self, carla_world, ego_vehicle=None, logger=None, debug=False):
        self.world = carla_world
        self.map = self.world.get_map()
        self.ego_vehicle = ego_vehicle
        self.logger = logger
        self.debug = debug

        self.runing = False
        self.stop_signal = False
        self.main_thread = None

        self.routing = []
        self.routing_wp = []
        self.recv_time = time.time()

        self.node = None

    def start(self):
        self.main_thread = threading.Thread(target=self.run)
        self.main_thread.start()

    def run(self):
        cyber.init()
        self.node = cyber.Node("routing_listener_node")
        self.node.create_reader(
            "/apollo/routing_response", RoutingResponse, self.routing_callback)
        self.runing = True
        self.stop_signal = False
        while not self.stop_signal:
            time.sleep(0.1)
        if self.logger != None:
            self.logger.info("routing_listener_node shutting down")
        cyber.shutdown()

    def stop(self):
        self.stop_signal = True
        self.main_thread.join()
        self.runing = False

    def routing_callback(self, routing_response):
        if self.debug and self.logger != None:
            self.logger.info(f"Received routing response at {time.time()}")
        self.recv_time = routing_response.header.timestamp_sec
        self.routing = []
        first_segment = routing_response.road[0].passage[0].segment[0]
        self.routing.append(f'{first_segment.id}_{first_segment.end_s}')
        self.routing_wp = []
        first_wp_t = self.LaneSegment_to_wp_touple(first_segment)
        self.routing_wp.append(first_wp_t[0])
        for road in routing_response.road:
            for segment in road.passage[0].segment:
                lane_wp_s, lane_wp_e = self.LaneSegment_to_wp_touple(
                    segment)
                self.routing.append(f'{segment.id}_{segment.end_s}')
                self.routing_wp.append(lane_wp_e)

        if self.debug:
            if self.logger != None:
                self.logger.info(f"waypoints:{self.routing_wp}")
            self.draw_routing_wps()

    def draw_routing_wps(self):
        if self.routing_wp[0] == None and self.ego_vehicle != None:
            self.routing_wp[0] = self.map.get_waypoint(
                self.ego_vehicle.get_transform().location)

        drawing_wps = []

        if len(self.routing_wp) <= 2:
            drawing_wps = self.routing_wp
        else:
            if self.routing_wp[0] != None:
                drawing_wps += self.routing_wp[0].next_until_lane_end(10)

            for i in range(2, len(self.routing_wp)):
                prv_wps = self.routing_wp[i].previous_until_lane_start(10)
                drawing_wps += prv_wps

        for wpt in drawing_wps:
            wpt_t = wpt.transform
            begin = wpt_t.location + carla.Location(z=0.5)
            angle = math.radians(wpt_t.rotation.yaw)
            end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
            self.world.debug.draw_arrow(begin, end, color=carla.Color(
                192, 255, 62), arrow_size=0.3, life_time=60)

    def LaneSegment_to_wp_touple(self, lane_segment):
        # return the start waypoint and the end waypoint
        if lane_segment.id == None or lane_segment.id == '':
            return ()
        road_id = lane_segment.id.split('_')
        if len(road_id) != 5:
            return ()
        wp_s = self.map.get_waypoint_xodr(int(road_id[1]),
                                          int(road_id[4]),
                                          float(lane_segment.start_s))

        wp_e = self.map.get_waypoint_xodr(int(road_id[1]),
                                          int(road_id[4]),
                                          float(lane_segment.end_s))
        return (wp_e, wp_s)


if __name__ == '__main__':
    client = carla.Client('172.17.0.1', 5000)
    world = client.get_world()

    routing_listener = ApolloRoutingListener(world, debug=True)
    routing_listener.start()

    def signal_handler(sig, frame):
        routing_listener.stop()
        exit()

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        time.sleep(1)
