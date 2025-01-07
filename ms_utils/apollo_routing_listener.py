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

        self.running = False
        self.stop_signal = False
        self.main_thread = None
        self.lock = threading.Lock()  # for plan_points

        self.routing = []
        self.routing_wps = []
        self.recv_time = None
        self.req_time = None

        self.node = None
        self.subscriber = None

    def start(self, node_name="routing_listener_node"):
        self.main_thread = threading.Thread(target=self.run, args=(node_name,))
        self.main_thread.start()

    def run(self, node_name="routing_listener_node"):
        # we don't init cyber here
        # cyber.init()
        self.node = cyber.Node(node_name)
        self.subscriber = self.node.create_reader(
            "/apollo/routing_response", RoutingResponse, self.routing_callback)
        self.running = True
        self.stop_signal = False
        while not self.stop_signal:
            time.sleep(0.1)
        if self.logger != None:
            self.logger.info("routing_listener_node shutting down")

        del self.subscriber
        del self.node

        # we don't stop cyber here
        # cyber.shutdown()

    def stop(self):
        self.stop_signal = True
        self.main_thread.join()
        self.running = False

    def routing_callback(self, routing_response):
        if self.debug and self.logger != None:
            self.logger.info(f"Received routing response at {time.time()}")
        self.recv_time = routing_response.header.timestamp_sec
        self.req_time = routing_response.routing_request.header.timestamp_sec
        with self.lock:
            self.routing = []
            self.routing_wps = []
            for i, road in enumerate(routing_response.road):
                if road == None:
                    continue
                for j, segment in enumerate(road.passage[0].segment):
                    if segment == None:
                        continue
                    lane_wp_s, lane_wp_e = self.LaneSegment_to_wp_touple(
                        segment)
                    ''' debug
                    time.sleep(1)
                    if lane_wp_s != None:
                        self.world.debug.draw_point(lane_wp_s.transform.location +
                                    carla.Location(0, 0, 2), life_time=10)
                        self.world.debug.draw_string(lane_wp_s.transform.location,
                                     f"{segment.id}\n start",
                                     life_time=10)
                    time.sleep(1)

                    if lane_wp_e != None:
                        self.world.debug.draw_point(lane_wp_e.transform.location +
                                    carla.Location(0, 0, 2), life_time=10)
                        self.world.debug.draw_string(lane_wp_e.transform.location,
                                     f"{segment.id} end",
                                     life_time=10)
                    '''
                    self.routing.append(
                        f'{segment.id}_{int(segment.start_s)}_{int(segment.end_s)}')
                    self.routing_wps.append([lane_wp_s, lane_wp_e])
        if self.debug:
            if self.logger != None:
                # self.logger.info(f"waypoints:{self.routing_wps}")
                self.logger.info(f"Recvs waypoints:{len(self.routing_wps)}")
            self.draw_routing_wps()

    def draw_routing_wps(self):
        drawing_wps = []

        if len(self.routing_wps) <= 2:
            drawing_wps = self.routing_wps
        else:
            for i, wp in enumerate(self.routing_wps[:-1]):
                if wp[0] == None or wp[1] == None:
                    continue
                if self.debug:
                    self.world.debug.draw_point(wp[0].transform.location +
                                                carla.Location(0, 0, 1), life_time=20)
                    self.world.debug.draw_string(wp[0].transform.location,
                                                 f"wp #{i} is_junction:{wp[0].is_junction}",
                                                 life_time=20)
                try:
                    next_wps = wp[0].next_until_lane_end(5)
                    drawing_wps += next_wps
                except RuntimeError as e:
                    try:
                        prev_wps = wp[1].previous_until_lane_start(5)
                        prev_wps.reverse()
                        drawing_wps += prev_wps
                    except RuntimeError as e:
                        if self.debug and self.logger != None:
                            self.logger.warning(f'wp #{i}: fail {e}')
                        drawing_wps.append(wp[0])
                        continue
            try:
                final_seg = self.routing_wps[-1][1].previous_until_lane_start(5)
                final_seg.reverse()
                drawing_wps += final_seg
            except RuntimeError as e:
                if self.debug and self.logger != None:
                    self.logger.warning(f'wp #{-1}: previous_until_lane_start {e}')
                drawing_wps.append(self.routing_wps[-1][1])

        for wpt in drawing_wps:
            wpt_t = wpt.transform
            begin = wpt_t.location + carla.Location(z=0.5)
            angle = math.radians(wpt_t.rotation.yaw)
            end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
            self.world.debug.draw_arrow(begin, end, color=carla.Color(
                192, 255, 62), arrow_size=0.3, life_time=30)

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
        i = 0
        wp_e = None
        while wp_e == None:
            wp_e = self.map.get_waypoint_xodr(int(road_id[1]),
                                              int(road_id[4]),
                                              float(int(lane_segment.end_s) - i))
            i += 1
            if i >= lane_segment.end_s:
                break
        return (wp_s, wp_e)


if __name__ == '__main__':
    from loguru import logger

    client = carla.Client('172.17.0.1', 4000)
    world = client.get_world()

    cyber.init()

    routing_listener = ApolloRoutingListener(world, debug=True, logger=logger)
    routing_listener.start()


    def signal_handler(sig, frame):
        routing_listener.stop()
        cyber.shutdown()
        exit()


    signal.signal(signal.SIGINT, signal_handler)

    while True:
        time.sleep(1)
