from cyber.python.cyber_py3 import cyber
from modules.common_msgs.routing_msgs.routing_pb2 import RoutingResponse
import carla
import time

class ApolloRoutingListener:
    def __init__(self, carla_world):
        self.world = carla_world
        self.map = self.world.get_map()
        cyber.init()
        self.routing = []
        self.routing_wp = []
        self.recv_time = time.time()
        self.node = cyber.Node("routing_listener_node")

    def listen_to_routing_response(self):
        self.node.create_reader(
            "/apollo/routing_response", RoutingResponse, self.routing_callback)

    def routing_callback(self, routing_response):
        print(f"Received routing response at {time.time()}")
        self.recv_time = routing_response.header.timestamp_sec
        self.routing = []
        first_segment = routing_response.road[0].passage[0].segment[0]
        self.routing.append(f'{first_segment.id}_{first_segment.end_s}')
        self.routing_wp = []
        first_wp_t = self.LaneSegment_to_wp_touple(first_segment)
        self.routing_wp.append(first_wp_t[0])
        for road in routing_response.road:
            for segment in road.passage[0].segment:
                # print(f"\t segment {segment.id}")
                lane_wp_s, lane_wp_e = self.LaneSegment_to_wp_touple(
                    segment)
                self.routing.append(f'{segment.id}_{segment.end_s}')
                self.routing_wp.append(lane_wp_e)

        # print(f'{self.routing}')

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
        # print(wp_e, wp_s)
        return (wp_e, wp_s)

    def start(self):
        self.listen_to_routing_response()
        self.node.spin()

    def stop(self):
        cyber.shutdown()


if __name__ == '__main__':
    client = carla.Client('172.17.0.1', 5000)
    world = client.get_world()

    listener = ApolloRoutingListener(world)
    try:
        listener.start()
    except KeyboardInterrupt:
        print("Shutting down listener.")
    finally:
        listener.stop()
