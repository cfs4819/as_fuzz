import threading
import time
from cyber_py3 import cyber
from carla_bridge.utils.transforms import cyber_point_to_carla_location
from modules.common_msgs.planning_msgs import planning_pb2
import carla


class PlanningListener:
    def __init__(self, carla_world,logger):
        self.world = carla_world
        self.plan_points = []
        self.logger = logger
        self.lock = threading.Lock()  # for plan_points

        self.stop_signal = False
        self.main_thread = None

    def start(self):
        self.main_thread = threading.Thread(target=self.run)
        self.main_thread.start()

    def run(self):
        cyber.init()
        self.node = cyber.Node("planning_listener")
        self.node.create_reader(
            "/apollo/planning", planning_pb2.ADCTrajectory, self.planning_callback)
        while not self.stop_signal:
            time.sleep(0.1)
        print("cyber shutting down")
        cyber.shutdown()

    def stop(self):
        self.stop_signal = True
        self.main_thread.join()

    def planning_callback(self, planning_data: planning_pb2.ADCTrajectory):
        new_plan_points = []
        print(f"new msg: len is {len(planning_data.trajectory_point)}")
        for trajectory_point in planning_data.trajectory_point:
            location = cyber_point_to_carla_location(
                trajectory_point.path_point)
            new_plan_points.append(location)

        with self.lock:
            self.plan_points = new_plan_points

        self.update_debug_drawings()

    def update_debug_drawings(self):
        with self.lock:
            for location in self.plan_points:
                self.world.debug.draw_point(
                    location, size=0.05, color=carla.Color(255, 0, 0), life_time=0.5)

    def get_plan_points(self):
        with self.lock:
            return list(self.plan_points)


if __name__ == "__main__":
    client = carla.Client('172.17.0.1', 5000)
    world = client.get_world()

    planning_listener = PlanningListener(world)
    planning_listener.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        planning_listener.stop()
