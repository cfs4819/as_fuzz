import threading
import time
from cyber_py3 import cyber
from carla_bridge.utils.transforms import cyber_point_to_carla_location
from modules.common_msgs.planning_msgs import planning_pb2
import carla


class PlanningListener: 
    def __init__(self, carla_world, logger):
        self.world = carla_world
        self.plan_points = []
        self.logger = logger
        self.lock = threading.Lock()  # for plan_points

        self.stop_signal = False
        self.main_thread = None

        self.stop_reason = None
        self.stop_reason_lock = threading.Lock()

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
        self.extract_stop_reason(planning_data)

    def extract_stop_reason(self, planning_data: planning_pb2.ADCTrajectory):
        """Extract stop reason from planning data."""
        stop_decision = planning_data.decision.main_decision.stop
        stop_reason_code = stop_decision.reason_code
        stop_reason_description = stop_decision.reason

        with self.stop_reason_lock:
            self.stop_reason = {
                "reason_code": stop_reason_code,
                "reason_description": stop_reason_description
            }

        print(f"Stop reason detected: {self.stop_reason}")

    def get_stop_reason(self):
        """Retrieve the last detected stop reason."""
        with self.stop_reason_lock:
            return self.stop_reason

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

    logger = None
    planning_listener = PlanningListener(world, logger)
    planning_listener.start()

    try:
        while True:
            time.sleep(1)
            stop_reason = planning_listener.get_stop_reason()
            if stop_reason:
                print(f"Current stop reason: {stop_reason}")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        planning_listener.stop()
