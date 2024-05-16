import carla
import time
from enum import Enum
from threading import Thread, Lock, Event


class UNSAFE_TYPE(Enum):
    COLLISION = 1
    CROSSING_SOLID_LANE = 2
    LANE_CHANGE = 3
    STUCK = 4
    ACCELERATION = 5


class UnsafeDetector(object):
    def __init__(self, world: carla.World, vehicle: carla.Vehicle):
        self.world = world
        self.vehicle = vehicle
        self.map = self.world.get_map()
        self.lane_change_detector = None
        self.collision_detector = None
        self.callbacks = []

        self.active_timers = {}
        self.timers_lock = Lock()
        self.threshold_time = 5.0

        self.stuck_event = Event()
        self.stuck_thread = None
        self.velocity_threshold = 0.1
        self.stuck_timeout = 60.0

        self.acceleration_event = Event()
        self.acceleration_thread = None
        self.acceleration_threshold = 5.0

    def init_sensors(self):
        # Setup the lane invasion and collision sensors
        lane_invasion_bp = self.world.get_blueprint_library().find(
            'sensor.other.lane_invasion')
        self.lane_change_detector = self.world.spawn_actor(lane_invasion_bp,
                                                           carla.Transform(),
                                                           attach_to=self.vehicle)

        collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_detector = self.world.spawn_actor(collision_bp,
                                                         carla.Transform(),
                                                         attach_to=self.vehicle)

    def start_detection(self):
        """Starts the detection of unsafe situations."""
        self.lane_change_detector.listen(self.on_lane_invasion)
        self.collision_detector.listen(self.on_collision)

        self.start_stuck_monitor(self.stuck_timeout)
        self.start_acceleration_monitor()
    
    def stop_detection(self):
        """Stops the detection of unsafe situations."""
        self.lane_change_detector.stop()
        self.collision_detector.stop()
        self.stop_acceleration_monitor() 
        self.stop_stuck_monitor()
    def start_stuck_monitor(self, timeout=60):
        """Starts a thread to monitor if the vehicle is stuck based on its velocity."""
        if self.stuck_thread and self.stuck_thread.is_alive():
            return
        self.stuck_event.clear()
        self.stuck_thread = Thread(target=self.monitor_stuck, args=(timeout,))
        self.stuck_thread.start()

    def start_acceleration_monitor(self, unsafe_threshold=5.0):
        """Starts a thread to monitor if the vehicle's acceleration exceeds the threshold."""
        if self.acceleration_thread and self.acceleration_thread.is_alive():
            return
        self.acceleration_event.clear()
        self.acceleration_thread = Thread(target=self.monitor_acceleration,
                                          args=(unsafe_threshold,))
        self.acceleration_thread.start()

    def monitor_stuck(self, timeout):
        """Monitor vehicle's velocity and trigger callbacks if stuck for too long."""
        start_time = None

        while not self.stuck_event.is_set():
            current_velocity = self.vehicle.get_velocity()
            speed = current_velocity.length()

            if speed < self.velocity_threshold:
                if start_time is None:
                    start_time = time.time()
            else:
                start_time = None

            if start_time and (time.time() - start_time) >= timeout:
                self.trigger_callbacks(
                    UNSAFE_TYPE.STUCK, "Vehicle has been static for too long.")
                start_time = None

            time.sleep(0.05)

    def monitor_acceleration(self, unsafe_threshold):
        """Monitor vehicle's acceleration and trigger callbacks if exceeds threshold."""
        while not self.acceleration_event.is_set():
            current_acceleration = self.vehicle.get_acceleration()
            acceleration_magnitude = current_acceleration.length()
            acc = current_acceleration.x
            if acceleration_magnitude > unsafe_threshold:
                self.trigger_callbacks(UNSAFE_TYPE.ACCELERATION,
                                       f"High acceleration detected: \
                                           {acc:3.2f} m/s^2")

            time.sleep(0.01)

    def stop_stuck_monitor(self):
        """Stops the stuck monitoring thread."""
        if self.stuck_event:
            self.stuck_event.set()
        if self.stuck_thread:
            self.stuck_thread.join()

    def stop_acceleration_monitor(self):
        """Stops the acceleration monitoring thread."""
        if self.acceleration_event:
            self.acceleration_event.set()
        if self.acceleration_thread:
            self.acceleration_thread.join()

    def on_collision(self, event):
        # Handle collision events
        self.trigger_callbacks(UNSAFE_TYPE.COLLISION, 'Collision detected')

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def on_lane_invasion(self, event: carla.LaneInvasionEvent):
        for marking in event.crossed_lane_markings:
            if marking.type in [carla.LaneMarkingType.Solid,
                                carla.LaneMarkingType.SolidSolid]:
                self.trigger_callbacks(
                    UNSAFE_TYPE.CROSSING_SOLID_LANE, 'Crossed a solid line')
                return
        # Handle lane invasion events
        waypoint = self.map.get_waypoint(
            self.vehicle.get_location(), project_to_road=False)
        uid = f"{waypoint.road_id}_{waypoint.section_id}"

        with self.timers_lock:
            if uid not in self.active_timers:
                stop_event = Event()
                timer_thread = Thread(target=self.lane_occupation_timer,
                                      args=(waypoint.road_id, waypoint.section_id, stop_event))
                self.active_timers[uid] = (timer_thread, stop_event)
                timer_thread.start()

    def crossing_two_lane(self):
        bounding_box = self.vehicle.bounding_box
        vehicle_transform = self.vehicle.get_transform()
        corners = bounding_box.get_world_vertices(vehicle_transform)[2:6]

        road_ids, section_ids, lane_ids = set(), set(), set()
        for corner in corners:
            waypoint = self.world.get_map().get_waypoint(corner, project_to_road=False)
            if not waypoint:
                continue
            road_ids.add(waypoint.road_id)
            section_ids.add(waypoint.section_id)
            lane_ids.add(waypoint.lane_id)

        if len(road_ids) == 1 and len(section_ids) == 1 and len(lane_ids) > 1:
            return True
        return False

    def lane_occupation_timer(self, road_id, section_id, stop_event: Event):
        start_time = time.time()
        while not stop_event.is_set() :
            if not self.crossing_two_lane():
                # no more crossing two lane
                break
            waypoint = self.map.get_waypoint(
                self.vehicle.get_location(), project_to_road=False)
            if not waypoint:
                continue
            if road_id != waypoint.road_id or section_id != waypoint.section_id:
                # left this area
                break
            if time.time() - start_time >= self.threshold_time:
                # time out
                self.trigger_callbacks(
                    UNSAFE_TYPE.LANE_CHANGE, 'Long time between lanes')
                start_time = time.time()

            time.sleep(0.1)

        with self.timers_lock:
            uid = f"{road_id}_{section_id}"
            del self.active_timers[uid]

    def trigger_callbacks(self, type, message):
        # Call registered callback functions
        for callback in self.callbacks:
            callback(type, message)

    def cleanup(self):
        # Clean up the sensors and stop all timers
        try:
            with self.timers_lock:
                for uid, (thread, stop_event) in self.active_timers.items():
                    stop_event.set()  # Signal the event to stop the thread
                    thread.join()  # Wait for the thread to finish
                self.active_timers.clear()

            if self.lane_change_detector:
                self.lane_change_detector.stop()
                self.lane_change_detector.destroy()
            if self.collision_detector:
                self.collision_detector.stop()
                self.collision_detector.destroy()

            self.stop_stuck_monitor()
            self.stop_acceleration_monitor()
        except Exception as e:
            print(f"Error cleaning up sensors and timers: {e}")


if __name__ == '__main__':
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    print(
        f'world connnected, working in synchronous_mode:{settings.synchronous_mode}')

    time.sleep(1)

    ego_vehicle = None
    for actor in world.get_actors().filter('vehicle.*'):
        # print(actor.attributes.get('role_name'))
        if actor.attributes.get('role_name') in ['hero', 'ego_vehicle']:
            ego_vehicle = actor
            break
    if ego_vehicle == None:
        print("No ego vehicle found")
        exit()
    print("Ego vehicle found")

    decector = UnsafeDetector(world, ego_vehicle)

    def callback(type, message):
        if type == UNSAFE_TYPE.ACCELERATION:
            print(f'{message}\r', end=' ')
        else:
            print(message)

    decector.register_callback(callback)
    decector.init_sensors()

    while True:
        print("Press 'q' to quit or any other key to continue...")
        key = input()
        if key.lower() == 'q':
            break

    decector.cleanup()
