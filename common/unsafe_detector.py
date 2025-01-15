from typing import Dict

import carla
import time
# from enum import Enum
from threading import Thread, Lock, Event

from planning.is_stuck import RoadBlockageChecker, is_vehicle_in_front, resolve_stuck_vehicles, is_vehicle_accelerating, \
    is_vehicle_around


class UNSAFE_TYPE():
    COLLISION = 1
    CROSSING_SOLID_LANE = 2
    LANE_CHANGE = 3
    STUCK = 4
    ACCELERATION = 5
    ROAD_BLOCKED = 6
    BLOCK_STUCK = 7

    type_str = ['NONE',
                'COLLISION',
                'CROSSING_SOLID_LANE',
                'LANE_CHANGE',
                'STUCK',
                'ACCELERATION',
                'ROAD_BLOCKED',
                'BLOCK_STUCK']


class UnsafeDetector(object):
    def __init__(self, world: carla.World,
                 vehicle: carla.Vehicle,
                 try_relese_block=False):
        self.world = world
        self.vehicle = vehicle
        self.try_relese_block = try_relese_block

        self.map = self.world.get_map()
        self.lane_change_detector = None
        self.collision_detector = None
        self.imu_sensor = None  # Add IMU sensor attribute
        self.callbacks = []

        # Road blockage detection attributes
        self.road_blockage_event = Event()
        self.road_blockage_thread = None
        # Road blockage checker
        self.road_blockage_checker = RoadBlockageChecker(self.map, world)
        self.distance_threshold = 2.5  # Example threshold for road blockage

        self.active_timers = {}
        self.timers_lock = Lock()
        self.threshold_time = 2.0

        self.stuck_event = Event()
        self.stuck_thread = None
        self.velocity_threshold = 0.1
        self.stuck_timeout = 60.0

        self.total_stuck_time = 0.0
        self.total_block_time = 0.0

        self.acceleration_threshold = 5.0

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def is_road_blocked(self) -> Dict:
        """
        Checks if any road is blocked based on the current state of the world.
        :return: Dictionary containing blockage status and details.
        """
        slow_vehicles = [
            actor for actor in self.world.get_actors()
            if "vehicle" in actor.type_id
               and actor.get_velocity().length() < 0.5
        ]
        if self.vehicle in slow_vehicles:
            slow_vehicles.remove(self.vehicle)

        slow_vehicles_in_front = []
        for slow_vehicle in slow_vehicles:
            if is_vehicle_in_front(self.vehicle, slow_vehicle) or is_vehicle_around(self.vehicle, slow_vehicle):
                slow_vehicles_in_front.append(slow_vehicle)
        return self.road_blockage_checker.is_road_blocked(slow_vehicles_in_front, self.distance_threshold)

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
        # Setup IMU sensor
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        self.imu_sensor = self.world.spawn_actor(
            imu_bp, carla.Transform(), attach_to=self.vehicle)

    def start_detection(self):
        """Starts the detection of unsafe situations."""
        self.lane_change_detector.listen(self.on_lane_invasion)
        self.collision_detector.listen(self.on_collision)
        self.imu_sensor.listen(self.on_imu_data)  # Add IMU data listener

        self.start_stuck_monitor(self.stuck_timeout)
        self.start_road_blockage_monitor()

    def stop_detection(self):
        """Stops the detection of unsafe situations."""
        self.lane_change_detector.stop()
        self.collision_detector.stop()
        self.imu_sensor.stop()  # Stop IMU sensor
        self.stop_stuck_monitor()
        self.stop_road_blockage_monitor()

    def start_stuck_monitor(self, timeout=60):
        """Starts a thread to monitor if the vehicle is stuck based on its velocity."""
        if self.stuck_thread and self.stuck_thread.is_alive():
            return
        self.stuck_event.clear()
        self.stuck_thread = Thread(target=self.monitor_stuck, args=(timeout,))
        self.stuck_thread.start()

    def monitor_stuck(self, timeout):
        """Monitor vehicle's velocity and trigger callbacks if stuck for too long."""
        start_time = None
        last_calc_time = time.time()
        while not self.stuck_event.is_set():
            current_velocity = self.vehicle.get_velocity()
            speed = current_velocity.length()

            if speed < self.velocity_threshold:
                if start_time is None:
                    start_time = time.time()
                    last_calc_time = time.time()
            else:
                start_time = None

            if start_time and (time.time() - start_time) >= timeout:
                self.trigger_callbacks(
                    UNSAFE_TYPE.STUCK, "Vehicle has been static for too long.")
                start_time = None
            elif start_time:
                self.total_stuck_time += time.time() - last_calc_time
                last_calc_time = time.time()
            time.sleep(0.05)

    def stop_stuck_monitor(self):
        """Stops the stuck monitoring thread."""
        if self.stuck_event:
            self.stuck_event.set()
        if self.stuck_thread:
            self.stuck_thread.join()

    def start_road_blockage_monitor(self):
        """Starts a thread to monitor road blockage."""
        if self.road_blockage_thread and self.road_blockage_thread.is_alive():
            return  # Avoid restarting an already running thread

        self.road_blockage_event.clear()
        self.road_blockage_thread = Thread(target=self.monitor_road_blockage)
        self.road_blockage_thread.start()

    def stop_road_blockage_monitor(self):
        """Stops the road blockage monitoring thread."""
        if self.road_blockage_event:
            self.road_blockage_event.set()
        if self.road_blockage_thread:
            self.road_blockage_thread.join()

    def monitor_road_blockage(self):
        """
        Thread for monitoring road blockage.
        - Detects road blockage and triggers callbacks.
        - Ensures a 3-second gap between consecutive blockage detections.
        - Records total blockage time.

        - branch:
                ┌─────────────────────────────────────┐
                │ while not road_blockage_event:      │
                │   road_blockage_result = ...        │
                └─────────────────────────────────────┘
                            ▼
                    road_blockage_result["blocked"]?
                            │
                    ┌─────────┴─────────┐
                    │                   │
                    │ Yes (Branch A)    │ No (Branch B)
                    │                   │
                    ▼                   ▼
                ┌────────────────────┐  ┌───────────────────────────────┐
                │ if blockage_start: │  │ if blockage_start_time ...    │
                │ A1: first-time     │  │ B1: blockage -> resolved,     │
                │     blockage       │  │     accumulate time           │
                └────────────────────┘  └───────────────────────────────┘
                    ▼
                ┌─────────────────────────────┐
                │ if callback_triggered:      │
                │ A2: callback already fired  │
                └─────────────────────────────┘
                    ▼
                ┌────────────────────────────────────────────────┐
                │ A2a: <3.0s => sleep(0.5) + continue            │
                │ A2b: >=3.0s => reset last_resolve_time         │
                │ A2c: >=10.0s => resolve_stuck_vehicles()       │
                └────────────────────────────────────────────────┘
                    ▼
                ┌────────────────────────────────────────────────────────────┐
                │ A3: trigger_callbacks => returns whether callback fired    │
                └────────────────────────────────────────────────────────────┘
                            ▼
                    time.sleep(0.5)
                        (loop continues)

        """
        blockage_start_time = None  # Timestamp when blockage is detected
        last_calc_time = time.time()  # Timestamp for last time increment
        callback_triggered = False  # Whether a callback has been triggered

        while not self.road_blockage_event.is_set():
            road_blockage_result = self.is_road_blocked()

            # No blockage at present
            if not road_blockage_result["blocked"]:
                # Blockage resolved
                if blockage_start_time is not None:
                    # self.total_block_time += time.time() - blockage_start_time
                    blockage_start_time = None
                callback_triggered = False
                last_calc_time = time.time()  # Reset last_calc_time for accurate tracking
                time.sleep(0.5)
                continue

            # Blockage detected
            current_time = time.time()
            if blockage_start_time is None:
                blockage_start_time = current_time
                last_calc_time = current_time

            # Increment total blockage time
            self.total_block_time += current_time - last_calc_time
            last_calc_time = current_time

            if not self.try_relese_block:
                time.sleep(0.5)
                continue

            if callback_triggered:
                elapsed_since_resolve = current_time - last_calc_time
                if elapsed_since_resolve < 3.0:
                    time.sleep(0.5)
                    continue

                # Reset resolve time for new blockage handling
                last_calc_time = current_time

                if current_time - blockage_start_time >= 10.0:
                    def condition(vehicle):
                        # The vehicle is in front of ego and not speeding up
                        return is_vehicle_in_front(self.vehicle, vehicle) \
                            and vehicle.get_velocity().length() < 1.0
                    resolve_stuck_vehicles(
                        road_blockage_result["vehicles_on_blocked_road"], condition, 1, 3)

            # Trigger callback for road blockage
            callback_triggered = self.trigger_callbacks(
                UNSAFE_TYPE.ROAD_BLOCKED,
                f"Road {road_blockage_result['blocked_road_id']} is blocked.",
                road_blockage_result["vehicles_on_blocked_road"]
            )[0]

            time.sleep(0.5)

    def on_collision(self, event):
        # Handle collision events
        self.trigger_callbacks(UNSAFE_TYPE.COLLISION, 'Collision detected')

    def on_lane_invasion(self, event: carla.LaneInvasionEvent):
        for marking in event.crossed_lane_markings:
            if marking.type == carla.LaneMarkingType.Solid:
                self.trigger_callbacks(UNSAFE_TYPE.CROSSING_SOLID_LANE,
                                       'Crossed a single solid line',
                                       1)
                return
            elif marking.type == carla.LaneMarkingType.SolidSolid:
                self.trigger_callbacks(UNSAFE_TYPE.CROSSING_SOLID_LANE,
                                       'Crossed a double solid line',
                                       2)
                return
            # if marking.type in [carla.LaneMarkingType.Solid,
            #                     carla.LaneMarkingType.SolidSolid]:
            #     self.trigger_callbacks(
            #         UNSAFE_TYPE.CROSSING_SOLID_LANE, 'Crossed a solid line')
            #     return
        # Handle lane invasion events
        waypoint = self.map.get_waypoint(
            self.vehicle.get_location(), project_to_road=False)
        uid = f"{waypoint.road_id}_{waypoint.section_id}"

        with self.timers_lock:
            if uid not in self.active_timers:
                # print(f'timer {uid}, started')
                stop_event = Event()
                timer_thread = Thread(target=self.lane_occupation_timer,
                                      args=(waypoint.road_id, waypoint.section_id, stop_event))
                self.active_timers[uid] = (timer_thread, stop_event)
                timer_thread.start()

    def on_imu_data(self, data):
        """Callback function for IMU sensor data."""
        acceleration = data.accelerometer
        acceleration_magnitude = acceleration.x
        if abs(acceleration_magnitude) > self.acceleration_threshold:
            self.trigger_callbacks(
                UNSAFE_TYPE.ACCELERATION,
                f"High acceleration detected: {acceleration_magnitude:.2f} m/s^2",
                round(acceleration_magnitude, 2)
            )

    def crossing_two_lane(self):
        bounding_box = self.vehicle.bounding_box
        vehicle_transform = self.vehicle.get_transform()
        corners = bounding_box.get_world_vertices(vehicle_transform)[2:6]

        road_ids, section_ids, lane_ids = set(), set(), set()
        for corner in corners:
            waypoint = self.map.get_waypoint(corner, project_to_road=False)
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

        while not stop_event.is_set():
            '''debug'''
            # bounding_box = self.vehicle.bounding_box
            # vehicle_transform = self.vehicle.get_transform()
            # corners = bounding_box.get_world_vertices(vehicle_transform)
            # db = self.world.debug
            # for index, cor in enumerate(corners):
            #     db.draw_point(cor, size=0.1, life_time=0.07, color=carla.Color(255, 0, 0))

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

            time.sleep(0.05)

        uid = f"{road_id}_{section_id}"
        del self.active_timers[uid]

    def trigger_callbacks(self, type, message, data=None):
        """
        Call registered callback functions and collect their return values.
        """
        results = []
        for callback in self.callbacks:
            result = callback(type, message, data)
            results.append(result)

        return results

    def cleanup(self):
        # Clean up the sensors and stop all timers
        try:
            with self.timers_lock:
                for uid, (thread, stop_event) in self.active_timers.items():
                    stop_event.set()  # Signal the event to stop the thread
                    if thread.is_alive():
                        thread.join()  # Wait for the thread to finish
                self.active_timers.clear()

            if self.lane_change_detector:
                self.lane_change_detector.stop()
                self.lane_change_detector.destroy()
            if self.collision_detector:
                self.collision_detector.stop()
                self.collision_detector.destroy()
            if self.imu_sensor:  # Clean up IMU sensor
                self.imu_sensor.stop()
                self.imu_sensor.destroy()

            self.stop_stuck_monitor()
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

    def callback(type, message, data):
        if type == UNSAFE_TYPE.ACCELERATION:
            # print(f'{message}\r', end=' ')
            pass
        else:
            print(message)

    decector.register_callback(callback)
    decector.init_sensors()
    decector.start_detection()

    while True:
        print("Press 'q' to quit or any other key to continue...")
        key = input()
        if key.lower() == 'q':
            break

    decector.cleanup()
