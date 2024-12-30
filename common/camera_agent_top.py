import carla
import threading
import cv2
import numpy as np
import time
import datetime
from carla import ColorConverter as cc
import os
from packaging import version


# import pygame


class ScenarioRecorder:
    def __init__(self,
                 world: carla.World,
                 cam_tf: carla.Transform,
                 save_floder_path,
                 resolution=(832, 468),
                 frame_rate=24.0, ):
        """
        Initializes the ScenarioRecorder class to record scenarios in the CARLA simulator.

        Parameters:
        - world (carla.World): The simulation world.
        - ego_vehicle (carla.Vehicle): The vehicle being controlled or observed in the simulation.
        - save_folder_path (str): Directory path for saving the recordings.
        - resolution (tuple): Image resolution for recording, default (832,468). Choose from
            [   320x180,    384x216,
                448x252,    512x288,
                576x324,    640x360,
                704x396,    768x432,
                832x468,    896x504,
                960x540,    1024x576
                and those larger then 1280x720
            ].
        - frame_rate (float): Recording frame rate in frames per second, default 24.0.
        """
        self.world = world
        self.cam_tf = cam_tf
        self.save_path = save_floder_path
        self.frame_rate = frame_rate

        self.sub_fig_res = resolution

        self.top_cam = None

        top_cam_loc = self.cam_tf.location + carla.Location(z=30)
        top_camm_rot = carla.Rotation(pitch=-90,
                                      yaw=self.cam_tf.rotation.yaw - 90)
        self.top_cam_tf = carla.Transform(top_cam_loc,
                                          top_camm_rot)

        self.top_cam = self.create_camera(self.top_cam_tf, 'recorder_top_cam')

        self.video_writer: cv2.VideoWriter = None
        self.stop_event = threading.Event()  # Initialize the stop event
        self.recording_thread: threading.Thread = None

        self.width, self.height = self.sub_fig_res
        # [top_frame]
        self.sensor_frame = None

        self.recording_frame_size = (self.height, self.width, 3)
        self.recording_frame = np.zeros(self.recording_frame_size,
                                        dtype=np.uint8)

    def create_camera(self, transform, role_name, attachment_type=carla.AttachmentType.Rigid):
        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.sub_fig_res[0]))
        cam_bp.set_attribute('image_size_y', str(self.sub_fig_res[1]))
        cam_bp.set_attribute('sensor_tick', str(1.0 / self.frame_rate))
        cam_bp.set_attribute('role_name', role_name)
        camera = self.world.spawn_actor(
            cam_bp, transform, attachment_type=attachment_type)
        self.world.wait_for_tick()
        return camera

    def top_img_callback(self, image):
        self.sensor_frame = image
        # print('callback')

    def start_recording(self, save_path=None):
        self.recording_frame = np.zeros(self.recording_frame_size,
                                        dtype=np.uint8)
        width, height = self.sub_fig_res

        self.top_cam.listen(self.top_img_callback)

        if save_path == None:
            if not os.path.exists(self.save_path):
                os.makedirs(self.save_path)
            curr_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            save_path = os.path.join(
                self.save_path, f'recording-{curr_datetime}.avi')

        # Create video writer for the final stitched video
        fourcc = cv2.VideoWriter_fourcc(*'XVID')

        self.stop_event.clear()

        final_frame_size = (width, height)

        print('creating video writer')
        self.video_writer = cv2.VideoWriter(save_path,
                                            fourcc,
                                            self.frame_rate,
                                            final_frame_size)
        print('video writer created')
        self.recording_thread = threading.Thread(
            target=self.recording_thread_handler)
        self.recording_thread.start()

    def recording_thread_handler(self):
        self.stop_event.clear()
        while not self.stop_event.is_set():
            if self.sensor_frame == None:
                continue

            start_time = time.time()

            record_array = np.empty((self.height, self.width, 3),
                                    dtype=np.uint8)

            array = np.frombuffer(self.sensor_frame.raw_data, dtype=np.dtype(
                "uint8")).reshape((self.sensor_frame.height, self.sensor_frame.width, 4))
            # Convert image to an array for video recording
            record_array = array[:, :, :3]

            self.recording_frame = record_array

            self.video_writer.write(self.recording_frame)

            elapsed_time = time.time() - start_time
            sleep_time = max(0, (1.0 / self.frame_rate) - elapsed_time)
            time.sleep(sleep_time)
        # array = np.frombuffer(self.sensor_frame[0].raw_data, dtype=np.dtype("uint8"))
        # print(array.shape)
        # array = array.reshape((self.sensor_frame[0].height, self.sensor_frame[0].width, 4))
        # print(array.shape)
        # print(self.sensor_frame[0].height*self.sensor_frame[0].width*4)
        # # CARLA's images are BGRA, not RGB
        # bgra_image = array[:, :, :4]  # Include alpha for saving to file
        # rgb_image = cv2.cvtColor(bgra_image, cv2.COLOR_BGRA2RGB)
        # cv2.imwrite('output_image.png', rgb_image)

    def stop_recording(self):
        self.stop_event.set()
        self.recording_thread.join()
        self.video_writer.release()

        self.top_cam.stop()
        self.recording_frame = np.zeros(self.recording_frame_size,
                                        dtype=np.uint8)

    def __del__(self):
        self.top_cam.destroy()


if __name__ == '__main__':
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    print(
        f'world connnected, working in synchronous_mode:{settings.synchronous_mode}')

    time.sleep(1)

    ego_vehicle = None
    spawn_one = False
    for actor in world.get_actors().filter('vehicle.*'):
        # print(actor.attributes.get('role_name'))
        if actor.attributes.get('role_name') in ['hero', 'ego_vehicle']:
            ego_vehicle = actor
            break
    if ego_vehicle == None:
        print("No ego vehicle found, spawning one")
        bps = world.get_blueprint_library().filter('vehicle.tesla.model3')

        ego_tf = world.get_map().get_spawn_points()[0]
        ego_bp = bps[0]
        ego_bp.set_attribute('role_name', 'ego_vehicle')
        ego_vehicle = world.try_spawn_actor(ego_bp, ego_tf)
        spawn_one = True
    print("Ego vehicle found")

    recorder = ScenarioRecorder(world,
                                ego_vehicle,
                                f'./save',
                                server_version=client.get_server_version())
    print("Starting first recording, will record for 20 seconds")
    recorder.start_recording()
    start = time.time()
    while time.time() - start < 20:
        time.sleep(0.1)
    recorder.stop_recording()
    print("Recording stopped")

    # print("Starting second recording, will record for 20 seconds")
    # recorder.start_recording()
    # start = time.time()
    # while time.time() - start < 20:
    #     time.sleep(0.1)
    # recorder.stop_recording()
    # print("Recording stopped")

    if spawn_one:
        ego_vehicle.destroy()
