import carla
import threading
import cv2
import numpy as np
import time
import datetime
from carla import ColorConverter as cc
import os


class ScenarioRecorder:
    def __init__(self,
                 world: carla.World,
                 ego_vehicle: carla.Vehicle,
                 save_floder_path,
                 resolution=(1280, 720),
                 frame_rate=24.0):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.save_path = save_floder_path
        self.frame_rate = frame_rate

        self.sub_fig_res = resolution

        self.top_cam_tf = carla.Transform(carla.Location(x=0, y=0.0, z=30),
                                          carla.Rotation(pitch=-90, yaw=-90, roll=0))

        self.tpp_cam_tf = carla.Transform(carla.Location(x=-5.5, y=0.0, z=3),
                                          carla.Rotation(pitch=1, yaw=0, roll=0))

        self.fpp_cam_tf = carla.Transform(carla.Location(x=0, y=0.0, z=1.8),
                                          carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

        self.back_cam_tf = carla.Transform(carla.Location(x=-0.8, y=0.0, z=1.8),
                                           carla.Rotation(pitch=0, yaw=-180, roll=0))

        self.cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.cam_bp.set_attribute('image_size_x', str(self.sub_fig_res[0]))
        self.cam_bp.set_attribute('image_size_y', str(self.sub_fig_res[1]))
        self.cam_bp.set_attribute('sensor_tick', str(1.0 / self.frame_rate))

        self.cam_bp.set_attribute('role_name', 'recorder_top_cam')
        self.top_cam = self.world.spawn_actor(self.cam_bp,
                                              self.top_cam_tf,
                                              attach_to=self.ego_vehicle)
        self.world.wait_for_tick()

        self.cam_bp.set_attribute('role_name', 'recorder_tpp_cam')
        self.tpp_cam = self.world.spawn_actor(self.cam_bp,
                                              self.tpp_cam_tf,
                                              attach_to=self.ego_vehicle,
                                              attachment_type=carla.AttachmentType.SpringArmGhost)
        self.world.wait_for_tick()

        self.cam_bp.set_attribute('role_name', 'recorder_fpp_cam')
        self.fpp_cam = self.world.spawn_actor(self.cam_bp,
                                              self.fpp_cam_tf,
                                              attach_to=self.ego_vehicle)
        self.world.wait_for_tick()

        self.cam_bp.set_attribute('role_name', 'recorder_back_cam')
        self.back_cam = self.world.spawn_actor(self.cam_bp,
                                               self.back_cam_tf,
                                               attach_to=self.ego_vehicle)
        self.world.wait_for_tick()

        self.video_writer: cv2.VideoWriter = None
        self.stop_event = threading.Event()  # Initialize the stop event
        self.recording_thread: threading.Thread = None

        self.width, self.height = self.sub_fig_res
        self.recording_frame = np.zeros(
            (self.height * 2, self.width * 2, 3), dtype=np.uint8)

    def top_img_callback(self, image):
        # Convert image to an array for video recording
        top_array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        top_array = np.reshape(top_array, (image.height, image.width, 4))
        top_array = top_array[:, :, :3]     # Remove alpha channel

        self.recording_frame[:self.height, :self.width, :] = top_array

    def tpp_img_callback(self, image):
        # Convert image to an array for video recording
        tpp_array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        tpp_array = np.reshape(tpp_array, (image.height, image.width, 4))
        tpp_array = tpp_array[:, :, :3]     # Remove alpha channel

        self.recording_frame[:self.height,
                             self.width:self.width*2, :] = tpp_array

    def fpp_img_callback(self, image):
        # Convert image to an array for video recording
        fpp_array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        fpp_array = np.reshape(fpp_array, (image.height, image.width, 4))
        fpp_array = fpp_array[:, :, :3]     # Remove alpha channel

        self.recording_frame[self.height:self.height *
                             2, :self.width, :] = fpp_array

    def back_img_callback(self, image):
        # Convert image to an array for video recording
        back_array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        back_array = np.reshape(back_array, (image.height, image.width, 4))
        back_array = back_array[:, :, :3]     # Remove alpha channel

        self.recording_frame[self.height:self.height *
                             2, self.width:self.width*2, :] = back_array

    def start_recording(self, save_path=None):
        self.recording_frame = np.zeros(
            (self.height * 2, self.width * 2, 3), dtype=np.uint8)
        width, height = self.sub_fig_res
        final_frame_size = (width * 2, height * 2)

        self.top_cam.listen(self.top_img_callback)
        self.tpp_cam.listen(self.tpp_img_callback)
        self.fpp_cam.listen(self.fpp_img_callback)
        self.back_cam.listen(self.back_img_callback)

        if save_path == None:
            if not os.path.exists(self.save_path):
                os.makedirs(self.save_path)
            curr_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            save_path = os.path.join(
                self.save_path, f'recording-{curr_datetime}.avi')

        # Create video writer for the final stitched video
        fourcc = cv2.VideoWriter_fourcc(*'XVID')

        self.stop_event.clear()
        self.video_writer = cv2.VideoWriter(save_path,
                                            fourcc,
                                            self.frame_rate,
                                            final_frame_size)
        self.recording_thread = threading.Thread(
            target=self.recording_thread_handler)
        self.recording_thread.start()

    def recording_thread_handler(self):
        while not self.stop_event.is_set():
            self.video_writer.write(self.recording_frame)

            time.sleep(1.0 / self.frame_rate)

    def stop_recording(self):
        self.stop_event.set()
        self.recording_thread.join()
        self.video_writer.release()

        self.top_cam.stop()
        self.tpp_cam.stop()
        self.fpp_cam.stop()
        self.back_cam.stop()
        self.recording_frame = np.zeros(
            (self.height * 2, self.width * 2, 3), dtype=np.uint8)

    def __del__(self):
        self.top_cam.destroy()
        self.tpp_cam.destroy()
        self.fpp_cam.destroy()
        self.back_cam.destroy()


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

    recorder = ScenarioRecorder(world, ego_vehicle, f'./save')
    print("Starting first recording, will record for 20 seconds")
    recorder.start_recording()
    start = time.time()
    while time.time() - start < 20:
        time.sleep(0.1)
    recorder.stop_recording()
    print("Recording stopped")
    
    print("Starting second recording, will record for 20 seconds")
    recorder.start_recording()
    start = time.time()
    while time.time() - start < 20:
        time.sleep(0.1)
    recorder.stop_recording()
    print("Recording stopped")
