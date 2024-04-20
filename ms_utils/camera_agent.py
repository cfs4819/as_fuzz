import carla
import threading
import cv2
import numpy as np
import os

class CameraRecorder:
    def __init__(self, parent_actor, relative_transform, rotation_angle, resolution, save_path):
        # Initialize the CARLA client and world
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Store the parameters
        self.parent_actor = parent_actor
        self.relative_transform = carla.Transform(carla.Location(*relative_transform), carla.Rotation(pitch=rotation_angle[0], yaw=rotation_angle[1], roll=rotation_angle[2]))
        self.resolution = resolution
        self.save_path = save_path
        self.capture = True
        self.frames = []

        # Create the camera blueprint
        blueprint_library = self.world.get_blueprint_library()
        cam_blueprint = blueprint_library.find('sensor.camera.rgb')
        cam_blueprint.set_attribute('image_size_x', str(self.resolution[0]))
        cam_blueprint.set_attribute('image_size_y', str(self.resolution[1]))
        cam_blueprint.set_attribute('fov', '110')

        # Spawn the camera attached to the parent actor
        self.camera = self.world.spawn_actor(cam_blueprint, self.relative_transform, attach_to=self.parent_actor)

        # Set up the listener
        self.camera.listen(self.image_callback)

        # Initialize threading
        self.thread = threading.Thread(target=self.run)
        self.stop_event = threading.Event()

    def image_callback(self, image):
        # Convert image to an array for video recording
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        self.frames.append(array)

    def run(self):
        while not self.stop_event.is_set():
            if self.frames:
                frame = self.frames.pop(0)
                self.video_writer.write(frame)

    def start_recording(self):
        # Create video writer object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(self.save_path, fourcc, 20.0, (self.resolution[0], self.resolution[1]))
        self.thread.start()

    def stop_recording(self):
        self.stop_event.set()
        self.thread.join()
        self.video_writer.release()

    def __del__(self):
        self.camera.destroy()
