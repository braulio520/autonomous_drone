import carla
import time
import random
import cv2
import glob
import os
import numpy as np

# Initialize CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()

# Find a blueprint for a vehicle
model_3_blueprint = world.get_blueprint_library().filter('model3')[0]

# Find all possible spawn points for vehicles
spawn_points = world.get_map().get_spawn_points()

# Select a random spawn point
spawn_point = random.choice(spawn_points)

# Spawn the drone
vehicle = world.spawn_actor(model_3_blueprint, spawn_point)

# Find a blueprint for a camera
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

# Set camera attributes
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '110')

# Define the location and orientation of the camera on the vehicle
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

# Add the camera to the vehicle you've spawned
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# Define a callback function to process the images captured by the camera
def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    cv2.imwrite(os.path.expanduser(f'~/Desktop/output/{image.frame:06d}.png'), array)

# Add the callback function to the camera
camera.listen(lambda image: process_image(image))

# Get the spectator
spectator = world.get_spectator()

start_time = time.time()

while True:
    world_snapshot = world.wait_for_tick()
    current_time = time.time()
    transform = vehicle.get_transform()

    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                            carla.Rotation(pitch=-90)))
    # Drive the vehicle forward
    vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))

    # If 10 seconds have passed, stop the vehicle
    if current_time - start_time > 10:
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

    # If 12 seconds have passed, stop the loop
    if current_time - start_time > 12:
        break

    time.sleep(0.01)

# Stop and destroy the vehicle and the camera
camera.stop()
camera.destroy()
vehicle.destroy()

# Pause for a moment to let images write to disk
time.sleep(2)

# Convert images to video
img_array = []
size = (0, 0)
for filename in sorted(glob.glob(os.path.expanduser('~/Desktop/output/*.png'))):
    img = cv2.imread(filename)
    if img is None:
        print(f"Failed to load image file {filename}")
        continue
    height, width, layers = img.shape
    size = (width, height)
    img_array.append(img)

if len(img_array) > 0:
    out = cv2.VideoWriter(os.path.expanduser('~/Desktop/video.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 30, size)
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
else:
    print("No images were found to write.")

