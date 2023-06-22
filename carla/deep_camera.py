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

# Spawn the vehicle
vehicle = world.spawn_actor(model_3_blueprint, spawn_point)

# Find a blueprint for a camera
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
depth_camera_bp = world.get_blueprint_library().find('sensor.camera.depth')

# Set camera attributes
rgb_camera_bp.set_attribute('image_size_x', '800')
rgb_camera_bp.set_attribute('image_size_y', '600')
rgb_camera_bp.set_attribute('fov', '110')

depth_camera_bp.set_attribute('image_size_x', '800')
depth_camera_bp.set_attribute('image_size_y', '600')
depth_camera_bp.set_attribute('fov', '110')

# Define the location and orientation of the camera on the vehicle
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

# Add the camera to the vehicle you've spawned
rgb_camera = world.spawn_actor(rgb_camera_bp, camera_transform, attach_to=vehicle)
depth_camera = world.spawn_actor(depth_camera_bp, camera_transform, attach_to=vehicle)

# Define a callback function to process the images captured by the camera
def process_rgb_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    cv2.imwrite(os.path.expanduser(f'~/Desktop/output/rgb_{image.frame:06d}.png'), array)

def process_depth_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    cv2.imwrite(os.path.expanduser(f'~/Desktop/output/depth_{image.frame:06d}.png'), array)

# Add the callback function to the camera
rgb_camera.listen(lambda image: process_rgb_image(image))
depth_camera.listen(lambda image: process_depth_image(image))

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
rgb_camera.stop()
depth_camera.stop()
rgb_camera.destroy()
depth_camera.destroy()
vehicle.destroy()

# Pause for a moment to let images write to disk
time.sleep(2)

# Convert images to video
img_array_rgb = []
img_array_depth = []
size = (0, 0)
for filename in sorted(glob.glob(os.path.expanduser('~/Desktop/output/*.png'))):
    img = cv2.imread(filename)
    if img is None:
        print(f"Failed to load image file {filename}")
        continue
    if "rgb" in filename:
        img_array_rgb.append(img)
    elif "depth" in filename:
        img_array_depth.append(img)
    height, width, layers = img.shape
    size = (width, height)

if len(img_array_rgb) > 0:
    out = cv2.VideoWriter(os.path.expanduser('~/Desktop/video_rgb.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 30, size)
    for i in range(len(img_array_rgb)):
        out.write(img_array_rgb[i])
    out.release()
else:
    print("No RGB images were found to write.")

if len(img_array_depth) > 0:
    out = cv2.VideoWriter(os.path.expanduser('~/Desktop/video_depth.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 30, size)
    for i in range(len(img_array_depth)):
        out.write(img_array_depth[i])
    out.release()
else:
    print("No depth images were found to write.")

