
import carla
import time
import random
import numpy as np
import open3d as o3d

# Initialize CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()

# Find a blueprint for a drone
model_3_blueprint = world.get_blueprint_library().filter('model3')[0]

# Find all possible spawn points for vehicles
spawn_points = world.get_map().get_spawn_points()

# Select a random spawn point
spawn_point = random.choice(spawn_points)

# Spawn the drone
vehicle = world.spawn_actor(model_3_blueprint, spawn_point)

# Find a blueprint for LiDAR
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('points_per_second', '100000')
lidar_bp.set_attribute('channels', '32')
lidar_bp.set_attribute('range', '10000')
lidar_bp.set_attribute('upper_fov', '10')
lidar_bp.set_attribute('lower_fov', '-10')

# Find a blueprint for GPS
gps_bp = world.get_blueprint_library().find('sensor.other.gnss')

# Define the location and orientation of the sensors on the vehicle
sensor_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

# Add LiDAR and GPS sensors to the vehicle
lidar_sensor = world.spawn_actor(lidar_bp, sensor_transform, attach_to=vehicle)
gps_sensor = world.spawn_actor(gps_bp, sensor_transform, attach_to=vehicle)

# define a callback function to process the lidar data
def process_lidar_data(lidar_data):
    points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    points = points[:, :3]  # we only take x, y, z
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    print(f"Lidar data captured at frame {lidar_data.frame}")

# Define a callback function to process the GPS data
def process_gps_data(gps_data):
    print(f"GPS data: Latitude = {gps_data.latitude}, Longitude = {gps_data.longitude}, Altitude = {gps_data.altitude}")

# Add the callback functions to the sensors
lidar_sensor.listen(lambda lidar_data: process_lidar_data(lidar_data))
gps_sensor.listen(lambda gps_data: process_gps_data(gps_data))

# Sleep for a moment to allow sensor data to come in
time.sleep(2)

# Stop and destroy the vehicle and the sensors
lidar_sensor.stop()
gps_sensor.stop()
lidar_sensor.destroy()
gps_sensor.destroy()
vehicle.destroy()
