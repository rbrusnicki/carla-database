import carla
import time
import csv
import cv2
import numpy as np

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.*model3')[0]

spawn_points = world.get_map().get_spawn_points()
vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])

vehicle.set_autopilot(True)

camera_bp = blueprint_library.find('sensor.camera.rgb')

fps = 20
camera_bp.set_attribute('sensor_tick', str(1.0 / fps))

camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

frames = []
vehicle_data = []

def save_video(frames, filename="carla_video.avi"):
    height, width, _ = frames[0].shape
    print("resolution:{}x{}".format(width, height))
    print("frame count: {}".format(len(frames)))
    video = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), 20, (width, height))

    for frame in frames:
        video.write(frame)

    video.release()

def save_vehicle_data(data, filename="vehicle_data.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Location_X", "Location_Y", "Location_Z", "Velocity_X", "Velocity_Y", "Velocity_Z",
                         "Throttle", "Brake", "Steer", "Gear", "Frame_Index"])
        for row in data:
            writer.writerow(row)
            
def save_combined_data(data, filename="vehicle_and_task_data.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", 
                         "Location_X", "Location_Y", "Location_Z", 
                         "Velocity_X", "Velocity_Y", "Velocity_Z",
                         "Throttle", "Brake", "Steer", "Gear", 
                         "Is_Junction", "Traffic_Light_Count",
                         "Nearby_Vehicle_Count", "Nearby_Pedestrian_Count",
                         "Cloudiness", "Precipitation", "Precipitation_Deposits",
                         "Wind_Intensity", "Sun_Azimuth_Angle", "Sun_Altitude_Angle",
                         "Frame_Index"])
        for row in data:
            writer.writerow(row)    

def get_vehicle_data():
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    control = vehicle.get_control()

    return [
        time.time(),
        transform.location.x, transform.location.y, transform.location.z,
        velocity.x, velocity.y, velocity.z,
        control.throttle, control.brake, control.steer, control.gear
    ]
    
def generate_task_description():
    weather = world.get_weather()
    weather_data = [
        weather.cloudiness,
        weather.precipitation,
        weather.precipitation_deposits,
        weather.wind_intensity,
        weather.sun_azimuth_angle,
        weather.sun_altitude_angle
    ]
    
    current_waypoint = world.get_map().get_waypoint(vehicle.get_location())
    
    # check for junction
    is_junction = current_waypoint.is_junction
    
    # traffic lights
    traffic_lights = world.get_actors().filter('traffic.traffic_light*')
    relevant_lights = []
    for tl in traffic_lights:
        if tl.get_transform().location.distance(vehicle.get_location()) < 50:
            relevant_lights.append(tl)
    
    # nearby vehicles
    other_vehicles = world.get_actors().filter('vehicle.*')
    nearby_vehicles = []
    for ov in other_vehicles:
        if ov.id != vehicle.id and ov.get_transform().location.distance(vehicle.get_location()) < 50:
            nearby_vehicles.append(ov)
            
    # nearby pedestrians
    pedestrians = world.get_actors().filter('walker.pedestrian.*')
    nearby_pedestrians = []
    for pedestrian in pedestrians:
        if pedestrian.get_transform().location.distance(vehicle.get_location()) < 50:
            nearby_pedestrians.append(pedestrian)
    
    # store task description
    task_description = [
        is_junction,
        len(relevant_lights),
        len(nearby_vehicles),
        len(nearby_pedestrians),
    ] + weather_data
    
    return task_description

frame_count = 0
def process_img(image):
    global frame_count
    image_data = np.array(image.raw_data)
    image_data = image_data.reshape((image.height, image.width, 4))
    image_data = image_data[:, :, :3] 
    frames.append(image_data)

    # vechicle data
    vehicle_state = get_vehicle_data()
    
    # task description data
    task_description = generate_task_description()
    
    # combine data
    combined_data = vehicle_state + task_description + [frame_count]
    vehicle_data.append(combined_data)

    frame_count += 1

camera.listen(lambda image: process_img(image))

start_time = time.time()
interval = 1.0 / fps
while time.time() - start_time < 10:
    world.tick()
    time.sleep(interval)

camera.stop()
save_video(frames)
#save_vehicle_data(vehicle_data)
save_combined_data(vehicle_data)

vehicle.destroy()
camera.destroy()

print("data saved")