import carla
import time
import csv
import cv2
import numpy as np

# 连接到CARLA服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 设置自动驾驶地图
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.*model3')[0]

# 定位点生成
spawn_points = world.get_map().get_spawn_points()
vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])

# 自动驾驶启用
vehicle.set_autopilot(True)

# 创建相机传感器
camera_bp = blueprint_library.find('sensor.camera.rgb')
# 设置分辨率
# camera_bp.set_attribute('image_size_x', '1920')  # 设置图像的宽度 (水平分辨率)
# camera_bp.set_attribute('image_size_y', '1080')  # 设置图像的高度 (垂直分辨率)
# 设置帧率 (FPS)
fps = 20
camera_bp.set_attribute('sensor_tick', str(1.0 / fps))

camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# 创建保存视频和数据的列表
frames = []
vehicle_data = []

# 保存视频
def save_video(frames, filename="carla_video.avi"):
    height, width, _ = frames[0].shape
    print("视频分辨率：{}x{}".format(width, height))
    print("帧数：{}".format(len(frames)))
    video = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), 20, (width, height))

    for frame in frames:
        video.write(frame)

    video.release()

# 保存车辆数据
def save_vehicle_data(data, filename="vehicle_data.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Location_X", "Location_Y", "Location_Z", "Velocity_X", "Velocity_Y", "Velocity_Z",
                         "Throttle", "Brake", "Steer", "Gear", "Frame_Index"])
        for row in data:
            writer.writerow(row)

# 获取车辆数据
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

# 回调函数从相机传感器获取帧并保存车辆数据
frame_count = 0
def process_img(image):
    global frame_count
    image_data = np.array(image.raw_data)
    image_data = image_data.reshape((image.height, image.width, 4))
    image_data = image_data[:, :, :3]  # 转换为RGB
    frames.append(image_data)

    # 同步保存车辆数据
    vehicle_state = get_vehicle_data()
    vehicle_state.append(frame_count)  # 添加帧索引，确保图片和车辆数据对应
    vehicle_data.append(vehicle_state)

    frame_count += 1

camera.listen(lambda image: process_img(image))

# 记录10秒视频和汽车相关数据
start_time = time.time()
interval = 1.0 / fps
while time.time() - start_time < 10:
    world.tick()
    time.sleep(interval)

# 停止相机监听并保存视频和数据
camera.stop()
save_video(frames)
save_vehicle_data(vehicle_data)

# 清理actor
vehicle.destroy()
camera.destroy()

print("视频和车辆数据已保存。")
