import carla
import matplotlib.pyplot as plt
import numpy as np

def connect_to_carla(host='localhost', port=2000):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    return world

def calculate_angle(p1, p2):
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

def get_route(world, start_location, end_location):
    carla_map = world.get_map()
    start_wp = carla_map.get_waypoint(start_location)
    end_wp = carla_map.get_waypoint(end_location)

    current_wp = start_wp
    route = [current_wp]
    while current_wp.transform.location.distance(end_wp.transform.location) > 2.0:
        next_wps = current_wp.next(2.0)
        if not next_wps:
            break
        current_wp = next_wps[0]
        if current_wp.transform.location.distance(start_wp.transform.location) > 50.0:
            break
        route.append(current_wp)
    return route

def plot_road_top_view(world):
    carla_map = world.get_map()
    waypoints = carla_map.generate_waypoints(distance=2.0)

    x_coords = []
    y_coords = []
    for waypoint in waypoints:
        transform = waypoint.transform
        location = transform.location
        x_coords.append(location.x)
        y_coords.append(location.y)

    plt.figure(figsize=(10, 10))
    plt.scatter(x_coords, y_coords, s=1, c='blue')

    spawn_points = world.get_map().get_spawn_points()
    spawn_x = [spawn_point.location.x for spawn_point in spawn_points]
    spawn_y = [spawn_point.location.y for spawn_point in spawn_points]
    plt.scatter(spawn_x, spawn_y, s=50, c='purple', label='Spawn Points')

    start_spawn_point = np.random.choice(spawn_points)
    start_location = start_spawn_point.location

    start_wp = carla_map.get_waypoint(start_location)

    
    end_location = start_wp.next(50)[-1].transform.location
    route = get_route(world, start_location, end_location)
    route_x = [wp.transform.location.x for wp in route]
    route_y = [wp.transform.location.y for wp in route]
    
    angles = []
    for i in range(1, len(route_x)):
        dx = route_x[i] - route_x[i - 1]
        dy = route_y[i] - route_y[i - 1]
        angle = np.arctan2(dy, dx)
        angles.append(angle)

    # 判断角度变化
    turns = []
    for i in range(1, len(angles)):
        delta_angle = angles[i] - angles[i - 1]
        delta_angle = (delta_angle + np.pi) % (2 * np.pi) - np.pi  # 将角度差值归一化到 [-pi, pi]

        if delta_angle > 0.03:  # 阈值可以根据需求调整
            turns.append(carla.LaneChange.Left)
        elif delta_angle < -0.03:
            turns.append(carla.LaneChange.Right)
        else:
            turns.append(carla.LaneChange.NONE)

    if carla.LaneChange.Left in turns and carla.LaneChange.Right in turns:
        print("Left and Right Turns Detected")
    elif carla.LaneChange.Left in turns:
        print("Left Turn Detected")
    elif carla.LaneChange.Right in turns:
        print("Right Turn Detected")
    else:
        print("No Turns Detected")
    

    plt.plot(route_x[0], route_y[0], c='red', linewidth=2, label='Vehicle Route')
    for i in range(1, len(route)):
        plt.arrow(route_x[i - 1], route_y[i - 1], route_x[i] - route_x[i - 1], route_y[i] - route_y[i - 1],
                  head_width=1.0, head_length=2.0, fc='red', ec='red')
    

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Top View of All Roads in CARLA with Curve Indicators, Vehicle Route, and Spawn Points')
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    try:
        world = connect_to_carla()
        for i in range(5):
            plot_road_top_view(world)
    except Exception as e:
        print(f"Error: {e}")
