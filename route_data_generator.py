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

    x1 = np.random.uniform(min(x_coords), max(x_coords))
    y1 = np.random.uniform(min(y_coords), max(y_coords))
    diff_x = np.random.uniform(20, 50)
    diff_y = np.random.uniform(20, 50)
    x2 = x1 + diff_x
    y2 = y1 + diff_y
    start_location = carla.Location(x=x1, y=y1, z=0.0)
    end_location = carla.Location(x=x2, y=y2, z=0.0)

    route = get_route(world, start_location, end_location)
    route_x = [wp.transform.location.x for wp in route]
    route_y = [wp.transform.location.y for wp in route]

    for i in range(1, len(route)):
        prev_wp = route[i - 1]
        curr_wp = route[i]
        prev_location = (prev_wp.transform.location.x, prev_wp.transform.location.y)
        curr_location = (curr_wp.transform.location.x, curr_wp.transform.location.y)
        prev_angle = calculate_angle(prev_location, curr_location)

        if i < len(route) - 1:
            next_wp = route[i + 1]
            next_location = (next_wp.transform.location.x, next_wp.transform.location.y)
            next_angle = calculate_angle(curr_location, next_location)
            angle_diff = next_angle - prev_angle

            if angle_diff > 0.1:
                print(f"Point {i}: Right Turn")
            elif angle_diff < -0.1:
                print(f"Point {i}: Left Turn")
            else:
                print(f"Point {i}: Straight")

    plt.plot(route_x, route_y, c='red', linewidth=2, label='Vehicle Route')
    for i in range(1, len(route)):
        plt.arrow(route_x[i - 1], route_y[i - 1], route_x[i] - route_x[i - 1], route_y[i] - route_y[i - 1],
                  head_width=1.0, head_length=2.0, fc='red', ec='red')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Top View of All Roads in CARLA with Curve Indicators and Vehicle Route')
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    try:
        world = connect_to_carla()
        for i in range(10):
            plot_road_top_view(world)
    except Exception as e:
        print(f"Error: {e}")
