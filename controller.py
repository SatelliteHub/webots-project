from controller import Robot
import math
import csv
import numpy as np

filename = '../../OccupancyGrid.csv'
delimiter = ','
occupancy_grid = np.genfromtxt(filename, delimiter=delimiter, dtype=str)
occupancy_grid[occupancy_grid == ''] = '0'
occupancy_grid = np.char.strip(occupancy_grid, "'")
print(occupancy_grid)

CELL_SIZE = 0.1


TIME_STEP = 64
robot = Robot()
ds = []
dsNames = ['ds_right', 'ds_left']

for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


def getGpsData():
    gp = robot.getDevice("gps1")
    gp.enable(TIME_STEP)
    gpsData = gp.getValues()
    print("X:", gpsData[0])
    print("Y:", gpsData[1])
    print("Z:", gpsData[2])
    return gpsData
    
def getRobotPose():
    global x, y, theta
    gpsData = getGpsData()
    
    x_gps = gpsData[0]
    y_gps = gpsData[1]

    print("x_gps: " + str(x_gps) + ", y_gps: " + str(y_gps))
    return (x_gps, y_gps)
    
def heuristic_cost_estimate(start, goal):
    return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)


def is_within_bounds(point, grid):
    return 0 <= point[0] < grid.shape[0] and 0 <= point[1] < grid.shape[1]


def get_neighbors(point, grid):
    neighbors = []

    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        new_point = (point[0] + dx, point[1] + dy)

        if is_within_bounds(new_point, grid) and grid[new_point] == '0':
            neighbors.append(new_point)

    return neighbors


def a_star(start, goal, grid):
    start = tuple(start)
    goal = tuple(goal)

    open_set = [start]
    came_from = {}

    g_score = {point: float("inf") for point in np.ndindex(grid.shape)}
    g_score[start] = 0

    f_score = {point: float("inf") for point in np.ndindex(grid.shape)}
    f_score[start] = heuristic_cost_estimate(start, goal)

    while open_set:
        current = min(open_set, key=lambda point: f_score[point])

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        open_set.remove(current)

        for neighbor in get_neighbors(current, grid):
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
                if neighbor not in open_set:
                    open_set.append(neighbor)

    return None
    
def control_robot_along_path(robot_location, path):
    if len(path) < 2:
        return 0.0, 0.0

    next_point = path[1]

    next_point_world = (
        (next_point[1] * CELL_SIZE),
        (next_point[0] * CELL_SIZE),
    )

    angle_to_next_point = math.atan2(
        next_point_world[0] - robot_location[0], next_point_world[1] - robot_location[1]
    )

    angle_diff = angle_to_next_point - theta

    # Ensure the angle difference is within the range of -pi to pi
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    angular_speed = angle_diff * 10
    linear_speed = 1.0

    left_speed = linear_speed - angular_speed * 0.102 / 2
    right_speed = linear_speed + angular_speed * 0.102 / 2

    return left_speed, right_speed



# Convert the goal coordinates (3, -4) to array indices
goal_x, goal_y = 3, -4
# goal_index = (goal_y + occupancy_grid.shape[0] // 2, goal_x + occupancy_grid.shape[1] // 2)
goal_index = (goal_y / CELL_SIZE*-1, goal_x /CELL_SIZE)


# Find the path using the A* algorithm
while robot.step(TIME_STEP) != -1:
    gpsData = getGpsData()
    x = 0.0
    y = 0.0
    theta = 0.0

    robot_location = getRobotPose()

    print("robot location: ", end='')
    print(robot_location)
    
    # robot_location_index = (
    #     int(round((robot_location[1] + occupancy_grid.shape[0] // 2) * CELL_SIZE)),
    #     int(round((robot_location[0] + occupancy_grid.shape[1] // 2) * CELL_SIZE)),
    # )

    robot_location_index = [int(robot_location[1]/CELL_SIZE*-1), int(robot_location[0]/CELL_SIZE)]

    print("robot loc index: ", end='')
    print(robot_location_index)

    path = a_star(robot_location_index, goal_index, occupancy_grid)

    if path:
        print("Path found:", path)
        leftSpeed, rightSpeed = control_robot_along_path(robot_location, path)
    else:
        print("No path found")
        leftSpeed = 0.0
        rightSpeed = 0.0

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)
