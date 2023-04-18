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


def initGps(TIME_STEP, robot):
    gps = []
    gpsNames = ['gps1', 'gps2']
    value = [[0, 0, 0], [0, 0, 0]]

    for i in range(len(gpsNames)):
        gps.append(robot.getDevice(gpsNames[i]))
        gps[i].enable(TIME_STEP)

    return gps, value


def initWheels(robot):
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

    for i in range(len(wheelsNames)):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(2.0)

    return wheels


# Initialize GPS and Wheels
gps, gps_values = initGps(TIME_STEP, robot)
wheels = initWheels(robot)


def getGpsData(gps, value):
    value[0] = gps[0].getValues()
    value[1] = gps[1].getValues()
    # print("GPS 1:", value[0])
    # print("GPS 2:", value[1])
    return value


def getRobotPose(gps_value):
    robot_location = [(gps_value[0][0] + gps_value[1][0]) / 2,
                      (gps_value[0][1] + gps_value[1][1]) / 2]
    orientation_vector = [gps_value[0][1] - gps_value[1]
                          [1], -1 * (gps_value[0][0] - gps_value[1][0])]
    mag_orientation_vector = math.sqrt(
        pow(orientation_vector[0], 2) + pow(orientation_vector[1], 2))
    orientation_unit_vector = [0, 0]

    if mag_orientation_vector != 0:
        orientation_unit_vector[0] = orientation_vector[0] / \
            mag_orientation_vector
        orientation_unit_vector[1] = orientation_vector[1] / \
            mag_orientation_vector

    return robot_location, orientation_unit_vector


def getTargetOrientationVector(goal, robot_location):
    target_orientation_vector = [goal[0]-robot_location[0], goal[1]-robot_location[1]]
    mag_target_orientation_vector = math.sqrt (pow(target_orientation_vector[0],2)+pow(target_orientation_vector[1],2))
    target_orientation_unit_vector = [0, 0]
    
    if mag_target_orientation_vector != 0:
        target_orientation_unit_vector[0] = target_orientation_vector[0]/mag_target_orientation_vector
        target_orientation_unit_vector[1] = target_orientation_vector[1]/mag_target_orientation_vector
    
    return target_orientation_unit_vector


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
                f_score[neighbor] = g_score[neighbor] + \
                    heuristic_cost_estimate(neighbor, goal)
                if neighbor not in open_set:
                    open_set.append(neighbor)

    return None


def turnLeft(wheels):
    print("turning left")
    wheels[0].setVelocity(2.0)
    wheels[1].setVelocity(-2.0)
    wheels[2].setVelocity(2.0)
    wheels[3].setVelocity(-2.0)


def turnRight(wheels):
    print("turning right")
    wheels[0].setVelocity(-2.0)
    wheels[1].setVelocity(2.0)
    wheels[2].setVelocity(-2.0)
    wheels[3].setVelocity(2.0)


def moveForward(wheels):
    print("moving forward")
    wheels[0].setVelocity(2.0)
    wheels[1].setVelocity(2.0)
    wheels[2].setVelocity(2.0)
    wheels[3].setVelocity(2.0)


def stop(wheels):
    print("stop")
    wheels[0].setVelocity(0.0)
    wheels[1].setVelocity(0.0)
    wheels[2].setVelocity(0.0)
    wheels[3].setVelocity(0.0)


def control_robot_along_path(robot_location, orientation_unit_vector, path, next_waypoint):
    if len(path) < 2:
        return

    next_point = next_waypoint

    next_point_world = (
        (next_point[0] * CELL_SIZE),
        (next_point[1] * CELL_SIZE),
    )

    target_orientation_unit_vector = getTargetOrientationVector(next_point_world, robot_location)
    # print("Target orientation unit vector:", target_orientation_unit_vector)

    angle_to_next_point = math.atan2(
        next_point_world[0] - robot_location[0], next_point_world[1] - robot_location[1]
    ) * (180 / math.pi)

    print("angle to next pt: ", end='')
    print(angle_to_next_point)

    # angle_diff = angle_to_next_point - math.atan2(orientation_unit_vector[1], orientation_unit_vector[0]) * (180 / math.pi)
    # print("angle diff: ", end='')
    # print(angle_diff)

    angle_diff = math.acos(np.dot(target_orientation_unit_vector, orientation_unit_vector)) * (180 / math.pi)
    if math.asin(np.cross(target_orientation_unit_vector, orientation_unit_vector)) < 0:
        delta_theta *= -1
    print("angle diff: ", end='')
    print(angle_diff)
    

    if abs(angle_diff) > 0.1:
        if angle_diff > 0:
            turnRight(wheels)
        else:
            turnLeft(wheels)
    else:
        moveForward(wheels)

    robot.step(TIME_STEP)

    distance_to_next_point = math.sqrt(
        (next_point_world[0] - robot_location[0]) ** 2 + (next_point_world[1] - robot_location[1]) ** 2)
    if distance_to_next_point < CELL_SIZE * 0.5:
        stop(wheels)


# Convert the goal coordinates (3, -4) to array indices
goal = [3, -4]
# goal_index = (goal_y + occupancy_grid.shape[0] // 2, goal_x + occupancy_grid.shape[1] // 2)
goal_index = (goal[1] / CELL_SIZE * -1, goal[0] / CELL_SIZE)

path_found = False
waypoint_num = 0
next_waypoint = []

# Find the path using the A* algorithm
while robot.step(TIME_STEP) != -1:
    gps_values = getGpsData(gps, gps_values)
    robot_location, orientation_unit_vector = getRobotPose(gps_values)
    target_orientation_unit_vector = getTargetOrientationVector(
        goal, robot_location)

    print("robot location: ", end='')
    print(robot_location)

    robot_location_index = [
        int(robot_location[1] / CELL_SIZE * -1), int(robot_location[0] / CELL_SIZE)]
    # print("robot loc index: ", end='')
    # print(robot_location_index)

    if path_found == False:
        path = a_star(robot_location_index, goal_index, occupancy_grid)
        if path:
            path_found = True
            waypoint_num = waypoint_num + 1
            next_waypoint = path[waypoint_num]
            print("Path found:", path)
            print("next waypoint: ", end='')
            print(next_waypoint)
        else:
            print("No path found")
            stop(wheels)

    print("x error: " + str(abs(next_waypoint[0] - robot_location_index[0])) +
          ", y error: " + str(abs(next_waypoint[1] - robot_location_index[1])))
    if abs(next_waypoint[0] - robot_location_index[0]) < 3 and abs(next_waypoint[1] - robot_location_index[1]) < 3:
        print("waypoint reached")
        waypoint_num = waypoint_num + 1
        next_waypoint = path[waypoint_num]

    control_robot_along_path(
        robot_location, orientation_unit_vector, path, next_waypoint)

    robot.step(TIME_STEP)
