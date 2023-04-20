from controller import Robot
import math
import csv
import numpy as np

# Read the occupancy grid from a CSV file and convert it to a numpy array
filename = '../../OccupancyGrid.csv'
delimiter = ','
occupancy_grid = np.genfromtxt(filename, delimiter=delimiter, dtype=str)
occupancy_grid[occupancy_grid == ''] = '0'
occupancy_grid = np.char.strip(occupancy_grid, "'")
print(occupancy_grid)

# Define cell size and time step, and initialise robot's GPS and wheels
CELL_SIZE = 0.1
TIME_STEP = 64
robot = Robot()


def initGps(TIME_STEP, robot):
    # Get robot's GPS and enable
    gps1 = robot.getDevice('gps1')
    gps1.enable(TIME_STEP)
    gps2 = robot.getDevice('gps2')
    gps2.enable(TIME_STEP)

    gps = [gps1, gps2]
    value = [[0, 0, 0], [0, 0, 0]]

    return gps, value



def initWheels(robot):
     # Get robot's wheel and set position and velocity
    wheel1 = robot.getDevice('wheel1') # Front Right Wheel
    wheel1.setPosition(float('inf'))
    wheel1.setVelocity(2.0)

    wheel2 = robot.getDevice('wheel2') # Front Left Wheel
    wheel2.setPosition(float('inf'))
    wheel2.setVelocity(2.0)

    wheel3 = robot.getDevice('wheel3') # Back Right Wheel
    wheel3.setPosition(float('inf'))
    wheel3.setVelocity(2.0)

    wheel4 = robot.getDevice('wheel4') # Back Left Wheel
    wheel4.setPosition(float('inf'))
    wheel4.setVelocity(2.0)

    wheels = [wheel1, wheel2, wheel3, wheel4]

    return wheels

# Initialise the distance sensor
ds = []
dsNames = ['ds_front_right', 'ds_front_left']

for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

avoidObstacleCounter = 0

# Initialise GPS and Wheels
gps, gps_values = initGps(TIME_STEP, robot)
wheels = initWheels(robot)

# Function to get robot's GPS data
def getGpsData(gps, value):
    value[0] = gps[0].getValues()
    value[1] = gps[1].getValues()
    # print("GPS 1:", value[0])
    # print("GPS 2:", value[1])
    return value

# Function to get robot's pose
def getRobotPose(gps_value):
    robot_location = [(gps_value[0][0] + gps_value[1][0]) / 2,
                      (gps_value[0][1] + gps_value[1][1]) / 2]
    orientation_vector = [gps_value[0][1] - gps_value[1][1], -1 * (gps_value[0][0] - gps_value[1][0])]
    mag_orientation_vector = math.sqrt(pow(orientation_vector[0], 2) + pow(orientation_vector[1], 2))
    orientation_unit_vector = [0, 0]

    if mag_orientation_vector != 0:
        orientation_unit_vector[0] = orientation_vector[0] / mag_orientation_vector
        orientation_unit_vector[1] = orientation_vector[1] / mag_orientation_vector

    return robot_location, orientation_unit_vector

# Function to get the target orientation vector
def getTargetOrientationVector(goal, robot_location):
    target_orientation_vector = [goal[0]-robot_location[0], goal[1]-robot_location[1]]
    mag_target_orientation_vector = math.sqrt (pow(target_orientation_vector[0],2)+pow(target_orientation_vector[1],2))
    target_orientation_unit_vector = [0, 0]
    
    if mag_target_orientation_vector != 0:
        target_orientation_unit_vector[0] = target_orientation_vector[0]/mag_target_orientation_vector
        target_orientation_unit_vector[1] = target_orientation_vector[1]/mag_target_orientation_vector
    
    return target_orientation_unit_vector

# Function for heuristic cost estimate function to calculate the estimated cost to reach the goal from the current position
def heuristic_cost_estimate(start, goal):
    return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)

# Function to check if the given point is within the bounds of the grid
def is_within_bounds(point, grid):
    return 0 <= point[0] < grid.shape[0] and 0 <= point[1] < grid.shape[1]

# Function to get the neighbours of a point in the grid
def get_neighbours(point, grid):
    neighbours = []

    # Iterate through all possible neighbouring points
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        new_point = (point[0] + dx, point[1] + dy)
        
        # Check if the new point is within bounds and not an obstacle
        if is_within_bounds(new_point, grid) and grid[new_point] == '0':
            neighbours.append(new_point)

    return neighbours

# A* algorithm to find the path from start to goal
def a_star(start, goal, grid):
    start = tuple(start)
    goal = tuple(goal)

    open_set = [start]
    came_from = {}

    g_score = {point: float("inf") for point in np.ndindex(grid.shape)}
    g_score[start] = 0

    f_score = {point: float("inf") for point in np.ndindex(grid.shape)}
    f_score[start] = heuristic_cost_estimate(start, goal)

    # Continue until all nodes are evaluated
    while open_set:
        # Select node with lowest f score
        current = min(open_set, key=lambda point: f_score[point])
        
        # If goal reached, return path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

         # Remove current node from open set
        open_set.remove(current)

        # Evaluate current node neighbours
        for neighbour in get_neighbours(current, grid):
            tentative_g_score = g_score[current] + 1

             # If tentative g score less than current neighbour g score then update its g score & f score
            if tentative_g_score < g_score[neighbour]:
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g_score
                f_score[neighbour] = g_score[neighbour] + heuristic_cost_estimate(neighbour, goal)
                if neighbour not in open_set:
                    open_set.append(neighbour)

    # If no path then return None
    return None


def turnLeft(wheels):
    print("turning left")
    wheels[0].setVelocity(1.0)
    wheels[1].setVelocity(0.0)
    wheels[2].setVelocity(1.0)
    wheels[3].setVelocity(0.0)


def turnRight(wheels):
    print("turning right")
    wheels[0].setVelocity(-1.0)
    wheels[1].setVelocity(0.0)
    wheels[2].setVelocity(-1.0)
    wheels[3].setVelocity(0.0)


def moveForward(wheels):
    print("moving forward")
    wheels[0].setVelocity(5.0)
    wheels[1].setVelocity(5.0)
    wheels[2].setVelocity(5.0)
    wheels[3].setVelocity(5.0)

def moveBackward(wheels):
    print("moving backward")
    wheels[0].setVelocity(-2.0)
    wheels[1].setVelocity(-2.0)
    wheels[2].setVelocity(-2.0)
    wheels[3].setVelocity(-2.0)


def stop(wheels):
    print("stop")
    wheels[0].setVelocity(0.0)
    wheels[1].setVelocity(0.0)
    wheels[2].setVelocity(0.0)
    wheels[3].setVelocity(0.0)


# Function to move the robot to the next waypoint
def moveToGoal(path, angle_diff):
    if len(path) < 2:
        return

    # Limit angle_diff to be between -180 and 180 degrees
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360

     # If angle difference greater than 2º then turn left or right depending on angle difference sign +/-
    if abs(angle_diff) > 2:
        if angle_diff > 0:
            turnRight(wheels)
        else:
            turnLeft(wheels)
    else:
        moveForward(wheels)

    robot.step(TIME_STEP)

    
# Function to avoid obstacles using distance sensors
def avoid_obstacle(ds):
    right_obstacle = ds[0].getValue() < 1000.0
    left_obstacle = ds[1].getValue() < 1000.0

    if left_obstacle and right_obstacle:
        return "backward"
    elif left_obstacle:
        return "right"
    elif right_obstacle:
        return "left"
    else:
        return "none"
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Add a distance threshold value
DISTANCE_THRESHOLD = CELL_SIZE * 0.5

# Goal and its index in the occupancy grid
goal = [3, -4]
goal_index = (goal[1] / CELL_SIZE * -1, goal[0] / CELL_SIZE)

# Variables initialisation
path_found = False
waypoint_num = 0
next_waypoint = []
state = "path_following" # Allow switiching between obstacle_avoidance (distance sensor) and path_following (A* path)

# Main loop - Find the path using the A* algorithm
while robot.step(TIME_STEP) != -1:
    # Get the GPS data and robot's pose
    gps_values = getGpsData(gps, gps_values)

    robot_location, orientation_unit_vector = getRobotPose(gps_values)
    robot_pose = getRobotPose(gps_values)

    # Print robot location and orientation for debugging
    print("robot location: ", end='')
    print(robot_location)
    print("orientation unit vect: ", end='')
    print(orientation_unit_vector)

    # Convert robot location to an index in the occupancy grid
    robot_location_index = [
        int(robot_location[1] / CELL_SIZE * -1), int(robot_location[0] / CELL_SIZE)]
    print("robot loc index: ", end='')
    print(robot_location_index)

    # If path not found use A* algorithm to find it
    if path_found == False:
        path = a_star(robot_location_index, goal_index, occupancy_grid)
        if path:
            path_found = True
            waypoint_num = waypoint_num + 1
            next_waypoint = path[waypoint_num]
            print("Path found:", path)
            print("next waypoint: ", end='')
            print(next_waypoint)

            # Convert the next waypoint to world coordinates
            next_waypoint_world = (
            (next_waypoint[0] * CELL_SIZE),
            (next_waypoint[1] * CELL_SIZE),
        )
        else:
            print("No path found")
            stop(wheels)

    

    # Check if the distance between the robot's location and the next waypoint is less than or equal to the threshold
    if path_found and distance(robot_location, next_waypoint_world) <= DISTANCE_THRESHOLD:
        waypoint_num = waypoint_num + 1
        next_waypoint = path[waypoint_num]
        print("next waypoint: ", end='')
        print(next_waypoint)
        print("next_waypoint_world: ", end='')
        print(next_waypoint_world)
        next_waypoint_world = (
        (next_waypoint[0] * CELL_SIZE),
        (next_waypoint[1] * CELL_SIZE),
    )

    # Calculate the target orientation and angle differences
    target_orientation_unit_vector = getTargetOrientationVector(next_waypoint_world, robot_location)
    print("Target orientation unit vector:", target_orientation_unit_vector)
    print("orientation unit vector: ", orientation_unit_vector)

    angle_to_next_waypoint = math.atan2(next_waypoint_world[0] - robot_location[0], next_waypoint_world[1] - robot_location[1]) * (180 / math.pi)
    print("angle to next pt: ", end='')
    print(angle_to_next_waypoint)

    angle_diff = angle_to_next_waypoint - math.atan2(orientation_unit_vector[0], orientation_unit_vector[1]) * (180 / math.pi)
    print("angle diff: ", end='')
    print(angle_diff)
    
    # Stop the robot if it is close enough to the next waypoint
    distance_to_next_waypoint = math.sqrt(
        (next_waypoint_world[0] - robot_location[0]) ** 2 + (next_waypoint_world[1] - robot_location[1]) ** 2)
    if distance_to_next_waypoint < CELL_SIZE * 0.5:
        stop(wheels)

    # Check if obstacle avoidance is needed
    obstacle_avoidance_direction = avoid_obstacle(ds)

    if obstacle_avoidance_direction != "none":
        state = "obstacle_avoidance"
    elif state == "obstacle_avoidance":
        state = "path_following"

    if state == "obstacle_avoidance":
        if obstacle_avoidance_direction == "left":
            turnLeft(wheels)
        elif obstacle_avoidance_direction == "right":
            turnRight(wheels)
        elif obstacle_avoidance_direction == "backward":
            moveBackward(wheels)
    elif state == "path_following":
        moveToGoal(path, angle_diff)
    
    # Below is when the above obstacle avoidance was not implemented
    # moveToGoal(path, angle_diff)
        
    robot.step(TIME_STEP)
