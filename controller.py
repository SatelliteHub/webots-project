from controller import Robot
import math
import numpy as np
import csv

# A* related functions and classes
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def neighbors(self, occupancy_grid):
        neighbors = []

        for dx, dy in [[1, 0], [0, 1], [-1, 0], [0, -1]]:
            new_x, new_y = self.x + dx, self.y + dy

            if 0 <= new_x < occupancy_grid.shape[1] and 0 <= new_y < occupancy_grid.shape[0]:
                if occupancy_grid[new_y, new_x] == '0':
                    neighbors.append(Node(new_x, new_y))

        return neighbors

    def distance_to(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"Node({self.x}, {self.y})"


def heuristic(node, goal):
    return node.distance_to(goal)


def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.insert(0, current)
    return total_path


def a_star(start, goal, occupancy_grid):
    # Convert start and goal from meters to grid cells
    start = (int(start[0] * 10), int(start[1] * 10))
    goal = (int(goal[0] * 10), int(goal[1] * 10))

    start_node = Node(start[0], start[1])  # Add this line
    goal_node = Node(goal[0], goal[1])     # Add this line

    open_set = {start_node}
    came_from = {}

    g_score = {node: float('inf') for node in open_set}
    g_score[start_node] = 0

    f_score = {node: float('inf') for node in open_set}
    f_score[start_node] = heuristic(start_node, goal_node)

    while open_set:
        current = min(open_set, key=lambda node: f_score[node])
        if current == goal_node:
            return reconstruct_path(came_from, current)

        open_set.remove(current)
        for neighbor in current.neighbors(occupancy_grid):
            tentative_g_score = g_score[current] + current.distance_to(neighbor)
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_node)
                if neighbor not in open_set:
                    open_set.add(neighbor)

    return None  # Open set is empty but goal was never reached

def is_occupied(occupancy_grid, x, y):
    return occupancy_grid[y][x] == '1'








# Robot related code
TIME_STEP = 64
robot = Robot()
ds = []
dsNames = ['ds_right', 'ds_left']
goal = [3, -4]

filename = 'OccupancyGrid.csv'
delimiter = ','
occupancy_grid = np.genfromtxt(filename, delimiter=delimiter, dtype=str)
occupancy_grid[occupancy_grid == ''] = '0'
occupancy_grid = np.char.strip(occupancy_grid, "'")
print(occupancy_grid)

for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

avoidObstacleCounter = 0

def getGpsData():
    gp = robot.getDevice("global")
    gp.enable(TIME_STEP)
    gpsData = gp.getValues()
    print("X:", gpsData[0])
    print("Y:", gpsData[1])
    print("Z:", gpsData[2])
    return gpsData

x = 0.0
y = 0.0
theta = 0.0

def getRobotPose(gps_value, leftSpeed, rightSpeed, theta):
    robot_location = [gps_value, 0]
    orientation_vector = [leftSpeed - rightSpeed, -1*(math.sin(theta)*(leftSpeed + rightSpeed))]
    mag_orientation_vector = math.sqrt(pow(orientation_vector[0], 2) + pow(orientation_vector[1], 2))
    orientation_unit_vector = [0, 0]

    if mag_orientation_vector != 0:
        orientation_unit_vector[0] = orientation_vector[0] / mag_orientation_vector
        orientation_unit_vector[1] = orientation_vector[1] / mag_orientation_vector

    robot_location[0] += orientation_unit_vector[0] * ((leftSpeed + rightSpeed) / 2.0 * TIME_STEP / 1000.0)
    robot_location[1] += orientation_unit_vector[1] * ((leftSpeed + rightSpeed) / 2.0 * TIME_STEP / 1000.0)

    return robot_location, orientation_unit_vector

def getTargetPose(goal, robot_location):
    target_orientation_vector = [goal[0]-robot_location[0], goal[1]-robot_location[1]]
    mag_target_orientation_vector = math.sqrt(pow(target_orientation_vector[0], 2) + pow(target_orientation_vector[1], 2))
    target_orientation_unit_vector = [0, 0]

    if mag_target_orientation_vector != 0:
        target_orientation_unit_vector[0] = target_orientation_vector[0] / mag_target_orientation_vector
        target_orientation_unit_vector[1] = target_orientation_vector[1] / mag_target_orientation_vector

    return target_orientation_unit_vector
    
# Find the path using A* algorithm
current_waypoint_index = 0
gpsData = getGpsData()  # Initialize gpsData
start = (0.0, 0.0)  # Hard-code the actual starting coordinates here
path = a_star(start, goal, occupancy_grid)

print("Path:", path)


while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:  # read sensors
        for i in range(2):
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 10

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

    gpsData = getGpsData()
    robot_location, orientation_unit_vector = getRobotPose(gpsData[0], leftSpeed, rightSpeed, theta)
    target_orientation_unit_vector = getTargetPose([3.0, -4.0], robot_location)
    
    delta_distance = (leftSpeed + rightSpeed) / 2.0 * TIME_STEP / 1000.0
    delta_theta = (leftSpeed - rightSpeed) / 0.102  # 0.102 is the distance between the wheels
    x += delta_distance * orientation_unit_vector[0]
    y += delta_distance * orientation_unit_vector[1]
    theta += delta_theta
    print("Robot Location:", robot_location)
    print("Orientation Unit Vector:", orientation_unit_vector)
    print("Target Orientation Unit Vector:", target_orientation_unit_vector)
    print("x:", x)
    print("y:", y)
    print("theta:", theta)


# New function to control the robot based on the next waypoint in the path
def control_robot_to_waypoint(waypoint, robot_location, orientation_unit_vector):
    target_orientation_unit_vector = getTargetPose(waypoint, robot_location)
    dot_product = orientation_unit_vector[0] * target_orientation_unit_vector[0] + orientation_unit_vector[1] * target_orientation_unit_vector[1]
    angle_error = math.acos(dot_product)

    # Simple proportional controller
    kP = 5
    leftSpeed = 1.0 - kP * angle_error
    rightSpeed = 1.0 + kP * angle_error

    return leftSpeed, rightSpeed

# Main loop
current_waypoint_index = 0
start = (gpsData[0], gpsData[2])  # Use the initial GPS data as the starting point
path = A_Star(start, goal, h, occupancy_grid)
print("Path:", path)

while robot.step(TIME_STEP) != -1:
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:  # read sensors
        for i in range(2):
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 10

        if current_waypoint_index < len(path):
            waypoint = [path[current_waypoint_index][0] / 10.0, path[current_waypoint_index][1] / 10.0]  # Convert waypoint from grid cells to meters
            robot_location, orientation_unit_vector = getRobotPose(gpsData[0], leftSpeed, rightSpeed, theta)
            leftSpeed, rightSpeed = control_robot_to_waypoint(waypoint, robot_location, orientation_unit_vector)

            # Check if the robot reached the current waypoint
            distance_to_waypoint = math.sqrt((robot_location[0] - waypoint[0]) ** 2 + (robot_location[1] - waypoint[1]) ** 2)
            if distance_to_waypoint < 0.2:
                current_waypoint_index += 1
        else:
            leftSpeed = 0.0
            rightSpeed = 0.0

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

    gpsData = getGpsData()
    robot_location, orientation_unit_vector = getRobotPose(gpsData[0], leftSpeed, rightSpeed, theta)

    delta_distance = (leftSpeed + rightSpeed) / 2.0 * TIME_STEP / 1000.0
    delta_theta = (leftSpeed - rightSpeed) / 0.102  # 0.102 is the distance between the wheels
    x += delta_distance * orientation_unit_vector[0]
    y += delta_distance * orientation_unit_vector[1]
    theta += delta_theta
    print("Robot Location:", robot_location)
    print("Orientation Unit Vector:", orientation_unit_vector)
