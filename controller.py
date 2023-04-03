from controller import Robot
import math

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
                avoidObstacleCounter = 100

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
    print("x:", x)
    print("y:", y)
    print("theta:", theta)
