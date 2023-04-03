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
    
    x = 0.0
    y = 0.0
    theta = 0.0
    
    def getRobotPose():
        global x, y, theta
        gpsData = getGpsData()
        
        # ##Convert GPS coordinates to meters
        x_gps = gpsData[0]
        y_gps = gpsData[2]
        
        #Update the robot's position based on its movement
        delta_distance = (leftSpeed + rightSpeed) / 2.0 * TIME_STEP / 1000.0
        delta_theta = (leftSpeed - rightSpeed) / 0.102  # 0.102 is the distance between the wheels
        x += delta_distance * math.sin(theta + delta_theta / 2.0)
        y += delta_distance * math.cos(theta + delta_theta / 2.0)
        theta += delta_theta
        
        return (x, y, theta)
        
    robot_location = getRobotPose()















