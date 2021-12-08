#  from controller import Robot, Motor, DistanceSensor
from shutil import move
from typing import final
from numpy.lib import angle
from numpy.lib.npyio import save
from numpy.lib.utils import safe_eval
from controller import Robot
import math
import numpy as np
from numpy.core.defchararray import equal

# create the Robot instance.
robot = Robot()

# define robot details
turtlebot3 = {
    "wheelDia": 66,
    "wheelBase": 160,
    "outerWheelBase": 178,
    "clearance": 0.09
}

# get the time step of the current world.
timestep = 64
base_speed = 1.5

# wayPoints = [ [0.0,0.0], [0, 0.1], [0.1, 0.1], [0.1, 0.0], [0.2,0.0], [0.2,0.1], [0.3,0.1], [0.3,0.0]]
wayPoints = [ [0.0,0.0], [0, 1.0],[0.5,0.5],[1.0,0.5]]

# returns the euclidian distance between the robot and the next waypoint
def getTargetDistance(robot_pos, coord):
    return math.sqrt((robot_pos[0]-coord[0])**2 + (robot_pos[1]-coord[1])**2)

# returns the angle of rotation directed to the next waypoint wrt the robot
def getTargetAngle(robot_pos, coord):
    theta = math.atan2( coord[1]-robot_pos[1], coord[0]-robot_pos[0])
    return theta 

# for getting angles wrt robot
def getAngleRobot(robot_pose, way_point):
    # print('Get angle Robot', robot_pose, way_point)
    quadrant = 0
    angle = math.atan2(way_point[1] - robot_pose[1], way_point[0] - robot_pose[0])
    
    # axis values: 1 +x, 2 -x, 3 +y,4 -y 
    axis = 0
    if robot_pose[0] < way_point[0] and robot_pose[1] < way_point[1]:
        quadrant = 1
    elif robot_pose[0] > way_point[0] and robot_pose[1] < way_point[1]:
        quadrant = 2
    elif robot_pose[0] > way_point[0] and robot_pose[1] > way_point[1]:
        quadrant = 3
        angle = -angle + 1.5708 
    elif robot_pose[0] < way_point[0] and robot_pose[1] > way_point[1]:
        quadrant = 4
    elif robot_pose[0] < way_point[0] and robot_pose[1] == way_point[1]:
        axis = 1
    elif robot_pose[0] > way_point[0] and robot_pose[1] == way_point[1]:
        axis = 2
    elif robot_pose[0] == way_point[0] and robot_pose[1] < way_point[1]:
        axis = 3
    elif robot_pose[0] == way_point[0] and robot_pose[1] > way_point[1]:
        axis = 4
    # print('q:', quadrant, 'a:', axis)
    # print('---Angle---', angle)
    return angle

# initialization of motors and returning left and right motor controllers
def initializeMotors():
    right_motor = robot.getDevice("right wheel motor")
    left_motor = robot.getDevice("left wheel motor")
    right_motor.setPosition(float('Inf'))
    left_motor.setPosition(float('Inf'))
    right_motor.setVelocity(0.0)
    left_motor.setVelocity(0.0)

    return [left_motor, right_motor]

# initialization of position sensors and returning left and right sensor controllers
def initializePositionSensors():
    left_ps = robot.getDevice('left wheel sensor')
    left_ps.enable(timestep)
    
    right_ps = robot.getDevice('right wheel sensor')
    right_ps.enable(timestep)

    return [left_ps, right_ps]

# initialization of lidar and returning lidar controllers
def initializeLidar():
    # lidar = robot.getDevice("LDS-01")
    # lidar.enable(timestep)
    # lidar.enablePointCloud()

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()

    lidar_main_motor = robot.getDevice("LDS-01_main_motor")
    lidar_secondary_motor = robot.getDevice("LDS-01_secondary_motor")
    lidar_main_motor.setPosition(float('Inf'))
    lidar_secondary_motor.setPosition(float('Inf'))
    lidar_main_motor.setVelocity(30.0)
    lidar_secondary_motor.setVelocity(60.0)

    return [lidar, lidar_main_motor, lidar_secondary_motor]

# initialization of accelerometer and returning accelerometer controllers
def initializeAccelerometer():
    acc = robot.getDevice('accelerometer')
    acc.enable(timestep)
    return acc

# initialization of gyroscope  and returning gyroscope  controllers
def initializeGyro():
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)
    return gyro

# initialization of GPS and returning GPS controllers
def initializeGps():
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    return gps

# getting accelerometer values
def getAccValues(acc):
    return acc.getValues()

# getting GPS values
def getGpsValues(gps, start_gps):
    gps_world = gps.getValues()
    # return [gps_world[0]-start_gps[0], -gps_world[2]+start_gps[2], gps_world[1]-start_gps[1]]
    return [-gps_world[2]+start_gps[0], -gps_world[0]-start_gps[1]]

# getting gyroscope values
def getGyroValues(gyro):
    return gyro.getValues()

# function to move robot forward
def moveRobot(motors, vel):
    print("Robot Moving")
    motors[0].setVelocity(vel)
    motors[1].setVelocity(vel) 

# function to rotate robot left or right
def rotateRobot(motors, vel, l_r):
    print("Robot Rotating",l_r)
    motors[0].setVelocity( l_r * vel)
    motors[1].setVelocity( -l_r * vel) 

# move in a curve:
def curveRobot(motors, vel, l_r):
    print('Curve')
    outer_wheel = turtlebot3['outerWheelBase']/2000 + turtlebot3['clearance']
    inner_wheel = turtlebot3['clearance'] - turtlebot3['outerWheelBase']/2000 + 0.01
    if (l_r == -1):
        print('Turning right')
        motors[0].setVelocity( vel )
        motors[1].setVelocity( vel * inner_wheel / outer_wheel)
    elif (l_r == 1):
        print('Turning Left')
        motors[1].setVelocity( vel )
        motors[0].setVelocity( vel * inner_wheel / outer_wheel)
# funtion to stop robot
def stopRobot(motors):
    print("Robot Stopped")
    motors[0].setVelocity(0)
    motors[1].setVelocity(0) 

# stopping 360 lidar motors
def stopLidar(lidar_main_motor, lidar_secondary_motor):
    lidar_main_motor.setVelocity(0)
    lidar_secondary_motor.setVelocity(0)

# function which returns position sensor values
def getPositionSensor(ps):
    return np.array([ps[0].getValue(), ps[1].getValue()]) * turtlebot3['wheelDia']/2000

# function which uses robot pose, position sensor datas to calculate latest robot pose
def getRobotPose(robot_pose, curr_ps, prev_ps):

    dist_values = curr_ps - prev_ps
    for ind in range(2):
        if abs(dist_values[ind]) < 0.0001:
            dist_values[ind] = 0
            curr_ps[ind] = prev_ps[ind]
                
    v = (dist_values[0] + dist_values[1]) / 2.0
    w = (dist_values[0] - dist_values[1]) / ( 2.0 * (turtlebot3['wheelBase']/2000))

    w_corr = w * (turtlebot3['wheelBase'] * math.pi / 1000) / 0.55737198
    dt = 1
    robot_pose[2] -= (w_corr * dt)
    vx = v * math.cos(robot_pose[2])
    vz = v * math.sin(robot_pose[2])

    robot_pose[0] += (vx * dt)
    robot_pose[1] += (vz * dt)

    return robot_pose

# function to get lidar values
def getLidar(lidar):
    range_image = np.array(lidar.getRangeImage())
    # return np.array([range_image[180:360],range_image[0:180]])
    # print('lS', range_image)
    # return np.concatenate([lidarSplit[1],lidarSplit[0]])
    return range_image

# function to smoothen the velicity of the robot
def getVelocity(target_v, prev_v):
     return target_v*0.05 + prev_v*0.95

# function to calculate the x or y coordinate of desired corner
def getWallDistance(lidar_points, wall_points, coord):
    x_y = []
    for i in range(0,len(wall_points)):
        dist = lidar_points[wall_points[i]]
        ang = wall_points[i] 
        if ang < 90:
            if (coord == 'x'):
                x_y.append(-(dist)*math.cos((ang)*1.57/90))
            if (coord == 'y'):
                x_y.append((dist)*math.sin((ang)*1.57/90))
        else:
            if (coord == 'x'):
                x_y.append((dist)*math.cos((180-ang)*1.57/90))
            if (coord == 'y'):
                x_y.append((dist)*math.sin((180-ang)*1.57/90))
    return x_y

# function which returns the waypoint from hitting the wall to circumnavigate the wall
def getWallWaypoint(entry_point, wallCoords,nextWayPoint, turnAngle):
    # print('Checking Waypoint', entry_point, wallCoords,nextWayPoint)

    wall_x = np.array(wallCoords[0][0])
    wall_y = np.array(wallCoords[1][0])
    # print('wallxy',wall_x,wall_y)

    wall_waypoint_dist = [getTargetDistance([wall_x[0], wall_y[0]],[nextWayPoint[0], nextWayPoint[1]]), getTargetDistance([wall_x[1], wall_y[1]],[nextWayPoint[0], nextWayPoint[1]])]
    # print('wall dist', wall_waypoint_dist)

    clearance_dist = 0.08
    # wall_waypoint = [wall_x[np.argmin(wall_waypoint_dist)] , wall_y[np.argmin(wall_waypoint_dist)]]
    if wall_x[np.argmin(wall_waypoint_dist)] > entry_point[0]:
        c_x = 1
    else:
        c_x = -1
    
    if wall_y[np.argmin(wall_waypoint_dist)] > entry_point[1]:
        c_y = -1
    else:
        c_y = 1
    # wall_waypoint = [wall_x[np.argmin(wall_waypoint_dist)] + c_x * clearance_dist * math.cos(1.5708-turnAngle) , wall_y[np.argmin(wall_waypoint_dist)] + c_y * clearance_dist * math.sin(1.5708-turnAngle)]

    wall_waypoint = [wall_x[np.argmin(wall_waypoint_dist)] + c_x * clearance_dist * math.cos(1.5708-turnAngle) , wall_y[np.argmin(wall_waypoint_dist)] + c_y * clearance_dist * math.sin(1.5708-turnAngle)]
    # print('Wall waypoint:', wall_waypoint)

    return wall_waypoint

# function to check if the robot has reached the next waypoint or not
def checkWaypoint(robot_pos, waypoint):
    # print('RP', robot_pose,'WP', waypoint,'W', wall)
    x_check = abs(robot_pos[0] - waypoint[0]) < 0.01   
    y_check = abs(robot_pos[1] - waypoint[1]) < 0.01 
    
    if x_check and y_check:
        return True
    else:
        return False

# function to check if robot has reached desired angle
def checkAngularWaypoint(robot_pose, theta, reached):
    print('pose:', robot_pose, 'thate', theta, 'reach', reached)

    ang_check = abs(robot_pose[2] - theta) < 0.02

    if ang_check:
        # print('Reached Angular Waypoint')
        return [True, 0]
    else:
        if robot_pose[2] > 0.99 * theta:
            # need to rotate right
            return [False, 1]
        elif robot_pose[2] < 1.01 * theta:
            return [False, -1]

# function to check if robot has reached desired angle
def checkEdgeAngle(robot_pose, theta):
    print('pose:', robot_pose, 'thate', theta)

    ang_check = abs(robot_pose[2] - theta) < 0.03
    if ang_check:
        # print('Reached Angular Waypoint')
        return [True]
    else:
        return [False]

# function to get wall index fron lidar data
def getWallIndex(lidar_point):
    lidar_wall = [not math.isinf(lidar_point[i]) for i in range(0,len(lidar_point))]
    # print('Lidar Map', lidar_wall)

    wall_points = []
    wall_type = []
    initVal = lidar_wall[0]
    prevVal = 0

    for i in range(0,len(lidar_wall)):
        if(lidar_wall[i] is not initVal):
            # print('Wall CHange')
            if(initVal is True and lidar_wall[i] is False):
                wall_type.append(True)
                wall_points.append([prevVal, i-1])
            prevVal = i
            initVal = lidar_wall[i]

    #print('After all', prevVal,len(lidar_wall)-1,initVal)
    if initVal == True:
        wall_type.append(initVal)
        wall_points.append([prevVal, len(lidar_wall)-1])
    # print('wall point', wall_points)
    return wall_points

# function to get the obstacle and clearance regions
def obstacleDetection(lidar_point, max_dist):
    x_wall_coord = []
    y_wall_coord = []
    angle_edge = []
    edge_obj = []
    if (min(lidar_point[75:105]) <= max_dist):
        # print('Obstacle Ahead')
        wall_points = getWallIndex(lidar_point)
        [x_coord,y_coord,angle,edge_index] = getLidarObstacleCoordinate(lidar_point)
        for i in range(0, len(wall_points)):
            wall = wall_points[i]
            x_wall_coord.append([x_coord[edge_index.index(wall[0])], x_coord[edge_index.index(wall[1])]])
            y_wall_coord.append([y_coord[edge_index.index(wall[0])], y_coord[edge_index.index(wall[1])]])
            angle_edge.append([angle[edge_index.index(wall[0])], angle[edge_index.index(wall[1])]])
            edge_obj.append({
                "left-corner": [x_coord[edge_index.index(wall[0])], y_coord[edge_index.index(wall[1])]],
                "right-corner": [x_coord[edge_index.index(wall[1])], y_coord[edge_index.index(wall[0])]],
                "edge-angles": [angle[edge_index.index(wall[0])], angle[edge_index.index(wall[1])]]
            })
        # print('Wall coord:', x_wall_coord, y_wall_coord, angle_edge)
        # print('Edge: ', edge_obj)
        # return [True, x_wall_coord, y_wall_coord, angle_edge, edge_obj]
        return [True, edge_obj]
    else: 
        return [False]

# function to chose the best corner to turn to
def getBestCorner(robot_pose, edge_1, edge_2, prim_waypoint):
    # print('Best corner', robot_pose, edge_1, edge_2, prim_waypoint)
    dist_1 = getTargetDistance(robot_pose, edge_1) + getTargetDistance(edge_1, prim_waypoint)
    dist_2 = getTargetDistance(robot_pose, edge_2) + getTargetDistance(edge_2, prim_waypoint)
    
    edge_M = []
    edge_m = []
    if (dist_1 == dist_2):
        # print('Selected Edge: ', edge_2)
        edge_M = edge_2
        edge_m = edge_1
    elif (dist_1 < dist_2):
        # print('Selected Edge:', edge_1)
        edge_M = edge_1
        edge_m = edge_2
    else:
        # print('Same distance', edge_2)
        edge_M = edge_2
        edge_m = edge_1
    
    theta = getTargetAngle([edge_m[0], edge_m[1], 0], edge_M)
    clearance_distance = 0.09
    # print('Wall Theta:', theta)
    final_edge = [edge_M[0] + clearance_distance * math.cos(theta) + robot_pose[0], edge_M[1] + clearance_distance * math.sin(theta) + robot_pose[1]]
    # print('Final edge:', final_edge)

    return [final_edge, theta]

# function which returns the rounded list 
def getRoundList(listData, decimal):
    roundedData = [round(listElement, decimal) for listElement in listData]
    return roundedData

# function to get range fro one value to another
def getRange(start, end, steps):
    increment = (end - start)/steps
    range_value = []
    for i in range(0,steps):
        range_value.append(start + i * increment)
    
    return range_value

# get lidar positions directly from the lidar data wrt the robot
def getLidarObstacleCoordinate(lidar_data):
    x_coord = []
    y_coord = []
    obj_ang = []
    obj_ind = []
    # in the lidar the end points are at 0 and 180 degree
    # so we have made the lidar to have resolution of 181 
    # this makes the index and angle to be same value
    for i in range(0,90):
        angle = 90 - i
        if (not math.isinf(lidar_data[i])):
            # print('1x',lidar_data[i],':',angle,':',lidar_data[i] * math.sin(angle * math.pi / 180))
            # print('1y',lidar_data[i],':',angle,':',lidar_data[i] * math.cos(angle * math.pi / 180))
            x_coord.append( -1 * lidar_data[i] * math.sin(angle * math.pi / 180))
            y_coord.append( lidar_data[i] * math.cos(angle * math.pi / 180))
            obj_ang.append(-angle)
            obj_ind.append(i)

    for i in range(90,len(lidar_data)):
        angle = i - 90
        if (not math.isinf(lidar_data[i])):
            # print('2x',lidar_data[i],':',angle,':',lidar_data[i] * math.sin(angle * math.pi / 180))
            # print('2y',lidar_data[i],':',angle,':',lidar_data[i] * math.cos(angle * math.pi / 180))
            x_coord.append(lidar_data[i] * math.sin(angle * math.pi / 180))
            y_coord.append(lidar_data[i] * math.cos(angle * math.pi / 180))
            obj_ang.append(angle)
            obj_ind.append(i)

    # print('x Data', getRoundList(x_coord,2))
    # print('y Data', getRoundList(y_coord,2))
    # print('obj angle', getRoundList(obj_ang,2))
    # print('obj  index', getRoundList(obj_ind,2))
    
    return [x_coord, y_coord, obj_ang, obj_ind]

# checking if we have cleared wall
def checkWallCleared(robot_pose, lidar_data, rotation):
    # print('RP:', robot_pose, 'Lidar:', lidar_data, 'Rotation:', rotation)
    side = 0
    edge_par = getWallIndex(lidar_data)
    print('Edge: ', edge_par)
    if rotation[1] == 1:
        side = edge_par[0][1] - edge_par[0][0]
    elif rotation[1] == -1:
        side = edge_par[len(edge_par)-1][1] - edge_par[len(edge_par)-1][0]
    print('Side:', side)
    return side

# function to check if robot is on the waypoint line
def checkWaypointLine(robot_pose, waypoint):
    
    waypoint_slope = math.atan2((wayPoints[waypoint][1] - wayPoints[waypoint-1][1]) , (wayPoints[waypoint][0] - wayPoints[waypoint-1][0]))
    robot_slope = math.atan2((wayPoints[waypoint][1] - robot_pose[1]) , (wayPoints[waypoint][0] - robot_pose[0]))
    print('WP:', waypoint_slope, 'RS:', robot_slope)
    if abs(waypoint_slope - robot_slope) < 0.01:
        return True
    else:
        return False

# the main function which executes the robot code
if __name__ == "__main__":

    # creating motor, position_sensor, lidar, gps variables
    motors = initializeMotors()
    ps = initializePositionSensors()
    lidar = initializeLidar()
    gps = initializeGps()
    
    # variable to hold the pose of the robot
    robot_pose = [0, 0, 1.5708]
    prev_pose = [0, 0, 0]
    start_gps = [0, 0]
    


    prev_ps=np.array([0,0])
    velocity = 0
    maxVelocity = 1

    turnAngle = 0
    entry_point = []
    wall_follow = 0
    wall_navigation_type = "onspot"
    wall_angles = []
    
    waypoint_count = 1

    navigation_type = "waypoint"
    get_waypoint_angle = True

    get_wall_angle = True
    corner_details = [0.0,0.0]

    saved_pose = [[0.0, 0.0, 0.0], 0.0]
    reached_angle = False
    reached_wall_angle = False

    Move = True
    while robot.step(timestep) != -1:
        time = robot.getTime()
        
        if ( time == 0.064):
            print('Getting initial GPS data',getGpsValues(gps, start_gps))
            start_gps = getGpsValues(gps, start_gps)

        print('----------',round(time,2),'---------', navigation_type, '------------------')

        # getting the value of velocity from smoothening function
        velocity = getVelocity(maxVelocity, velocity)
        
        # calculation of robot pose
        robot_pose = getRobotPose(robot_pose,getPositionSensor(ps), prev_ps )
        
        # printing robot pose and next waypoint
        
        print('Robot Pose:', getRoundList(robot_pose, 3))
        # print('WayPoint', wayPoints[waypoint_count], 'Angle', turnAngle)
        # print('GPS ', getRoundList(getGpsValues(gps, start_gps),3))
        print('Waypoint line: ', checkWaypointLine(robot_pose, waypoint_count))
        # getting lidar data (point cloud) from getter function 
        lidar_data =  getLidar(lidar[0])
        # print('Lidar data', getRoundList(lidar_data, 2))

        # lidar map
        lidar_obstacle = obstacleDetection(lidar_data, turtlebot3["clearance"])
        #  getLidarObstacleCoordinate(lidar_data)
        # print('Lidar Obstacle:', lidar_obstacle)

        if lidar_obstacle[0]:
            navigation_type = "wallfollow"

        print('Wall Navigation:', wall_navigation_type)
        # getAngleRobot(robot_pose, [-0.5, -0.5])
        test_time = True
        if test_time and navigation_type == "waypoint":
            if get_waypoint_angle:
                print('@1')
                gps_data = getGpsValues(gps, start_gps)
                saved_pose = [[gps_data[0], gps_data[1], robot_pose[2]],0.0]
                reached_angle = False
                turnAngle = round(getAngleRobot(robot_pose, wayPoints[waypoint_count]), 3)
                # print('Waypoint Turn angle:', turnAngle, reached_angle)
                get_waypoint_angle = False
            
            angleCheck = checkAngularWaypoint(robot_pose, turnAngle, reached_angle)
            # print('angl ', angleCheck)
            if angleCheck[0]:
                maxVelocity = 1
                moveRobot(motors, velocity)
                reached_angle = True
                
                if checkWaypoint(robot_pose, wayPoints[waypoint_count]):
                    print("reached Waypoint:", waypoint_count)
                    stopRobot(motors)
                    
                    waypoint_count = waypoint_count + 1
                    if (waypoint_count == len(wayPoints)):
                        print('Finished Navigation')
                        stopLidar(lidar[1], lidar[2])
                        break
                    get_waypoint_angle = True
            else:
                maxVelocity = 0.5

                if not angleCheck[0]:
                    rotateRobot(motors, velocity,angleCheck[1])
                else: 
                    print('Reached')
        # when wall is detected
        elif test_time and navigation_type == "wallfollow":
            print('wall Angles:', wall_angles)
            # stopRobot(motors)
            if get_wall_angle:
                print('@2')
                gps_data = getGpsValues(gps, start_gps)
                reached_wall_angle = False
                corner_details = getBestCorner(robot_pose, lidar_obstacle[1][0]["left-corner"],lidar_obstacle[1][0]["right-corner"],wayPoints[waypoint_count])
                get_wall_angle = False
                # saved_pose = [robot_pose, checkAngularWaypoint(robot_pose, corner_details[1], reached_wall_angle)[1]]
                saved_pose = [[gps_data[0], gps_data[1], robot_pose[2]], checkAngularWaypoint(robot_pose, corner_details[1], reached_wall_angle)[1]]
                wall_angles.append(corner_details[1])
            
            # print('Corner Details:', corner_details)
            if wall_navigation_type == "onspot":
                angleCheck = checkAngularWaypoint(robot_pose, corner_details[1], reached_wall_angle)
            elif wall_navigation_type == "curve":
                angleCheck = checkEdgeAngle(robot_pose, saved_pose[0][2])
            
            print('Angle Check:', angleCheck, )

            if angleCheck[0] and not (wall_navigation_type == "curve"):
                print('Linear')
                maxVelocity = 1
                moveRobot(motors, velocity)
                reached_angle = True
                # print('Corner Details: ', corner_details)

                if wall_navigation_type == "sidefollow" and checkWaypointLine(getGpsValues(gps, start_gps), waypoint_count):
                    print('Moved Out of the obstacle')
                    # stopRobot(motors)
                    # break
                    navigation_type = "waypoint"
                    get_waypoint_angle = True
                
                if checkWallCleared(robot_pose, lidar_data,saved_pose) < 2:
                    print("reached Edge end:")
                    stopRobot(motors)
                    wall_navigation_type = "curve"
                    
            elif wall_navigation_type == "onspot":
                print('OnSp')
                maxVelocity = 0.5
                rotateRobot(motors, velocity,saved_pose[1])
            elif wall_navigation_type == "curve":
                print('Curv')
                maxVelocity = 0.5
                curveRobot(motors, velocity, saved_pose[1])
                if angleCheck[0]:
                    print('Curve Traversed')
                    wall_navigation_type = "sidefollow"
                    stopRobot(motors)
                    saved_pose = [[gps_data[0], gps_data[1], wall_angles[0] + saved_pose[1] * 3.14], checkAngularWaypoint(robot_pose, corner_details[1], reached_wall_angle)[1]]
        else:
            print(getWallIndex(lidar_data))


        prev_ps = getPositionSensor(ps)
