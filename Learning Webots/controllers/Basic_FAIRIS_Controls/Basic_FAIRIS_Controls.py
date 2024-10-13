"""Basic_FAIRIS_Controls controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import struct

import math, sys


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

front_left_motor = robot.getDevice('front left wheel motor')
front_right_motor = robot.getDevice('front right wheel motor')
rear_left_motor = robot.getDevice('rear left wheel motor')
rear_right_motor = robot.getDevice('rear right wheel motor')
all_motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
for motor in all_motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
        
        # Webots Astra Camera: https://cyberbotics.com/doc/guide/range-finder-sensors#orbbec-astra
depth_camera = robot.getDevice('camera depth')
depth_camera.enable(timestep)
rgb_camera = robot.getDevice('camera rgb')
rgb_camera.enable(timestep)

        # Webots RpLidarA2: https://www.cyberbotics.com/doc/guide/lidar-sensors#slamtec-rplidar-a2
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()
detections = []

        # Webots IMU: https://www.cyberbotics.com/doc/guide/imu-sensors#mpu-9250
        # Webots IMU Accelerometer: https://www.cyberbotics.com/doc/reference/accelerometer
accelerometer = robot.getDevice('imu accelerometer')
accelerometer.enable(timestep)

        # Webots IMU Gyro: https://www.cyberbotics.com/doc/reference/gyro
gyro = robot.getDevice('imu gyro')
gyro.enable(timestep)

        # Webots IMU Compass: https://www.cyberbotics.com/doc/reference/compass
compass = robot.getDevice('imu compass')
compass.enable(timestep)

        # Webots GPS:
gps = robot.getDevice('gps')
gps.enable(timestep)

        # Webots Disance Sensors: https://www.cyberbotics.com/doc/reference/distancesensor
front_left_ds = robot.getDevice('front left distance sensor')
front_right_ds = robot.getDevice('front right distance sensor')
rear_left_ds = robot.getDevice('rear left distance sensor')
rear_right_ds = robot.getDevice('rear right distance sensor')
all_distance_sensors = [front_left_ds,front_right_ds, rear_right_ds,rear_left_ds]
for ds in all_distance_sensors:
    ds.enable(timestep)
    

close = depth_camera.getMinRange()
far = depth_camera.getMaxRange()
width = depth_camera.getWidth()
height = depth_camera.getHeight()
print("Depth Camera: the minimal range:{} the maximum range:{}".format(close, far))
print("Depth Camer: the image width:{} the image height:{}".format(width,height))


Field_of_View = lidar.getFov()
close = lidar.getMinRange()
far = lidar.getMaxRange()
print("Lidar: the minimal range:{} the maximum range:{}".format(close, far))
print("The Lidar FOV: {}".format(Field_of_View))
values_per_scan = lidar.getHorizontalResolution()
layers = lidar.getNumberOfLayers()
print("Lidar: HorizontalResolution: {}  Layers: {}".format(values_per_scan,layers))

def get_bearing(self):
        compass_reading = self.compass.getValues()
        rad = math.atan2(compass_reading[0],compass_reading[1]) + math.pi/2
        bearing = (rad - math.pi/2) / math.pi * 180.0
        if bearing < 0.0:
            bearing += 360.0
        return round(bearing,3)


def stop(self):
        for motor in self.all_motors:
            motor.setVelocity(0)
def go_forward(self,velocity=10):
        for motor in self.all_motors:
            motor.setVelocity(velocity)
 
once = 1
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    #fit = gps.getValues()[1]
    #print("left distance sensor is {}".format(front_left_ds.getValue()))
    #print("right distance sensor is {}".format(front_right_ds.getValue()))
    
    #how to use the Lidar
    detections = []
    scan = lidar.getRangeImage()
    for section in scan:
        if section > .2 and section < 12:
            #print("The index: {}, The value: {}".format(scan.index(section),section))
            detections.append(scan.index(section))
    #print(detections)
   
    #how to use the RangeFinder
    focus = depth_camera.getRangeImage()
    range_finder_width = depth_camera.getWidth()
    range_finder_height = depth_camera.getHeight()
    
    x = depth_camera.getWidth()
    y = depth_camera.getHeight()
    #print("gridX:{}, gridY:{}".format(x,y))
    #print(x*y*sys.getsizeof(float))
    #print("length of rangfider pixel detections {}".format(len(focus)))
    for x_coordinate in range(0, x):
        depth = depth_camera.rangeImageGetDepth(focus, range_finder_width, 5000, 0)
    #print("Pixel {}".format(depth))
    # for y_coordinate in range(0, 479):
    #   depth = depth_camera.rangeImageGetDepth(focus, range_finder_width, x, y)
   
    #print("{} {}".format(x_coordinate,y_coordinate))
    #print(len(focus))
    #while once:
    #    print(focus)
    #    once = 0
    
    for motor in all_motors:
        motor.setVelocity(10)
 
    compass_reading = compass.getValues()
    rad = math.atan2(compass_reading[0],compass_reading[1]) + math.pi/2
    bearing = (rad - math.pi/2) / math.pi * 180.0
    if bearing < 0.0:
        bearing += 360.0
    get_bearing = (round(bearing,3))

    
    
    pass

# Enter here exit cleanup code.
