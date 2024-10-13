"""FAIRIS_Controller controller."""

from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv

from utilities import normalize_to_range
from PPO_agent import PPOAgent, Transition


from gym.spaces import Box, Discrete
import numpy as np
import math
import random


# get the time step of the current world.

GPS_MIN = -2.5708
GPS_MAX = 2.3711


class FAIRIS (RobotSupervisorEnv):
    def __init__(self):
        super().__init__()
        self.Robot = self.getSelf()
        self.timestep = int(self.getBasicTimeStep())

        self.observation_space = Box(low=np.array([0,0]),
                                     high=np.array([5,799]),
                                     dtype=np.int64)
        self.action_space = Discrete(7)

        #FAIRIS motors for all 4 wheels 
        self.front_left_motor = self.getDevice('front left wheel motor')
        self.front_right_motor = self.getDevice('front right wheel motor')
        self.rear_left_motor = self.getDevice('rear left wheel motor')
        self.rear_right_motor = self.getDevice('rear right wheel motor')
        self.all_motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in self.all_motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)
        self.max_motor_velocity = self.rear_left_motor.getMaxVelocity()
        
        # Webots Astra Camera: https://cyberbotics.com/doc/guide/range-finder-sensors#orbbec-astra
        self.depth_camera = self.getDevice('camera depth')
        self.depth_camera.enable(self.timestep)
        self.rgb_camera = self.getDevice('camera rgb')
        self.rgb_camera.enable(self.timestep)

        # Webots RpLidarA2: https://www.cyberbotics.com/doc/guide/lidar-sensors#slamtec-rplidar-a2
        self.lidar = self.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()


        # Webots IMU: https://www.cyberbotics.com/doc/guide/imu-sensors#mpu-9250
        # Webots IMU Accelerometer: https://www.cyberbotics.com/doc/reference/accelerometer
        self.accelerometer = self.getDevice('imu accelerometer')
        self.accelerometer.enable(self.timestep)

        # Webots IMU Gyro: https://www.cyberbotics.com/doc/reference/gyro
        self.gyro = self.getDevice('imu gyro')
        self.gyro.enable(self.timestep)

        # Webots IMU Compass: https://www.cyberbotics.com/doc/reference/compass
        self.compass = self.getDevice('imu compass')
        self.compass.enable(self.timestep)

        # Webots GPS:
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        self.Coordinate_X = []
        self.Coordinate_Y = []

        # Webots Disance Sensors: https://www.cyberbotics.com/doc/reference/distancesensor
        self.front_left_ds = self.getDevice('front left distance sensor')
        self.front_right_ds = self.getDevice('front right distance sensor')
        self.rear_left_ds = self.getDevice('rear left distance sensor')
        self.rear_right_ds = self.getDevice('rear right distance sensor')
        self.all_distance_sensors = [self.front_left_ds,self.front_right_ds, self.rear_right_ds,self.rear_left_ds]
        for ds in self.all_distance_sensors:
            ds.enable(self.timestep)

        self.steps_per_episode = 200  # Max number of steps per episode
        self.episode_score = 0  # Score accumulated during an episode
        self.episode_score_list = []  # A list to save all the episode scores, used to check if task is solved
        #sets up the variables for running simulation
        self.lowest = 2
        self.max_cluster = 4
        self.detections = []

    def get_observations(self):
        self.detections = self.Scan_Progress()
        print("{}".format(self.detections))
        clusters = self.get_clusters(self.detections)
        sections = len(self.detections)
        print("the number of clusters is {}, the length is {}".format(clusters,sections))
        print("_____________________________________________________________________________")
        return [clusters, sections]
    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]
    def get_reward(self, action=None):
        size = len(self.detections)
        current_clusters = self.get_clusters(self.detections) 
        if(current_clusters == 1):
            if size < 25:
                return 7
            elif size >= 25 and size <= 75:
                return 3
            elif size > 75 and size < 100:
                return 2
            else:
                return 1
        if(current_clusters == self.max_cluster):
            return 0
        if(current_clusters < self.max_cluster):######################################this comes before so fix this shit
            self.max_cluster = current_clusters
            return 1
        return 0
    def is_done(self):
        if self.get_clusters(self.detections) > 4 or self.get_clusters(self.detections) > self.max_cluster:
            return True
        if self.getTime() > 60:
            return True
        return False
    
    def solved(self):
        if (len(self.episode_score_list)>100):     #if there has been OVER 100 trials/episodes
            if np.mean(self.episode_score_list[-100:]) > 195.0:  # Last 100 episodes' scores average value
               return True
        return False

    def get_info(self):
        return None

    def render(self, mode='human'):
        pass
    
    def apply_action(self,action):
        action = int(action[0])   #something related to the discrete action space  #Get the action the agent output, transform into physical motion of robot 
        #print("{}".format(action),end=" ")
        if action == 0:                                     # Move Forwards
            for motor in self.all_motors:
                motor.setPosition(float('inf'))
                motor.setVelocity(5)
        elif action == 1:                                   # Move Backwards
            for motor in self.all_motors:
                motor.setPosition(float('inf'))
                motor.setVelocity(-5)
        elif action == 2:                                   #Small left turn
            self.front_left_motor.setVelocity(-1 * 2.5)
            self.rear_left_motor.setVelocity(-1 * 2.5)
            self.front_right_motor.setVelocity(1 * 2.5)
            self.rear_right_motor.setVelocity(1 * 2.5)
        elif action == 3:                                   #Small right turn
            self.front_left_motor.setVelocity(1 * 2.5)
            self.rear_left_motor.setVelocity(1 * 2.5)
            self.front_right_motor.setVelocity(-1 * 2.5)
            self.rear_right_motor.setVelocity(-1 * 2.5)
        elif action == 4:                                   #Big left turn
            self.front_left_motor.setVelocity(-1 * 5)
            self.rear_left_motor.setVelocity(-1 * 5)
            self.front_right_motor.setVelocity(1 * 5)
            self.rear_right_motor.setVelocity(1 * 5)
        elif action == 5:                                   #Big left turn
            self.front_left_motor.setVelocity(1 * 5)
            self.rear_left_motor.setVelocity(1 * 5)
            self.front_right_motor.setVelocity(-1 * 5)
            self.rear_right_motor.setVelocity(-1 * 5)
        else:                                               # Stop Moving
            for motor in self.all_motors:
                motor.setPosition(float('inf'))
                motor.setVelocity(0)
            
        
        


    """def velocity_saturation(self, motor_velocity):
        if motor_velocity > self.max_motor_velocity:
            return self.max_motor_velocity
        elif motor_velocity < -1*self.max_motor_velocity:
            return -1*self.max_motor_velocity
        else:
            return motor_velocity
    def get_bearing(self):
        compass_reading = self.compass.getValues()
        rad = math.atan2(compass_reading[0],compass_reading[1]) + math.pi/2
        bearing = (rad - math.pi/2) / math.pi * 180.0
        if bearing < 0.0:
            bearing += 360.0
        return round(bearing,3)
    def rotation_PID(self, end_bearing, K_p=1):
        delta =  end_bearing - self.get_bearing()
        velocity = self.velocity_saturation(K_p * abs(delta))
        if -180 <= delta <= 0 or 180 < delta <= 360:
            self.front_left_motor.setVelocity(1 * velocity)
            self.rear_left_motor.setVelocity(1 * velocity)
            self.front_right_motor.setVelocity(-1 * velocity)
            self.rear_right_motor.setVelocity(-1 * velocity)
        elif 0 < delta <= 180 or -360 <= delta < -180:
            self.front_left_motor.setVelocity(-1 * velocity)
            self.rear_left_motor.setVelocity(-1 * velocity)
            self.front_right_motor.setVelocity(1 * velocity)
            self.rear_right_motor.setVelocity(1 * velocity)
    def rotate_to(self, end_bearing, margin_error = .001):
        while self.step(self.timestep) != -1:
            self.rotation_PID(end_bearing)
            if end_bearing - margin_error <= self.get_bearing() <= end_bearing + margin_error:
                self.stop()
                break
    def rotate(self,degree, margin_error = .01):
        start_bearing = self.get_bearing()
        end_bearing = start_bearing - degree
        if end_bearing > 360:
            end_bearing -= 360
        elif end_bearing < 0:
            end_bearing += 360
        self.rotate_to(end_bearing, margin_error=margin_error)"""


    def Scan_Progress(self):
        detections = []
        scan = self.lidar.getRangeImage()
        for section in scan:
            if section > .2 and section < 12:
                #print("The index: {}, The value: {}".format(scan.index(section),section))
                detections.append(scan.index(section))
        #print(detections)
        #print("_____________________________________________________________________")
        return detections


    def get_clusters(self, detections):
        #nothing = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
        cluster = 1       #random.choice(nothing)
        #print(detections)
        for section in detections:
            if (section+1 -section>1):
                #print("Fortnite, this shit occurs so cluster should increase")
                cluster = cluster + 1
        return cluster


    
        
        


solved = False
episode_count = 0
episode_limit = 200

environment = FAIRIS()
agent = PPOAgent(number_of_inputs=environment.observation_space.shape[0], number_of_actor_outputs=environment.action_space.n)

while not solved and episode_count < episode_limit:
    observation = environment.reset()
    environment.lowest = 2
    environment.max_cluster = 4
    environment.episode_score = 0
    environment.detections = []

    for step in range(environment.steps_per_episode):

        selected_action, action_prob = agent.work(observation, type_="selectAction")

        new_observation, reward, done, info = environment.step([selected_action])

        trans = Transition(observation, selected_action, action_prob, reward, new_observation)
        agent.store_transition(trans)

        if done:
            environment.episode_score_list.append(environment.episode_score)
            agent.train_step(batch_size=step+1)
            solved = environment.solved()
            break

        environment.episode_score += reward
        observation = new_observation

    print("Episode #", episode_count, "score:", environment.episode_score)
    episode_count += 1 

if not solved:
    print("Task is not solved, deploying agent for testing...")
elif solved:
    print("Task is solved, deploying agent for testing...")

observation = environment.reset()
environment.episode_score = 0.0
while True:
    selected_action, action_prob = agent.work(observation, type_="selectActionMax")
    observation, _, done, _ = environment.step([selected_action])
    if done:
        observation = environment.reset()