"""DeepBot_FAIRIS controller."""




from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv

from utilities import normalize_to_range
from PPO_agent import PPOAgent, Transition


from gym.spaces import Box, Discrete
import numpy as np

# get the time step of the current world.

GPS_MIN = -2.5708
GPS_MAX = 2.3711


class FAIRIS (RobotSupervisorEnv):
    def __init__(self):
        super().__init__()
        self.Robot = self.getSelf()
        self.timestep = int(self.getBasicTimeStep())

        self.observation_space = Box(low=np.array([0.0, 0.0, 0.0, 0.0, GPS_MIN]),
                                     high=np.array([2.0, 2.0, 2.0, 2.0, GPS_MAX]),
                                     dtype=np.float64)
        self.action_space = Discrete(2)

        #FAIRIS motors for all 4 wheels 
        self.front_left_motor = self.getDevice('front left wheel motor')
        self.front_right_motor = self.getDevice('front right wheel motor')
        self.rear_left_motor = self.getDevice('rear left wheel motor')
        self.rear_right_motor = self.getDevice('rear right wheel motor')
        self.all_motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in self.all_motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(5)
        
        # Webots Astra Camera: https://cyberbotics.com/doc/guide/range-finder-sensors#orbbec-astra
        self.depth_camera = self.getDevice('camera depth')
        self.depth_camera.enable(self.timestep)
        self.rgb_camera = self.getDevice('camera rgb')
        self.rgb_camera.enable(self.timestep)

        # Webots RpLidarA2: https://www.cyberbotics.com/doc/guide/lidar-sensors#slamtec-rplidar-a2
        self.lidar = self.getDevice('lidar')
        self.lidar.enable(self.timestep)
        #self.lidar_value = self.lidar.getRangeImage()
        self.lidar.enablePointCloud()
        self.lidar.disablePointCloud()

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

    def get_observations(self):
        front_left_distance = self.front_left_ds.getValue()
        front_right_distance = self.front_right_ds.getValue()
        rear_left_distance = self.rear_left_ds.getValue()
        rear_right_distance = self.rear_right_ds.getValue()
        self.Coordinate_Y = self.gps.getValues()[1]
        #print(self.Coordinate_Y)
        return [front_left_distance,front_right_distance, rear_left_distance, rear_right_distance, self.Coordinate_Y]
    


    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]
    
    def get_reward(self, action=None):
        front_left_distance = self.front_left_ds.getValue()
        front_right_distance = self.front_right_ds.getValue()
        rear_left_distance = self.rear_left_ds.getValue()
        rear_right_distance = self.rear_right_ds.getValue()
        #print("the lowest is {}".format(self.lowest))
        if front_left_distance < self.lowest:
            self.lowest = front_left_distance
            return 1
        if front_right_distance < self.lowest:
            self.lowest = front_right_distance
            return 1
        if rear_left_distance < self.lowest:
            self.lowest = rear_left_distance
            return 1
        if rear_right_distance < self.lowest:
            self.lowest = rear_right_distance
            return 1
        return 0

    def is_done(self):
        if self.episode_score >100:
            return True
        if self.getTime() > 60:
            return True
        self.Coordinate_Y = self.gps.getValues()[1]
        if  self.Coordinate_Y <= GPS_MIN or self.Coordinate_Y >= GPS_MAX:
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
        action = int(action[0])   #something related to the discrete action space
                                    #Get the action the agent output, transform into physical motion of robot 
        if action == 0:
            motor_speed = 5
        else:
            motor_speed = -5   #####################################################################
        self.front_left_motor.setPosition(float('inf'))
        self.front_left_motor.setVelocity(motor_speed)
        self.front_right_motor.setPosition(float('inf'))
        self.front_right_motor.setVelocity(motor_speed)
        self.rear_left_motor.setPosition(float('inf'))
        self.rear_left_motor.setVelocity(motor_speed)
        self.rear_right_motor.setPosition(float('inf'))
        self.rear_right_motor.setVelocity(motor_speed)
        
        


solved = False
episode_count = 0
episode_limit = 200

environment = FAIRIS()
agent = PPOAgent(number_of_inputs=environment.observation_space.shape[0], number_of_actor_outputs=environment.action_space.n)

while not solved and episode_count <episode_limit:
    observation = environment.reset()
    environment.lowest = 2
    environment.episode_score = 0

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


#https://github.com/aidudezzz/deepbots/blob/dev/deepbots/supervisor/controllers/robot_supervisor_env.py






