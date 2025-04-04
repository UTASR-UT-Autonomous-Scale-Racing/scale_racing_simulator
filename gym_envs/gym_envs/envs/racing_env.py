'''
Here is outlined a custom gymnasium environment for our racing simulator.
'''

# gynasium (for reinforcement learning setup)
import gymnasium as gym
from gymnasium import spaces
import numpy as np

# pybullet (for physics simulation)
import pybullet as p

# other
import time
import os

# custom
# from construct_environment import construct_environment #TODO move this inside the package
from gym_envs.envs.construct_environment import construct_environment, find_centerline

class RacingEnv(gym.Env): # subclass the Gym.Env
    # follow: follows behind the car as it goes around the track
    # bird-eye: birds eye view of the track. 
    # also implicitly supports no rendering. 
    metadata = {"render_mode": ["follow", "bird-eye"], "render_fps":60} # define the render modes that will be supported. 

    def __init__(
            self, 
            inner_track_path,
            outer_track_path,
            start_position = [0,4,0.1],
            start_orientation = [0,0,0,1.0],
            render_mode=None,
            target_frame_period=1./240,
            car_path="car2.urdf", # "car.urdf",
            wheel_indices = [1,3,4,5], # [4,6,7,8],
            steering_indices = [0,2],
            speed_modifier = 100
            ): # add any other input parameters here
        
        self.inner_track_path = inner_track_path # the path to the inner track (.obj)
        self.outer_track_path = outer_track_path # the path to the outer track (.obj)
        self.target_frame_period = target_frame_period # the target period for each loop frame (1/240 is real time)
        self.start_position = start_position # 3 element list holding the car's starting position.
        self.start_orientation = start_orientation # 3 element euler representation of the car's start orientation.
        self.car_path = car_path # path to car urdf. 
        self.wheel_indices = wheel_indices # the joint indices for the wheels. 
        self.steering_indices = steering_indices
        self.speed_modifier = speed_modifier # the amount to mulitiply the throttle by.

        assert render_mode is None or render_mode in self.metadata["render_mode"]
        self.render_mode = render_mode

        # set the physics client depending on render mode. 
        self.physicsClient = None
        if self.render_mode == None:
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)


        # configure the simulation environment
        # set up the track
        self.outer_track_id, self.inner_track_id = construct_environment(self.outer_track_path, self.inner_track_path)
        self.centerline = find_centerline(self.outer_track_id, self.inner_track_id)

        # load the car:
        p.setAdditionalSearchPath(os.getcwd())

        self.carID = p.loadURDF(self.car_path, self.start_position, self.start_orientation)



        # define the observation space

        # find the track bounds dynamically from the environment. 
        outer_track_aabb_min, outer_track_aabb_max = p.getAABB(self.outer_track_id)
        inner_track_aabb_min, inner_track_aabb_max = p.getAABB(self.inner_track_id)

        # Combine the bounding boxes to get the absolute minimum and maximum
        combined_min_position = [
            min(outer_track_aabb_min[i], inner_track_aabb_min[i]) for i in range(3)
        ]
        combined_max_position = [
            max(outer_track_aabb_max[i], inner_track_aabb_max[i]) for i in range(3)
        ]

        # Define observation space bounds (example values, adjust as needed)
        position_low = np.array(combined_min_position)  # 3D position min
        position_high = np.array(combined_max_position)    # 3D position max

        #TODO find the correct ranges for the rest of these 
        orientation_low = np.array([-1, -1, -1, -1])  # Quaternion min
        orientation_high = np.array([1, 1, 1, 1])    # Quaternion max

        linear_velocity_low = np.array([-5, -5, -5])  # 3D linear velocity min
        linear_velocity_high = np.array([5, 5, 5])    # 3D linear velocity max

        angular_velocity_low = np.array([-5, -5, -5])  # 3D angular velocity min
        angular_velocity_high = np.array([5, 5, 5])    # 3D angular velocity max

        # Concatenate all components
        low = np.concatenate([position_low, orientation_low, linear_velocity_low, angular_velocity_low])
        high = np.concatenate([position_high, orientation_high, linear_velocity_high, angular_velocity_high])

        # Define the observation space
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float64)

        # define the action space
        action_low = np.array([-1.0, -1.0])  # Full reverse throttle, full left steer
        action_high = np.array([1.0, 1.0])   # Full forward throttle, full right steer

        self.action_space = spaces.Box(low=action_low, high=action_high, dtype=np.float64)
    
    def _get_obs(self):
        '''
        Private method to translate the environment's state into an observation. 
        This is useful because we need to compute observations in both reset and step methods.
        '''
        
        cp, co = p.getBasePositionAndOrientation(self.carID)
        lv, av = p.getBaseVelocity(self.carID)

        # print("cp: ", cp)
        # print("co: ", co)
        # print("lv: ", lv)
        # print("av: ", av)

        return np.concatenate([
            cp, # position
            co, # orientation
            lv, # linear velocity
            av # angular velocity
        ]).astype(np.float64)
    
    def _get_info(self):
        '''
        private method that gets the auxiliary information that is retured by step and reset.
        '''
        info = {}
        return info
    
    def get_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.carID)

    def reset(self, seed=None, options=None):
        '''
        Determines how to reset the environment for the next episode
        '''
        super().reset(seed=seed)

        # set the initial state (position, velocity, etc.)
        p.resetBaseVelocity(self.carID, linearVelocity=[0,0,0], angularVelocity=[0,0,0])
        p.resetBasePositionAndOrientation(self.carID, self.start_position, self.start_orientation)

        obs = self._get_obs()
        info = self._get_info()

        print("obs returned from reset: ", obs)

        return obs, info # initial observation must match the observation_space format. 
    
    def step(self, action):
        '''
        Defines how the environment updates based on an action.
        '''

        # interpret the action, first element is throttle (-1 to 1) second is steering (-1 to 1)
        throttle = action[0]
        
        steering = action[1]

        throttle = throttle * self.speed_modifier
        # print("throttle: ", throttle)
        # steering = 

        # update the wheel joints in response to the throttle
        for wheel in self.wheel_indices:
            # print("found wheel")
            p.setJointMotorControl2(
                bodyUniqueId = self.carID,
                jointIndex = wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity = throttle,
                force = 10000
            )

        for joint in self.steering_indices:
            # Apply the steering angle to the steer_joint (joint 1)
            p.setJointMotorControl2(bodyUniqueId=self.carID,
                                    jointIndex=joint,  # steer_joint
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=steering,
                                    force=1000.0)  # Adjust max force as needed

        # print(p.getJointState(self.carID, self.wheel_indices[0]))
        p.stepSimulation()
        time.sleep(1./240.)
        # time.sleep(2)

        # determine what rewards (if any) have been obtained in this step.         

        # apply action to update state
        next_state = None # the updated state
        reward = None # a scalar value indicating progress
        terminated = False # True if the episode is over
        truncated = False # True if the episode was cut short
        info = {} # additional debugging info

        self.render()

        return next_state, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "follow":
            object_position, object_orientation = p.getBasePositionAndOrientation(self.carID)
            p.resetDebugVisualizerCamera(
                cameraDistance=5,
                cameraYaw=-90,
                cameraPitch=-30,
                cameraTargetPosition=object_position
            )
        elif self.render_mode == "bird-eye":
            object_position, object_orientation = p.getBasePositionAndOrientation(self.carID)
            p.resetDebugVisualizerCamera(
                cameraDistance=5,
                cameraYaw=-90,
                cameraPitch=-89,
                cameraTargetPosition=object_position
            )
        else: 
            # no render. 
            pass
    
    def close(self):
        '''
        cleans up resources when finished
        e.g. close the pybullet environment, and shut down rendering.
        '''
        p.disconnect(self.physicsClient)