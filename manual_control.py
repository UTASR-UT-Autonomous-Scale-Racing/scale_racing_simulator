'''
Allows for manual control over the car, for testing. 
'''
import pybullet as p
import time
import os
from gym_envs.envs import construct_environment
import sys


inner_track_path = "./tracks/track_basic/inner.obj"
outer_track_path = "./tracks/track_basic/outer.obj"
target_dt = 1./240. # the target period for each loop (1/240 is real time)

physicsClient = p.connect(p.GUI) # start the graphical interface
construct_environment.construct_environment(outer_track_path, inner_track_path)

# checking physics parameters:
params = p.getPhysicsEngineParameters()
print(f"Timestep: {params['fixedTimeStep']}")
# print(f"Real-time mode: {params['realTimeSimulation']}")

# load the car
carStartPos = [0,4,0.1]
carStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath(os.getcwd())
# carId = p.loadURDF("car.urdf", carStartPos, carStartOrientation)
carId = p.loadURDF("car2.urdf", [1,4,0.1], carStartOrientation)



# print("\n\n")

# num_joints = p.getNumJoints(car2Id)
# for i in range(num_joints):
#         joint_info = p.getJointInfo(car2Id, i)
#         joint_name = joint_info[1].decode("utf-8")
#         # link_name = joint_info[12].decode("utf-8")  # JointInfo[12] is the child link name
#         print(joint_info)
#         # link_name = None
#         # if joint_name == "left_hinge_to_left_front_wheel":
#         #     print(f"Found at joint index {i}")
#         #     steer_link_index = i
#         #     break

# num_joints = p.getNumLinks(carId)
# for i in range(num_joints):
#         joint_info = p.getJointInfo(car2Id, i)
#         joint_name = joint_info[1].decode("utf-8")
#         # link_name = joint_info[12].decode("utf-8")  # JointInfo[12] is the child link name
#         print(joint_info)

# print("\n\n")

# sys.exit(1)

# base to left hinge: joint index 0
# base to right hinge: joint index 2
# left_hinge_to_left_front_wheel index 1
# right_hinge_to_right_front_wheel index 3
# base_to_left_back_wheel index 4
# base_to_right_back_wheel index 5

# set steering

# p.setJointMotorControl2(
#     bodyUniqueId=car2Id,
#     jointIndex=0,  # steer_joint
#     controlMode=p.POSITION_CONTROL,
#     targetPosition=-0.5,
#     force=100
#     )  # Adjust max force as needed

# p.setJointMotorControl2(
#     bodyUniqueId=car2Id,
#     jointIndex=2,  # steer_joint
#     controlMode=p.POSITION_CONTROL,
#     targetPosition=-0.5,
#     force=100
#     )  # Adjust max force as needed

# # set wheel motion.
# for wheel in [1,3,4,5]:
#     p.setJointMotorControl2(bodyUniqueId=car2Id,
#         jointIndex=wheel,
#         controlMode=p.VELOCITY_CONTROL,
#         targetVelocity=10,
#         force=10000)  # Max force to apply

# sys.exit(1)


# wheel_indices = [4, 6, 7, 8] # joint indices for wheels
# for car2
wheel_indices = [1,3,4,5]

p.changeDynamics(carId, 1, lateralFriction=0.5)
p.changeDynamics(carId, 3, lateralFriction=0.5)
p.changeDynamics(carId, 4, lateralFriction=0.5)
p.changeDynamics(carId, 5, lateralFriction=0.5)

# add control parameters to pybullet GUI
throttle_slider = p.addUserDebugParameter("Throttle", -100, 100, 0)
steering_slider = p.addUserDebugParameter("Steering", -1, 1, 0)

# add output parameters to pybullet GUI

# an angle representing how far off "north" we are, in radians
orientation_text_id = p.addUserDebugText(
    text = "Orientation: 0 rad",
    textPosition = [0,0,0],
    textColorRGB = [0,0,0],
    lifeTime = 0
)

# the x,y position of the robot
position_text_id = p.addUserDebugText(
    text = "Position: (0,0) m from start",
    textPosition = [0,0,0],
    textColorRGB = [0,0,0],
    lifeTime = 0
)

# the x, y velocity of the robot
linear_velocity_text_id = p.addUserDebugText(
    text = "L Velocity: (0,0) m/s",
    textPosition = [0,0,0],
    textColorRGB = [0,0,0],
    lifeTime = 0
)

# the angular velocity of the robot
angular_velocity_text_id = p.addUserDebugText(
    text = "A Velocity: (0,0,0) rad/s",
    textPosition = [0,0,0],
    textColorRGB = [0,0,0],
    lifeTime = 0
)

prev_time = time.time()
for i in range(10000):
    start_time = time.time()

    # read the control inputs:
    throttle_value = p.readUserDebugParameter(throttle_slider)
    steering_value = p.readUserDebugParameter(steering_slider)

    # apply them to the relevant joints
    for wheel in wheel_indices: # TODO figure out if the wheels are sliding
        p.setJointMotorControl2(bodyUniqueId=carId,
                            jointIndex=wheel,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=throttle_value,
                            force=10000)  # Max force to apply

    #TODO fix steering control

    # find the steering link: 
    # Iterate through joints to find the one that connects to "steer_link"
    # num_joints = p.getNumJoints(carId)

    # for i in range(num_joints):
    #     joint_info = p.getJointInfo(carId, i)
    #     link_name = joint_info[12].decode("utf-8")  # JointInfo[12] is the child link name
        
    #     if link_name == "steer_link":
    #         print(f"Found 'steer_link' at joint index {i}")
    #         steer_link_index = i
    #         break

    # TODO might need to increase the lateral friction of the car

    


    # p.changeDynamics(carId, 4, lateralFriction=0.5)
    # p.changeDynamics(carId, 6, lateralFriction=0.5)
    # print(p.getJointState(carId, 4)) # position, velocity, reaction_forces, applied_joint_motor_torque
    # print(p.getJointState(carId, 6))


    #TODO i think the urdf of the car i'm using is borked, since the wheels aren't actually connected to the steering link. 

    joint_state = p.getJointState(carId, 1)
    # print("Joint Position:", joint_state[0])
    # print("Joint Velocity:", joint_state[1])
    steering_value = max(-0.5, min(0.5, steering_value))
    # Apply the steering angle to the steer_joint (joint 1)
    p.setJointMotorControl2(bodyUniqueId=carId,
                            jointIndex=0, # 1 for car1,  # steer_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=steering_value,
                            force=100)  # Adjust max force as needed
    
    p.setJointMotorControl2(bodyUniqueId=carId,
                            jointIndex=2,  # steer_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=steering_value,
                            force=100)  # Adjust max force as needed

    # update outputs
    cp, co = p.getBasePositionAndOrientation(carId)
    lv, av = p.getBaseVelocity(carId)

    # first find the positions to display them above the car: 
    position_text_loc = [cp[0], cp[1], cp[2] + 1]
    orientation_text_loc = [cp[0], cp[1], cp[2] + 1.2]
    linear_velocity_text_loc = [cp[0], cp[1], cp[2] + 1.4]
    angular_velocity_text_loc = [cp[0], cp[1], cp[2] + 1.6]

    _, _, yaw = p.getEulerFromQuaternion(co)
    p.addUserDebugText(
        text = f"Orientation: {yaw:.2f} rad",
        textPosition = orientation_text_loc,
        textColorRGB = [0,0,0],
        replaceItemUniqueId = orientation_text_id
    )

    relative_pos = [cp[i] - carStartPos[i] for i in range(3)]
    p.addUserDebugText(
        text = f"Position: ({relative_pos[0]:.2f},{relative_pos[1]:.2f}) m from start",
        textPosition = position_text_loc,
        textColorRGB = [0,0,0],
        replaceItemUniqueId = position_text_id
    )

    p.addUserDebugText(
        text = f"L Velocity: ({lv[0]:.2f},{lv[1]:.2f}) m/s",
        textPosition = linear_velocity_text_loc,
        textColorRGB = [0,0,0],
        replaceItemUniqueId = linear_velocity_text_id
    )

    p.addUserDebugText(
        text = f"A Velocity: ({av[0]:.2f},{av[1]:.2f},{av[2]:.2f}) rad/s",
        textPosition = angular_velocity_text_loc,
        textColorRGB = [0,0,0],
        replaceItemUniqueId = angular_velocity_text_id
    )

    object_position, object_orientation = p.getBasePositionAndOrientation(carId)
    p.resetDebugVisualizerCamera(
        cameraDistance=2,
        cameraYaw=-90,
        cameraPitch=-30,
        cameraTargetPosition=object_position
    )

    p.stepSimulation() # by default, p.stepSimulation() advances the physics engine by one simulation step.
    # by default, PyBullet uses a fixed time step of 1/240 seconds (240Hz)
    # if you want the simulation to run in real time, you should also wait 1/240 seconds before calling p.stepSimulation again, otherwise it runs faster than real time.
    # but if computations take more than 1/240 seconds per step, then the simulation will already be running slower than real time.
    # so find the elapsed time and only sleep if necessary. 
    elapsed_time = time.time() - start_time

    sleep_time = max(0, target_dt - elapsed_time)
    # if sleep_time == 0:
    #     print(f"running slower than real time! slowdown:  {elapsed_time / target_dt}") # so right now, this simulation is running 6x slower than real time, idk why. 
    time.sleep(sleep_time)
    # time.sleep(1./240.)
    