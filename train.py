import gymnasium as gym
import gym_envs # our custom gym environment package
import numpy as np
import pybullet as p

env = gym.make(
    'gym_envs/RacingEnv-v0',
    inner_track_path = "./tracks/track_basic/inner.obj",
    outer_track_path = "./tracks/track_basic/outer.obj",
    render_mode = "bird-eye",
    # render_mode = None,
    speed_modifier = 50
    )

env = env.unwrapped
observation, info = env.reset()

episode_over = False

global debug_line
debug_line = None


def get_closest_point(centerline, car_position):
    '''
    find the closest point on the centerline to the car's current position.

    returns the closest point (x,y) tuple of the closest p;oint on the centerline
    closest_index: Index of the closest point in the cneterline array. 
    '''

    distances = np.linalg.norm(centerline - np.array(car_position), axis=1)
    closest_index = np.argmin(distances)
    closest_point = centerline[closest_index]
    return closest_point, closest_index

def compute_lateral_error(car_position, closest_point, tangent_vector):
    '''
    compute the lateral error (signed distance) from the car to the centerline

    car_position (x,y): tuple of the car's current position
    closest_point (x,y): tuple of the closest point on the centerline
    tangent_vector (dx, dy): tangent vector of the centerline at the closest point. 
    
    returns:
        lateral_error: Signed distance from the centerline
    '''

    car_to_closest = np.array(car_position) - np.array(closest_point)
    # signed lateral error using cross product
    lateral_error = np.cross(car_to_closest, tangent_vector) / np.linalg.norm(tangent_vector)
    return lateral_error

def compute_heading_error(car_heading, tangent_vector):
    '''
    Compute the heading error between the car's orientation and the track's centerline direction

    car_heading: Current heading of the car (radians)
    tangent_vector: (dx, dy) tangent direction of the track at the closest point.

    Returns: 
        heading_error: Difference in angle (radians)
    '''

    centerline_angle = np.arctan2(tangent_vector[1], tangent_vector[0])
    heading_error = car_heading - centerline_angle
    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
    return heading_error

def compute_steering_angle(lateral_error, heading_error, kp=0.5, kd=1.0, max_steering_angle=np.pi/4, verbose=False):
    '''
    Compute the steering angle based on proportional control

    lateral_error: signed distance to the centerline
    heading_error: Difference in angle between the car's heading and the centerline direction.
    kp: Proportional gain for lateral error.
    kd: Proportional gain for heading error.

    returns: 
        steering_angle: Steering angle command in the range [-1,1]
    '''

    if verbose: 
        print(f"kp: {kp}, kd: {kd}")
        print("kp * lateral_error: ", kp*lateral_error)
        print("kd * heading_error: ", kd*heading_error)

    # steering_angle = -kp * lateral_error + kd * heading_error
    # steering_angle = -kp * lateral_error - kd * heading_error
    steering_angle = kp * lateral_error - kd * heading_error
    steering_angle = np.clip(steering_angle, -np.pi/4, np.pi/4)  # Clip to realistic steering limits

    steering_normalized = steering_angle / max_steering_angle
    return steering_normalized

def compute_throttle(heading_error, lateral_error, heading_weight = 0.4, lateral_weight = 0.6, max_safe_lateral_error = 1.0, max_speed=1.0, min_speed=0.2):
    """
    Compute the throttle value based on heading error.
    
    heading_error: Difference in angle between car heading and centerline direction.
    lateral_error: the perpendicular distance from the car to the closest point on the centerline.
    max_safe_lateral_error: the largest "safe" lateral error in meters.
    max_speed: Maximum allowed speed.
    min_speed: Minimum allowed speed when turning.
    heading_weight: Weight given to heading factor
    lateral_weaight: Weight given to the lateral factor (heading and lateral weights should sum to 1.)
    
    Returns:
        throttle: Throttle value (0 to 1).
    """

    lateral_factor = min(abs(lateral_error) / max_safe_lateral_error, 1.0) # assume 
    heading_factor = abs(heading_error) / (np.pi / 4)

    combined_factor = heading_weight * heading_factor + lateral_weight * lateral_factor

    # print("combined factor: ", combined_factor)

    throttle = max_speed * (1 - combined_factor)
    return max(min_speed, throttle)  # Ensure minimum speed

def proportional_controller(car_position, car_heading, centerline, kp=0.5, kd=1.0, verbose=False):
    """
    Compute steering and throttle using a proportional controller.
    
    car_position: (x, y) tuple of the car's position.
    car_heading: Car's current heading (radians).
    centerline: Nx2 array of (x, y) coordinates representing the track centerline.
    
    Returns:
        steering_angle: Steering angle command.
        throttle: Throttle command.
    """
    closest_point, closest_index = get_closest_point(centerline, car_position)
    
    # Compute tangent vector (direction of centerline)
    if closest_index < len(centerline) - 1:
        next_point = centerline[closest_index + 1]
    else:
        next_point = centerline[closest_index - 1]

    if verbose: print(f"closest_point: {closest_point}, closest_index: {closest_index}")
    if verbose: print(f"next point: {next_point}")

    zeros_column = np.zeros((1))
    # start = closest_point.extend(0)
    start = np.hstack((closest_point, zeros_column))
    # end = next_point.extend(0)
    end = np.hstack((next_point, zeros_column))
    global debug_line
    if debug_line:
        p.removeUserDebugItem(debug_line)
    debug_line = p.addUserDebugLine(start, end, lineColorRGB=[1,0,0], lineWidth = 2)

    #TODO make this disappear
    
    tangent_vector = np.array(next_point) - np.array(closest_point)
    if verbose: print("tangent_vector: ", tangent_vector)
    # Compute lateral and heading errors
    lateral_error = compute_lateral_error(car_position, closest_point, tangent_vector)
    if verbose: print("lateral error: ", lateral_error)
    heading_error = compute_heading_error(car_heading, tangent_vector)
    if verbose: print("heading error: ", heading_error)
    # Compute control outputs
    steering_angle = compute_steering_angle(lateral_error, heading_error, kp, kd, verbose=verbose)
    if verbose: print("steering_angle: ", steering_angle)
    throttle = compute_throttle(heading_error, lateral_error)
    if verbose: print("throttle: ", throttle)
    
    return steering_angle, throttle


#TODO could also try Pure Pursuit Controller, or Model Preddictive Control.



while not episode_over:
    # action = env.action_space.sample() # replace this with an agent policy that uses the observation and info.
    
    centerline = env.centerline
    position, o = env.get_position_and_orientation()
    # print("position: ", position)
    # print("orientation: ", o)
    car_position = position[:2] # extract the (x,y)
    # extract the yaw from the quaternion
    car_heading = np.arctan2(2*(o[3]*o[2] + o[0] * o[1]), 1 - 2*(o[1]**2 + o[2] ** 2))
    # print("heading: ", car_heading)
    verbose = False
    kp = 1.0
    kd = 0.5
    steering, throttle = proportional_controller(car_position, car_heading, centerline, kp, kd, verbose=verbose)

    # print(f"steering_angle: {steering}, throttle: {throttle}")

    # put together the action. 
    action = [throttle, steering]

    # print("action: ", action)
    observation, reward, terminated, truncated, info = env.step(action)

env.close()
