#!/usr/bin/env python3

################################################################################

# Copyright (c) 2025, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy # Quality of Service (tune communication between nodes)
import tf2_ros # ROS bindings for tf2 library to handle transforms
from std_msgs.msg import Int32, Float32, Header, String, ColorRGBA # Int32, Float32, Header, String message classes
from geometry_msgs.msg import Point, TransformStamped # Point and TransformStamped message classes
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes
from visualization_msgs.msg import Marker, MarkerArray # Marker types for RViz visualization
from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation
from threading import Thread # Thread-based parallelism
import json # JSON serialization for network messages
import zmq # ZeroMQ for inter-container communication
import cv2 # OpenCV for fast image encoding

# Python module imports
from cv_bridge import CvBridge # ROS bridge for opencv library to handle images
import gevent # Coroutine-based concurrency library
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from gevent.threadpool import ThreadPool # Real OS thread pool (doesn't block gevent event loop)
from geventwebsocket.handler import WebSocketHandler # Handler for WebSocket messages and lifecycle events
import socketio # Socket.IO realtime client and server
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
from io import BytesIO # Manipulate bytes data in memory
from PIL import Image # Python Imaging Library's (PIL's) Image module
import gzip # Inbuilt module to compress and decompress data and files
import autodrive_roboracer.config as config # AutoDRIVE Ecosystem ROS 2 configuration for RoboRacer vehicle

################################################################################

# AutoDRIVE class
class AutoDRIVE:
    def __init__(self):
        # Vehicle data
        self.id                       = 1
        self.throttle                 = 0
        self.steering                 = 0
        self.speed                    = 0
        self.encoder_angles           = np.zeros(2, dtype=float)
        self.position                 = np.zeros(3, dtype=float)
        self.orientation_quaternion   = np.zeros(4, dtype=float)
        self.angular_velocity         = np.zeros(3, dtype=float)
        self.linear_acceleration      = np.zeros(3, dtype=float)
        self.lidar_scan_rate          = 40
        self.lidar_range_array        = np.zeros(1080, dtype=float)
        self.lidar_intensity_array    = np.asarray([])
        self.front_camera_image       = np.zeros((192, 108, 3), dtype=np.uint8)
        self.depth_camera_image       = np.zeros((480, 540, 3), dtype=np.uint8)
        # Race data
        self.lap_count       = 0
        self.lap_time        = 0
        self.last_lap_time   = 0
        self.best_lap_time   = 0
        self.collision_count = 0
        # Vehicle commands
        self.throttle_command = 0.0 # [-1, 1]
        self.steering_command = 0.0 # [-1, 1]
        # Simulation commands
        self.reset_command = False # True or False

################################################################################

# Global declarations
global autodrive_bridge, cv_bridge, publishers, transform_broadcaster, zmq_publisher
autodrive = AutoDRIVE()
zmq_publisher = None
simulator_connected = False  # Track simulator connection state

# ZMQ statistics
zmq_stats = {
    'published': 0,
    'dropped': 0,
    'last_report': 0
}

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_int_msg(i, val):
    i.data = int(val)
    return i

def create_float_msg(f, val):
    f.data = float(val)
    return f

def create_joint_state_msg(js, joint_angle, joint_name, frame_id):
    js.header = Header()
    js.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    js.header.frame_id = frame_id
    js.name = [joint_name]
    js.position = [joint_angle]
    js.velocity = []
    js.effort = []
    return js

def create_point_msg(p, position):
    p.x = position[0]
    p.y = position[1]
    p.z = position[2]
    return p

def create_imu_msg(imu, orientation_quaternion, angular_velocity, linear_acceleration):
    imu.header = Header()
    imu.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    imu.header.frame_id = 'imu'
    imu.orientation.x = orientation_quaternion[0]
    imu.orientation.y = orientation_quaternion[1]
    imu.orientation.z = orientation_quaternion[2]
    imu.orientation.w = orientation_quaternion[3]
    imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.angular_velocity.x = angular_velocity[0]
    imu.angular_velocity.y = angular_velocity[1]
    imu.angular_velocity.z = angular_velocity[2]
    imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.linear_acceleration.x = linear_acceleration[0]
    imu.linear_acceleration.y = linear_acceleration[1]
    imu.linear_acceleration.z = linear_acceleration[2]
    imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    return imu

def create_laserscan_msg(ls, lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    ls.header = Header()
    ls.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    ls.header.frame_id = 'lidar'
    ls.angle_min = -2.35619 # Minimum angle of laser scan (0 degrees)
    ls.angle_max = 2.35619 # Maximum angle of laser scan (270 degrees)
    ls.angle_increment = 0.004363323 # Angular resolution of laser scan (0.25 degree)
    ls.time_increment = (1/lidar_scan_rate)/1080 # Time required to scan 1 degree
    ls.scan_time = 1/lidar_scan_rate # Time required to complete a scan
    ls.range_min = 0.06 # Minimum sensor range (in meters)
    ls.range_max = 10.0 # Maximum sensor range (in meters)
    ls.ranges = lidar_range_array
    ls.intensities = lidar_intensity_array
    return ls

def create_image_msg(img, frame_id):
    img = cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
    img.header = Header()
    img.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    img.header.frame_id = frame_id
    return img

def create_tf_msg(child_frame_id, parent_frame_id, position_tf, orientation_tf):
    tf = TransformStamped()
    tf.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    tf.header.frame_id = parent_frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = position_tf[0] # Pos X
    tf.transform.translation.y = position_tf[1] # Pos Y
    tf.transform.translation.z = position_tf[2] # Pos Z
    tf.transform.rotation.x = orientation_tf[0] # Quat X
    tf.transform.rotation.y = orientation_tf[1] # Quat Y
    tf.transform.rotation.z = orientation_tf[2] # Quat Z
    tf.transform.rotation.w = orientation_tf[3] # Quat W
    return tf

def broadcast_transforms(tf_broadcaster, autodrive):
    tf_list = []
    tf_list.append(create_tf_msg("roboracer_1", "world", autodrive.position, autodrive.orientation_quaternion)) # Vehicle frame defined at center of rear axle
    tf_list.append(create_tf_msg("left_encoder", "roboracer_1", np.asarray([0.0, 0.12, 0.0]), quaternion_from_euler(0.0, 120*autodrive.encoder_angles[0]%6.283, 0.0)))
    tf_list.append(create_tf_msg("right_encoder", "roboracer_1", np.asarray([0.0, -0.12, 0.0]), quaternion_from_euler(0.0, 120*autodrive.encoder_angles[1]%6.283, 0.0)))
    tf_list.append(create_tf_msg("ips", "roboracer_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0])))
    tf_list.append(create_tf_msg("imu", "roboracer_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0])))
    tf_list.append(create_tf_msg("lidar", "roboracer_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0])))
    tf_list.append(create_tf_msg("front_camera", "roboracer_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947])))
    tf_list.append(create_tf_msg("depth_camera", "roboracer_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947])))
    tf_list.append(create_tf_msg("front_left_wheel", "roboracer_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(autodrive.steering))/(2*0.141537-2*0.0765*np.tan(autodrive.steering))))))
    tf_list.append(create_tf_msg("front_right_wheel", "roboracer_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(autodrive.steering))/(2*0.141537+2*0.0765*np.tan(autodrive.steering))))))
    tf_list.append(create_tf_msg("rear_left_wheel", "roboracer_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, autodrive.encoder_angles[0]%6.283, 0.0)))
    tf_list.append(create_tf_msg("rear_right_wheel", "roboracer_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, autodrive.encoder_angles[1]%6.283, 0.0)))
    tf_broadcaster.sendTransform(tf_list)


#########################################################
# ROS 2 MESSAGE DEFINITIONS
#########################################################

msg_int32 = Int32()
msg_float32 = Float32()
msg_string = String()
msg_jointstate = JointState()
msg_point = Point()
msg_imu = Imu()
msg_laserscan = LaserScan()
msg_transform = TransformStamped()

#########################################################
# ROS 2 PUBLISHER FUNCTIONS
#########################################################

# VEHICLE DATA PUBLISHER FUNCTIONS

def publish_actuator_feedbacks(throttle, steering):
    publishers['pub_throttle'].publish(create_float_msg(msg_float32, throttle))
    publishers['pub_steering'].publish(create_float_msg(msg_float32, steering))

def publish_speed_data(speed):
    publishers['pub_speed'].publish(create_float_msg(msg_float32, speed))

def publish_encoder_data(encoder_angles):
    publishers['pub_left_encoder'].publish(create_joint_state_msg(msg_jointstate, encoder_angles[0], "left_encoder", "left_encoder"))
    publishers['pub_right_encoder'].publish(create_joint_state_msg(msg_jointstate, encoder_angles[1], "right_encoder", "right_encoder"))

def publish_ips_data(position):
    publishers['pub_ips'].publish(create_point_msg(msg_point, position))

def publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration):
    publishers['pub_imu'].publish(create_imu_msg(msg_imu, orientation_quaternion, angular_velocity, linear_acceleration))

def publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    publishers['pub_lidar'].publish(create_laserscan_msg(msg_laserscan, lidar_scan_rate, lidar_range_array.tolist(), lidar_intensity_array.tolist()))

def publish_camera_images(front_camera_image):
    publishers['pub_front_camera'].publish(create_image_msg(front_camera_image, "front_camera"))

# Depth Camera
def publish_depth_images(depth_camera_image):
    publishers['pub_depth_camera'].publish(create_image_msg(depth_camera_image, "depth_camera"))

def publish_lap_count_data(lap_count):
    publishers['pub_lap_count'].publish(create_int_msg(msg_int32, lap_count))

def publish_lap_time_data(lap_time):
    publishers['pub_lap_time'].publish(create_float_msg(msg_float32, lap_time))

def publish_last_lap_time_data(last_lap_time):
    publishers['pub_last_lap_time'].publish(create_float_msg(msg_float32, last_lap_time))

def publish_best_lap_time_data(best_lap_time):
    publishers['pub_best_lap_time'].publish(create_float_msg(msg_float32, best_lap_time))

def publish_collision_count_data(collision_count):
    publishers['pub_collision_count'].publish(create_int_msg(msg_int32, collision_count))

# SOFTWARE STACK OUTPUT PUBLISHERS (ZMQ -> ROS)
def publish_stack_pose_data(payload_json: str):
    msg_string.data = payload_json
    publishers['pub_stack_pose'].publish(msg_string)

def publish_stack_segmentation_data(payload_json: str):
    try:
        p = json.loads(payload_json)
        raw = base64.b64decode(p['mask'])
        shape = tuple(p['shape'])
        mask = np.frombuffer(raw, dtype=np.uint8).reshape(shape)
        # Normalize sparse-value masks (e.g. depth cast to uint8 gives [0,10])
        # so they fill the [0, 255] range and are visible in RViz/rqt_image_view.
        if mask.ndim == 2:
            mx = int(mask.max())
            if 0 < mx < 64:
                mask = (mask.astype(np.float32) * (255.0 / mx)).astype(np.uint8)
            img_msg = cv_bridge.cv2_to_imgmsg(mask, encoding='mono8')
        else:
            img_msg = cv_bridge.cv2_to_imgmsg(mask, encoding='rgb8')
        img_msg.header.stamp = autodrive_bridge.get_clock().now().to_msg()
        img_msg.header.frame_id = 'front_camera'
        publishers['pub_stack_segmentation'].publish(img_msg)
    except Exception:
        import traceback; traceback.print_exc()

def publish_stack_boundary_data(payload_json: str):
    try:
        p = json.loads(payload_json)
        now = autodrive_bridge.get_clock().now().to_msg()
        marker_array = MarkerArray()
        lines = [
            ('outer_line', 0, (1.0, 0.0, 0.0)),  # red
            ('inner_line', 1, (0.0, 0.0, 1.0)),  # blue
        ]
        for key, mid, (r, g, b) in lines:
            if key not in p or not p[key]:
                continue
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'world'
            m.ns = 'boundary'
            m.id = mid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03  # line width in metres
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
            for pt in p[key]:
                gp = Point()
                gp.x = float(pt[0]); gp.y = float(pt[1]); gp.z = 0.0
                m.points.append(gp)
            marker_array.markers.append(m)
        publishers['pub_stack_boundary'].publish(marker_array)
    except Exception:
        import traceback; traceback.print_exc()

#########################################################
# ZEROMQ NETWORK PUBLISHER FUNCTIONS
#########################################################

def publish_to_network(autodrive):
    """Publish sensor data to pilot_core via ZeroMQ."""
    global zmq_publisher, zmq_stats, simulator_connected
    if zmq_publisher is None or not simulator_connected:
        return
    
    try:
        # Forward raw JPEG bytes directly — no decode/re-encode round-trip
        # If async capture hasn't completed yet, send None — IMU/pose still go through
        front_jpeg = getattr(autodrive, '_front_jpeg', None)
        front_cam_payload = {
            'data': base64.b64encode(front_jpeg).decode('utf-8'),
            'encoding': 'jpeg',
        } if front_jpeg is not None else None

        # Depth: forward raw bytes from simulator (JPEG), skip PNG re-encode (~30ms saved)
        depth_jpeg = getattr(autodrive, '_depth_jpeg', None)
        if depth_jpeg is not None:
            depth_cam_payload = {
                'data': base64.b64encode(depth_jpeg).decode('utf-8'),
                'encoding': 'jpeg',
            }
        else:
            depth_cam_payload = None

        # Build message payload
        payload = {
            'timestamp': autodrive_bridge.get_clock().now().to_msg().sec +
                        autodrive_bridge.get_clock().now().to_msg().nanosec * 1e-9,
            'throttle': float(autodrive.throttle),
            'steering': float(autodrive.steering),
            'speed': float(autodrive.speed),
            'position': autodrive.position.tolist(),
            'orientation_quat': autodrive.orientation_quaternion.tolist(),
            'angular_velocity': autodrive.angular_velocity.tolist(),
            'linear_acceleration': autodrive.linear_acceleration.tolist(),
            'front_camera': front_cam_payload,
            'depth_camera': depth_cam_payload,
        }
        
        # Serialize and send (non-blocking)
        msg_json = json.dumps(payload)
        zmq_publisher.send_string(msg_json, zmq.NOBLOCK)
        
        # Update stats
        zmq_stats['published'] += 1
        
    except zmq.Again:
        # Buffer full, frame dropped
        zmq_stats['dropped'] += 1
        
        # Report every 100 drops
        if zmq_stats['dropped'] % 100 == 0:
            drop_rate = zmq_stats['dropped'] / (zmq_stats['published'] + zmq_stats['dropped']) * 100
            print(f"⚠ ZMQ: Dropped {zmq_stats['dropped']} frames ({drop_rate:.1f}% drop rate)")
            
    except Exception as e:
        print(f"ZMQ publish error: {e}")

#########################################################
# ROS 2 SUBSCRIBER CALLBACKS
#########################################################

# VEHICLE DATA SUBSCRIBER CALLBACKS

def callback_throttle_command(throttle_command_msg):
    global autodrive
    autodrive.throttle_command = float(np.round(throttle_command_msg.data, 3))

def callback_steering_command(steering_command_msg):
    global autodrive
    autodrive.steering_command = float(np.round(steering_command_msg.data, 3))

def callback_reset_command(reset_command_msg):
    global autodrive
    autodrive.reset_command = reset_command_msg.data

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    global simulator_connected
    simulator_connected = True
    print("Simulator connected!")

# Registering "disconnect" event handler for the server
@sio.on('disconnect')
def disconnect(sid):
    global simulator_connected, autodrive
    simulator_connected = False
    # Reset autodrive state to prevent stale data publishing
    autodrive = AutoDRIVE()
    print("Simulator disconnected!")

def _process_frame(data):
    """Process one simulator frame in a thread pool worker.

    Runs gzip, all ROS2 publishes, and ZMQ publish in a real OS thread so the
    gevent event loop is never blocked by DDS or C-level calls.

    Camera images are kept as raw JPEG bytes (no PIL decode) and forwarded
    directly to ZMQ to avoid a decode→re-encode round-trip.  ROS2 camera image
    topics are skipped since pilot_core uses ZMQ, not ROS2, for sensor data.
    """
    global autodrive, autodrive_bridge, cv_bridge, publishers, transform_broadcaster
    try:
        # Actuator feedbacks
        autodrive.throttle = float(data["V1 Throttle"])
        autodrive.steering = float(data["V1 Steering"])
        # Speed
        autodrive.speed = float(data["V1 Speed"])
        # Wheel encoders
        autodrive.encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        # IPS
        autodrive.position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        # IMU
        autodrive.orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        autodrive.angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        autodrive.linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        # LIDAR
        autodrive.lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])
        autodrive.lidar_range_array = np.fromstring(gzip.decompress(base64.b64decode(data["V1 LIDAR Range Array"])).decode('utf-8'), sep='\n')
        # Cameras — store raw JPEG bytes, no PIL decode needed for ZMQ path
        if "V1 Front Camera Image" in data:
            autodrive._front_jpeg = base64.b64decode(data["V1 Front Camera Image"])
        autodrive._depth_jpeg = base64.b64decode(data["V1 Depth Camera Image"]) if "V1 Depth Camera Image" in data else None
        # Lap data
        autodrive.lap_count = int(float(data["V1 Lap Count"]))
        autodrive.lap_time = float(data["V1 Lap Time"])
        autodrive.last_lap_time = float(data["V1 Last Lap Time"])
        autodrive.best_lap_time = float(data["V1 Best Lap Time"])
        autodrive.collision_count = int(float(data["V1 Collisions"]))

        # ROS2 publishing (scalar topics only — image topics skipped, ~100ms saved)
        # publish_actuator_feedbacks(autodrive.throttle, autodrive.steering)
        # publish_speed_data(autodrive.speed)
        # publish_encoder_data(autodrive.encoder_angles)
        # publish_ips_data(autodrive.position)
        # publish_imu_data(autodrive.orientation_quaternion, autodrive.angular_velocity, autodrive.linear_acceleration)
        # broadcast_transforms(transform_broadcaster, autodrive)
        # publish_lidar_scan(autodrive.lidar_scan_rate, autodrive.lidar_range_array, autodrive.lidar_intensity_array)
        # publish_lap_count_data(autodrive.lap_count)
        # publish_lap_time_data(autodrive.lap_time)
        # publish_last_lap_time_data(autodrive.last_lap_time)
        # publish_best_lap_time_data(autodrive.best_lap_time)
        # publish_collision_count_data(autodrive.collision_count)

        # ZMQ publish to pilot_core
        publish_to_network(autodrive)
    except Exception:
        import traceback
        traceback.print_exc()


# One worker thread: frames processed serially so the ZMQ socket is never
# accessed from two threads simultaneously.
_frame_pool = ThreadPool(1)


# Bridge FPS counter
_bridge_frame_count = 0
_bridge_last_report = 0.0

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    global autodrive, _bridge_frame_count, _bridge_last_report
    if data:
        import time as _time
        # Respond to simulator IMMEDIATELY — never blocked by processing
        sio.emit('Bridge', data={
            'V1 Throttle': str(autodrive.throttle_command),
            'V1 Steering': str(autodrive.steering_command),
            'V1 Reset': str(autodrive.reset_command),
            'Reset': str(autodrive.reset_command)
        })
        # FPS counter — prints once per second
        _bridge_frame_count += 1
        _now = _time.time()
        if _now - _bridge_last_report >= 1.0:
            print(f"[Bridge] Simulator FPS: {_bridge_frame_count / (_now - _bridge_last_report):.1f}" if _bridge_last_report else "")
            _bridge_frame_count = 0
            _bridge_last_report = _now
        # All heavy work runs in a real OS thread, not in the gevent event loop
        _frame_pool.spawn(_process_frame, data)

#########################################################
# ZEROMQ COMMAND RECEIVER THREAD
#########################################################

# Shutdown event for graceful termination
import threading
zmq_shutdown_event = threading.Event()

def zmq_command_receiver_thread(zmq_subscriber):
    """Background thread to receive ZMQ payloads from software stack."""
    global autodrive
    print("ZMQ receiver thread started (stack outputs)")
    
    consecutive_errors = 0
    MAX_CONSECUTIVE_ERRORS = 10
    
    while not zmq_shutdown_event.is_set():
        try:
            # Non-blocking receive with timeout (100ms)
            msg_text = zmq_subscriber.recv_string(zmq.NOBLOCK)
            # print(f"ZMQ RAW: {msg_text}")

            # Support topic-prefixed payloads: "topic {json}"
            topic = None
            payload_text = msg_text
            if " " in msg_text:
                candidate_topic, candidate_payload = msg_text.split(" ", 1)
                if candidate_topic in {"control", "telemetry", "pose", "segmentation", "map", "boundary"}:
                    topic = candidate_topic
                    payload_text = candidate_payload

            # Robust JSON parsing (ignore trailing data)
            decoder = json.JSONDecoder()
            try:
                payload, end = decoder.raw_decode(payload_text)
                if payload_text[end:].strip():
                    # Ignore trailing garbage after the first JSON object
                    payload_text = payload_text[:end]
            except json.JSONDecodeError:
                # Fallback: if message has a topic but failed parse, try parsing after first space
                if topic is None and " " in msg_text:
                    _, fallback_payload = msg_text.split(" ", 1)
                    payload, end = decoder.raw_decode(fallback_payload)
                    payload_text = fallback_payload[:end] if fallback_payload[end:].strip() else fallback_payload
                else:
                    raise

            if topic == "segmentation":
                publish_stack_segmentation_data(payload_text)

            elif topic == "telemetry" or topic == "pose":
                publish_stack_pose_data(payload_text)

            elif topic == "map" or topic == "boundary":
                publish_stack_boundary_data(payload_text)

            elif topic is None or topic == "control":
                # Update command state — bridge() emits these on the next simulator frame
                if 'throttle' in payload:
                    autodrive.throttle_command = float(payload['throttle'])
                if 'steering' in payload:
                    autodrive.steering_command = float(payload['steering'])
                if 'reset' in payload:
                    autodrive.reset_command = bool(payload['reset'])

            else:
                print(f"ZMQ: Unhandled topic '{topic}'")

            # Reset error counter on success
            consecutive_errors = 0
                
        except zmq.Again:
            # No message available, sleep briefly and continue
            zmq_shutdown_event.wait(0.01)  # 10ms sleep, but can be interrupted
            continue
            
        except json.JSONDecodeError as e:
            print(f"ZMQ: Invalid JSON received: {e}")
            consecutive_errors += 1
            
        except (ValueError, KeyError) as e:
            print(f"ZMQ: Invalid command data: {e}")
            consecutive_errors += 1
            
        except zmq.ZMQError as e:
            print(f"ZMQ: Socket error: {e}")
            consecutive_errors += 1
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                print("ZMQ: Too many consecutive errors, terminating receiver thread")
                break
            zmq_shutdown_event.wait(0.1)  # Back off on errors
            
        except Exception as e:
            print(f"ZMQ: Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            consecutive_errors += 1
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                print("ZMQ: Too many consecutive errors, terminating receiver thread")
                break
            zmq_shutdown_event.wait(0.1)
    
    print("ZMQ command receiver thread terminated")

#########################################################
# AUTODRIVE ROS 2 BRIDGE INFRASTRUCTURE
#########################################################

# def timer_callback():
#     global autodrive
#     # Actuator feedbacks
#     publish_actuator_feedbacks(autodrive.throttle, autodrive.steering)
#     # Speed
#     publish_speed_data(autodrive.speed)
#     # Wheel encoders
#     publish_encoder_data(autodrive.encoder_angles)
#     # IPS
#     publish_ips_data(autodrive.position)
#     # IMU
#     publish_imu_data(autodrive.orientation_quaternion, autodrive.angular_velocity, autodrive.linear_acceleration)
#     # Coordinate transforms
#     broadcast_transform(msg_transform, transform_broadcaster, "roboracer_1", "world", autodrive.position, autodrive.orientation_quaternion) # Vehicle frame defined at center of rear axle
#     broadcast_transform(msg_transform, transform_broadcaster, "left_encoder", "roboracer_1", np.asarray([0.0, 0.12, 0.0]), quaternion_from_euler(0.0, 120*autodrive.encoder_angles[0]%6.283, 0.0))
#     broadcast_transform(msg_transform, transform_broadcaster, "right_encoder", "roboracer_1", np.asarray([0.0, -0.12, 0.0]), quaternion_from_euler(0.0, 120*autodrive.encoder_angles[1]%6.283, 0.0))
#     broadcast_transform(msg_transform, transform_broadcaster, "ips", "roboracer_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
#     broadcast_transform(msg_transform, transform_broadcaster, "imu", "roboracer_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
#     broadcast_transform(msg_transform, transform_broadcaster, "lidar", "roboracer_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]))
#     broadcast_transform(msg_transform, transform_broadcaster, "front_camera", "roboracer_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]))
#     broadcast_transform(msg_transform, transform_broadcaster, "front_left_wheel", "roboracer_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(autodrive.steering))/(2*0.141537-2*0.0765*np.tan(autodrive.steering)))))
#     broadcast_transform(msg_transform, transform_broadcaster, "front_right_wheel", "roboracer_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(autodrive.steering))/(2*0.141537+2*0.0765*np.tan(autodrive.steering)))))
#     broadcast_transform(msg_transform, transform_broadcaster, "rear_left_wheel", "roboracer_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, autodrive.encoder_angles[0]%6.283, 0.0))
#     broadcast_transform(msg_transform, transform_broadcaster, "rear_right_wheel", "roboracer_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, autodrive.encoder_angles[1]%6.283, 0.0))
#     # LIDAR
#     publish_lidar_scan(autodrive.lidar_scan_rate, autodrive.lidar_range_array, autodrive.lidar_intensity_array)
#     # Cameras
#     publish_camera_images(autodrive.front_camera_image)
#     # Lap data
#     publish_lap_count_data(autodrive.lap_count)
#     publish_lap_time_data(autodrive.lap_time)
#     publish_last_lap_time_data(autodrive.last_lap_time)
#     publish_best_lap_time_data(autodrive.best_lap_time)
#     publish_collision_count_data(autodrive.collision_count)

def main():
    # Global declarations
    global autodrive, autodrive_bridge, cv_bridge, publishers, transform_broadcaster, zmq_publisher

    # ZeroMQ infrastructure
    zmq_context = zmq.Context()
    
    # Publisher socket for sensor data (pilot_core subscribes to this)
    zmq_publisher = zmq_context.socket(zmq.PUB)
    zmq_publisher.setsockopt(zmq.SNDHWM, 10)  # Only buffer last 10 frames (images are large!)
    zmq_publisher.setsockopt(zmq.LINGER, 0)   # Don't wait on close
    zmq_publisher.bind("tcp://*:5555")
    print("ZMQ Publisher bound to port 5555 (sensor data, HWM=10)")
    
    # Subscriber socket for control commands (pilot_core publishes to this)
    zmq_subscriber = zmq_context.socket(zmq.SUB)
    zmq_subscriber.setsockopt(zmq.RCVHWM, 50)  # Buffer commands + telemetry from both ports
    zmq_subscriber.setsockopt(zmq.LINGER, 0)   # Don't wait on close
    # No CONFLATE — allows segmentation/boundary messages to queue alongside control commands
    zmq_subscriber.connect("tcp://localhost:5556")  # Control commands (ActuatorWorker)
    zmq_subscriber.connect("tcp://localhost:5557")  # Telemetry (ZMQPublishWorker: segmentation, boundaries)
    zmq_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
    print("ZMQ Subscriber connected to stack:5556 (control) and stack:5557 (telemetry)")
    
    # Start ZMQ command receiver thread
    zmq_thread = Thread(target=zmq_command_receiver_thread, args=(zmq_subscriber,), daemon=True)
    zmq_thread.start()

    # ROS 2 infrastructure
    rclpy.init() # Initialize ROS 2 communication for this context
    autodrive_bridge = rclpy.create_node('autodrive_bridge') # Create ROS 2 node
    qos_profile = QoSProfile( # Ouality of Service profile
        durability=QoSDurabilityPolicy.VOLATILE, # Volatile durability with no attempt made to persist samples
        reliability=QoSReliabilityPolicy.RELIABLE, # Reliable (not best effort) communication to guarantee that samples are delivered
        history=QoSHistoryPolicy.KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
    cv_bridge = CvBridge() # ROS bridge object for OpenCV library to handle image data
    transform_broadcaster = tf2_ros.TransformBroadcaster(autodrive_bridge) # Initialize transform broadcaster
    publishers = {e.name: autodrive_bridge.create_publisher(e.type, e.topic, qos_profile)
                  for e in config.pub_sub_dict.publishers} # Publishers
    callbacks = {
        '/autodrive/roboracer_1/throttle_command': callback_throttle_command,
        '/autodrive/roboracer_1/steering_command': callback_steering_command,
        '/autodrive/reset_command': callback_reset_command
    } # Subscriber callback functions
    [autodrive_bridge.create_subscription(e.type, e.topic, callbacks[e.topic], qos_profile) for e in config.pub_sub_dict.subscribers] # Subscribers

    # timer_period = 0.025 # Timer period in seconds
    # autodrive_bridge.create_timer(timer_period, timer_callback)

    # If num_threads is not specified then num_threads will be multiprocessing.cpu_count() if it is implemented
    # Otherwise it will use a single thread
    executor = rclpy.executors.MultiThreadedExecutor() # Create an executor to control which thread callbacks get executed in
    executor.add_node(autodrive_bridge) # Add node whose callbacks should be managed by this executor

    process = Thread(target=executor.spin, daemon=True) # Runs callbacks in the thread
    process.start() # Activate the thread as a demon (background process) and prompt it to the target function (spin the executor)

    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    
    try:
        pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as a gevent WSGI server
    except KeyboardInterrupt:
        print("\nShutting down autodrive_bridge...")
    finally:
        # Signal ZMQ thread to stop
        zmq_shutdown_event.set()
        zmq_thread.join(timeout=2.0)  # Wait up to 2 seconds for thread to finish
        
        # Cleanup ZMQ
        if zmq_publisher:
            zmq_publisher.close()
        if zmq_subscriber:
            zmq_subscriber.close()
        if zmq_context:
            zmq_context.term()
        
        # Cleanup ROS2
        executor.shutdown() # Executor shutdown
        autodrive_bridge.destroy_node() # Explicitly destroy the node
        rclpy.shutdown() # Shutdown this context
        
        print("Autodrive bridge terminated cleanly")

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE ROS 2 bridge

