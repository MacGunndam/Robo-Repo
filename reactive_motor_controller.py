# Team Grinch Midterm
# Motor controll and reading LIDAR data
# Goal: Start car and stop when 20 cm from object
# Members:
#   Bennett Fragomeni
#   Ricardo GonzaleZ
#   Robert Gunn
#   Emerson Scott


# imports
import numpy as np
import math
import sys
import signal
import subprocess

# ros imports
import rclpy
from rclpy.node import Node

# Messages Used
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import CameraMsg



THRESHOLD = 0.5
STOP_THRESHOLD = 0.25
HALLWAY_THRESHOLD = 0.85
TURN_THRESHOLD = HALLWAY_THRESHOLD - 0.1
THROTTLE = 1.0

class LidarSubscriber(Node):

    def __init__(self, motor_pub):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/rplidar_ros/scan', self.listener_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.stopped = 0 # boolean to stop motor
        self.motor_pub : MotorPublisher = motor_pub
        self.current_state = ""


    def listener_callback(self, msg):
        ranges = msg.ranges
        # self.get_logger().info("Got %d range readings" % len(ranges))

        ranges_length = len(ranges)

        ''' 
        translating lidar readings into a compass for the robot, where N=front, 
        E=right, S=behind, W=left. Each lidar reading is roughly 45 degrees apart
                   lidar_N
             lidar_NW | lidar_NE
                  \   |    /
                   \  |   /
        lidar_W --- robot ---- lidar_E
                   /  |   \
                  /   |    \
             lidar_SW | lidar_SE
                   lidar_S
        '''
        lidar_N = ranges[0]
        lidar_NE = ranges[7*ranges_length // 8]
        lidar_E = ranges[3*ranges_length // 4]
        lidar_SE = ranges[5*ranges_length // 8]
        lidar_S = ranges[ranges_length // 2]
        lidar_SW = ranges[3*ranges_length // 8]
        lidar_W = ranges[ranges_length // 4]
        lidar_NW = ranges[ranges_length // 8]
        all_compass_scans = [lidar_N, lidar_NE, lidar_E, lidar_SE, lidar_S, lidar_SW, lidar_W, lidar_NW]

        dangerously_close = any(scan < STOP_THRESHOLD for scan in all_compass_scans)

        # if the robot is too close to any object, send a stop message to the motors and set stopped attribute to 1
        if dangerously_close:
            self.get_logger().info("WARNING: Dangerously close!")
            self.motor_pub.stop_motor()
            # self.stopped = 1

        # state when the robot is in the middle of a hallway with no turns coming up and walls on either side
        # in this case, try and keep the robot centered by making small turns depending on proximity to walls
        elif lidar_N >= HALLWAY_THRESHOLD and math.isfinite(lidar_E) and math.isfinite(lidar_W):
            # if the robot is closer to right wall, turn slightly left
            if lidar_E < lidar_W:
                self.motor_pub.change_angle(-0.05)
                self.get_logger().info("turning slightly left")
            # if the robot is closer to left wall, turn slightly right
            elif lidar_E > lidar_W:
                self.motor_pub.change_angle(0.05)
                self.get_logger().info("turning slightly right")

        # state when the robot is approaching a wall directly in front of it
        # in this case, turn the robot right until lidar_N goes back to state 1 (hopefully)
        # this doesn't check that the right is another hallway
        # if going down a corridor with no turns, then this behavior will cause it to u-turn or stop depending on its speed
        elif lidar_N <= TURN_THRESHOLD and lidar_NW <= TURN_THRESHOLD + 0.15:
            self.motor_pub.change_angle(0.15)
            self.get_logger().info("found turn condition. turning right")

        # if none of the other states are active, simply drive forward
        else:
            self.motor_pub.start_motor_forward()
            self.motor_pub.change_angle(0)
            # self.get_logger().info("no other states active. driving forward")
            self.current_state = "Forward"

# class CameraSubscriber(Node):
#     """ Recieves messages from camera """
#     def __init__(self):
#         super().__init__('camera_subscriber')
#         self.subscription = self.create_subscription(CameraMsg, '/camera/scan', self.listener_callback, 10)
        
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         # go through the camera message and filter on red pixels
#         for pixel in msg:
#               break
#         return

class MotorPublisher(Node):
    """ Publishes messages to motor for control """
    def __init__(self):
        super().__init__('motor_publisher')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.current_throttle : float = 0.0
        self.current_angle : float = 0.0

    def publish_message(self):
        # set the message
        wheel_msg = ServoCtrlMsg()
        wheel_msg.throttle = self.current_throttle
        wheel_msg.angle = self.current_angle
        # Publish the message
        self.wheel_publisher.publish(wheel_msg)

    def start_motor_forward(self):
        # set shared variables
        self.current_throttle = THROTTLE
        self.publish_message()

    def start_motor_backward(self):
        # set shared variables
        self.current_throttle = -THROTTLE
        self.publish_message()   

    def stop_motor(self):
        # set shared variables
        self.current_throttle = 0.0
        self.current_angle = 0.0
        self.publish_message()

    def change_angle(self, angle):
        # set shared angle
        self.current_angle = self.current_angle + angle
        self.publish_message()        


def publish_telemetry(lidar : LidarSubscriber, motor : MotorPublisher):
    """ Publishes telemetry to console """
    print("Motor Status | state %s | throttle %0.1f | angle %0.2f |" % \
          (lidar.current_state, motor.current_throttle, motor.current_angle))
    return

def main(args=None):
    count = 0
    rclpy.init(args=args)
    print("Starting Program")

    # Setting up out Pubs / Subs
    motor_publisher = MotorPublisher()
    lidar_subscriber = LidarSubscriber(motor_publisher)
    # camera_subscriber = CameraSubscriber()

    print("Spinning")


    while(1):
        rclpy.spin_once(lidar_subscriber)
        if lidar_subscriber.stopped == 1:
            print("stop condition met")
            break
        if count % 5000:
            count = 0
            publish_telemetry(lidar_subscriber, motor_publisher)
        count+=1

    print("Stopping Motor")
    motor_publisher.stop_motor()

# signal handler to stop the motor on SIG INT (Ctrl-C)
def signal_handler(sig, frame):
    print("Exiting")
    subprocess.call("ros2 topic pub --once /ctrl_pkg/servo_msg deepracer_interfaces_pkg/msg/ServoCtrlMsg '{throttle: 0.0, angle: 0.0}'", shell=True)
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()
