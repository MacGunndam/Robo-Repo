# Team Grinch Final
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
import time
import psutil

# ros imports
import rclpy
from rclpy.node import Node

# Messages Used
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan, Image

ON = 1
OFF = 0
################################
# Feature Activation
################################
FEATURE_STOP_SIGN = OFF
FEATURE_TELEMETRY = ON
FEATURE_HW_MONITOR = OFF
FEATURE_BACKWARD_DRIVING = OFF


################################
# Constants
################################
# How long to stop at stop sign
STOP_WAIT = 5

SPEED_MULTIPLIER = 1
ANGLE_MULTIPLIER = 1

# THRESHOLD = 0.5
STOP_THRESHOLD = 0.75 # if any lidar beam returns a distance less than this amount, STOP
HALLWAY_THRESHOLD = 2.25 # makes sure both lidar_E and lidar_W are under this distance for hallway state
HALLWAY_END_THRESHOLD = 3.0 # drive while staying centered until lidar_N reaches this distance
TURN_THRESHOLD = 0.95 * HALLWAY_END_THRESHOLD # when the lidar_N see's this distance, turn right
# the turning threshold is slightly lower than the hallway's to let the robot drive forward a bit before turning

THROTTLE = 0.65 * SPEED_MULTIPLIER

class LidarSubscriber(Node):

    def __init__(self, motor_pub, reverse_driving):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/rplidar_ros/scan', self.listener_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.stopped = 0 # boolean to stop motor
        self.motor_pub : MotorPublisher = motor_pub
        self.current_state = ""
        self.reverse_drive = reverse_driving
        # lidar ranges
        self.all_compass_scans = [0, 0, 0, 0, 0, 0, 0, 0]

    def listener_callback(self, msg):
        ranges = msg.ranges
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
        middle = 0
        lidar_N = ranges[middle]
        # lidar_N = (ranges[middle] + ranges[middle + 5] + ranges[ranges_length-6]) / 3
        middle = 7*ranges_length // 8
        lidar_NE = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3
        middle = 3*ranges_length // 4
        lidar_E = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3
        middle = 5*ranges_length // 8
        lidar_SE = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3
        middle = ranges_length // 2
        lidar_S = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3
        middle = 3*ranges_length // 8
        lidar_SW = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3
        middle = ranges_length // 4
        lidar_W = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3
        middle = ranges_length // 8
        lidar_NW = (ranges[middle] + ranges[middle - 5] + ranges[middle + 5]) / 3

        self.all_compass_scans = [lidar_N, lidar_NE, lidar_E, lidar_SE, lidar_S, lidar_SW, lidar_W, lidar_NW]

        dangerously_close = any(scan < STOP_THRESHOLD for scan in self.all_compass_scans)

        # if the robot is too close to any object, send a stop message to the motors and set stopped attribute to 1
        if dangerously_close:
            # self.get_logger().info("WARNING: Dangerously close!")
            self.current_state = "DANGER!"
            self.motor_pub.stop_motor()
            # self.stopped = 1

        # normal driving / correcting
        if self.reverse_drive == 0:
            # state when the robot is approaching a wall directly in front of it
            # in this case, turn the robot right until lidar_N goes back to state 1 (hopefully)
            # this doesn't check that the right is another hallway
            # if going down a corridor with no turns, then this behavior will cause it to u-turn or stop depending on its speed
            if lidar_N <= TURN_THRESHOLD and ((lidar_NE > lidar_N) or (lidar_E > lidar_N)):
                self.motor_pub.current_throttle = 0.8 * THROTTLE
                self.motor_pub.set_angle(-0.6)
                self.get_logger().info("found turn condition. turning right")
                self.current_state = "Right Turn"

            # state when the robot is in the middle of a hallway with no turns coming up and walls on either side
            # in this case, try and keep the robot centered by making small turns depending on proximity to walls
            elif lidar_N >= HALLWAY_END_THRESHOLD and lidar_E <= HALLWAY_THRESHOLD and lidar_W <= HALLWAY_THRESHOLD:
                self.motor_pub.current_throttle = THROTTLE
                # if the robot is closer to right wall, turn slightly left
                if lidar_E < lidar_W:
                    # publish message
                    self.motor_pub.set_angle(0.2 * ANGLE_MULTIPLIER)
                    # self.get_logger().info("turning slightly left")
                    self.current_state = "Slight Left"
                # if the robot is closer to left wall, turn slightly right
                elif lidar_E > lidar_W:
                    # publish message
                    self.motor_pub.set_angle(-0.2 * ANGLE_MULTIPLIER)
                    # self.get_logger().info("turning slightly right")
                    self.current_state = "Slight Right"

            # if none of the other states are active, simply drive forward
            else:
                self.motor_pub.start_motor_forward()
                # self.get_logger().info("no other states active. driving forward")
                self.current_state = "Forward"
        # reverse driving is the same as forward, but with different lidar beams
        else:
            # state when the robot is approaching a wall directly in front of it
            # in this case, turn the robot right until lidar_N goes back to state 1 (hopefully)
            # this doesn't check that the right is another hallway
            # if going down a corridor with no turns, then this behavior will cause it to u-turn or stop depending on its speed
            if lidar_S <= TURN_THRESHOLD and ((lidar_SW > lidar_S) or (lidar_W > lidar_S)):
                self.motor_pub.current_throttle = 0.8 * -THROTTLE
                self.motor_pub.set_angle(0.6)
                self.get_logger().info("found turn condition. turning right")
                self.current_state = "Right Turn"

            # state when the robot is in the middle of a hallway with no turns coming up and walls on either side
            # in this case, try and keep the robot centered by making small turns depending on proximity to walls
            elif lidar_S >= HALLWAY_END_THRESHOLD and lidar_E <= HALLWAY_THRESHOLD and lidar_W <= HALLWAY_THRESHOLD:
                self.motor_pub.current_throttle = -THROTTLE
                # if the robot is closer to right wall, turn slightly left
                if lidar_E < lidar_W:
                    # publish message
                    self.motor_pub.set_angle(-0.2 * ANGLE_MULTIPLIER)
                    # self.get_logger().info("turning slightly left")
                    self.current_state = "Slight Left"
                # if the robot is closer to left wall, turn slightly right
                elif lidar_E > lidar_W:
                    # publish message
                    self.motor_pub.set_angle(0.2 * ANGLE_MULTIPLIER)
                    # self.get_logger().info("turning slightly right")
                    self.current_state = "Slight Right"

            # if none of the other states are active, simply drive forward
            else:
                self.motor_pub.start_motor_forward()
                # self.get_logger().info("no other states active. driving forward")
                self.current_state = "Forward"

class CameraSubscriber(Node):
    """ Recieves messages from camera """
    def __init__(self, motor_publisher):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(Image, '/camera_pkg/display_mjpeg', self.listener_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.num_color = 0
        self.motor : MotorPublisher = motor_publisher
        self.current_state = "None"

    def listener_callback(self, msg : Image):
        if FEATURE_STOP_SIGN == ON:
            # go through the camera message and filter on red pixels
            # data is [b, g, r,   b, g, r,   b, g, r, ...]
            BLUE = 0
            GREEN = 1
            RED = 2
            # slice is [start:end:step], so doing [0::3] will give us every 3rd element
            # green_data = msg.data[GREEN::3]
            self.num_color = 0
            i = 0
            while i < len(msg.data):
                # get b, g, r values
                b = msg.data[i]
                g = msg.data[i+1]
                r = msg.data[i+2]
                # method to recognise green
                if (g / (b+r+1)) > 1:
                    self.num_color+=1
                    # print("b %d, g %d, r%d " % (b, g, r))
                i+=3
            if self.num_color > 2000:
                self.get_logger().info("met threshold for red values %d " % self.num_color)
                self.motor.stop_motor()
                self.current_state = "Stopped"
                self.get_logger().info("WARNING: Dangerously close!")
                self.get_logger().info("Waiting... ")
                for t in range(STOP_WAIT):
                    self.get_logger().info("%d..." % (STOP_WAIT - t))
                    time.sleep(1)
                self.get_logger().info("Done Waiting")
                self.motor.start_motor_forward()
        return

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

    def start_motor_forward(self, throttle=THROTTLE):
        # set shared variables
        self.current_throttle = throttle
        self.current_angle = 0.0
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

    def set_angle(self, angle):
        # set shared angle
        self.current_angle = angle
        self.publish_message()      


def publish_telemetry(motor : MotorPublisher, lidar : LidarSubscriber, camera : CameraSubscriber):
    """ Publishes telemetry to console """
    motor_status = "|{0:-^44}|".format("") + "\n"
    motor_status += "|{0:^44}|".format("MOTOR STATUS") + "\n"
    motor_status += "|{0:^20}|{1:^11}|{2:^11}|".format("STATE", "THROTTLE", "ANGLE") + "\n"
    motor_status += "|{0:^20}|{1:^11.1f}|{2:^11.2f}|".format(lidar.current_state, motor.current_throttle, motor.current_angle) + "\n"
    motor_status += "|{0:-^44}|".format("")
    print(motor_status)

    lidar_status = "|{0:^44}|".format("LIDAR") + "\n"
    lidar_status += "|{0:^14}|{1:^14}|{2:^14}|".format("NW: %0.2f" % lidar.all_compass_scans[7], "N: %0.2f" % lidar.all_compass_scans[0], "NE: %0.2f" % lidar.all_compass_scans[1]) + "\n"
    lidar_status += "|{0:^14}|{1:^14}|{2:^14}|".format("W: %0.2f" % lidar.all_compass_scans[6], "ROBOT", "E: %0.2f" % lidar.all_compass_scans[2]) + "\n"
    lidar_status += "|{0:^14}|{1:^14}|{2:^14}|".format("SW: %0.2f" % lidar.all_compass_scans[5], "S: %0.2f" % lidar.all_compass_scans[4], "SE: %0.2f" % lidar.all_compass_scans[3]) + "\n"
    lidar_status += "|{0:-^44}|".format("")
    print(lidar_status)

    if FEATURE_STOP_SIGN == 1:
        camera_status = "|{0:^44}|".format("CAMERA") + "\n"
        camera_status += "|{0:^14}|{1:^14}|".format("STATE", camera.current_state) + "\n"
        camera_status += "|{0:-^44}|".format("")
        print(camera_status)

    if FEATURE_HW_MONITOR == 1:
        hw_status = "|{0:^44}|".format("HARDWARE") + "\n"
        hw_status += "|{0:^14}|{1:^14}|{2:^14}|".format("CPU : %0.2f" % psutil.cpu_percent(), 
                                                                "RAM : %0.2f" % psutil.virtual_memory().percent,
                                                                "FREE : %0.02f" % (100 - psutil.virtual_memory().percent)) + "\n"
        hw_status += "|{0:-^44}|".format("") + "\n\n"
        print(hw_status)

    return

def main(args=None):
    count = 0
    rclpy.init(args=args)
    print("Starting Program")

    # Setting up out Pubs / Subs
    motor_publisher = MotorPublisher()
    lidar_subscriber = LidarSubscriber(motor_publisher, FEATURE_BACKWARD_DRIVING)
    camera_subscriber = None
    if FEATURE_STOP_SIGN == ON:
        camera_subscriber = CameraSubscriber(motor_publisher)

    print("Spinning")
    motor_publisher.start_motor_forward(1.0)

    while(1):
        rclpy.spin_once(lidar_subscriber)
        if lidar_subscriber.stopped == 1:
            print("stop condition met")
            break
        if count % 5 == 0:
            if FEATURE_TELEMETRY == ON:
                publish_telemetry(motor_publisher, lidar_subscriber, camera_subscriber)
        if FEATURE_STOP_SIGN == ON and count % 50 == 0:
            count = 0
            rclpy.spin_once(camera_subscriber)

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