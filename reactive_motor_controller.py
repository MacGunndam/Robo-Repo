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

# ros imports
import rclpy
from rclpy.node import Node

# Messages Used
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan

THRESHOLD = 0.5
STOP_THRESHOLD = 0.25
HALLWAY_THRESHOLD = 0.85
TURN_THRESHOLD = HALLWAY_THRESHOLD - 0.1

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/rplidar_ros/scan', self.listener_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.stopped = 0 # boolean to stop motor
        self.motor_pub = MotorPublisher()
        
        print("Starting motor")
        self.motor_pub.start_motor()

    def listener_callback(self, msg):
        ranges = msg.ranges
        self.get_logger().info("Got %d range readings" % len(ranges))

        ranges_length = len(ranges)

        ''' 
        translating lidar readings into a compass for the robot, where N=front, E=right, S=behind, W=left
        each lidar reading is roughly 45 degrees apart
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
                self.motor_pub.stop_motor()
                self.stopped = 1

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
              self.motor_pub.change_angle(0)
              self.get_logger().info("no other states active. driving forward")




        # couldn't figure out how to print multiple variables in one call so here we are
        # self.get_logger().info("lidar_N: [%f]" % lidar_N)
        # self.get_logger().info("lidar_E: [%f]" % lidar_E)
        # self.get_logger().info("lidar_S: [%f]" % lidar_S)
        # self.get_logger().info("lidar_W: [%f]" % lidar_W)

        # self.get_logger().info("lidar_NE: [%f]" % lidar_NE)
        # self.get_logger().info("lidar_SE: [%f]" % lidar_SE)
        # self.get_logger().info("lidar_SW: [%f]" % lidar_SW)
        # self.get_logger().info("lidar_NW: [%f]" % lidar_NW)
        
        # # tests to make sure each direction works (they do)
        # if lidar_N <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("N Threshold Met: %0.4f" % lidar_N)

        # if lidar_E <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("E Threshold Met: %0.4f" % lidar_E)

        # if lidar_S <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("S Threshold Met: %0.4f" % lidar_S)

        # if lidar_W <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("W Threshold Met: %0.4f" % lidar_W)



        # if lidar_NE <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("NE Threshold Met: %0.4f" % lidar_NE)

        # if lidar_SE <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("SE Threshold Met: %0.4f" % lidar_SE)

        # if lidar_SW <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("SW Threshold Met: %0.4f" % lidar_SW)

        # if lidar_NW <= THRESHOLD:
        #         self.threshold_met = 1
        #         self.get_logger().info("NW Threshold Met: %0.4f" % lidar_NW)
        # for elt in ranges:
        #     if elt < THRESHOLD and elt > 0.33:
        #         self.stopped = 1
        #         self.get_logger().info("Threshold Met: %0.4f" % elt)
        #         break
            

class MotorPublisher(Node):
        def __init__(self):
                super().__init__('motor_publisher')
                self.wheel_publisher = self.create_publisher(
                ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
                
        def start_motor(self):
                # set the message
                wheel_msg = ServoCtrlMsg()
                wheel_msg.throttle = 0.65
                wheel_msg.angle = 0.0
                # Publish the message
                self.wheel_publisher.publish(wheel_msg)
                # self.wheel_publisher.publish(wheel_msg)
        
        def stop_motor(self):
                # set the message
                wheel_msg = ServoCtrlMsg()
                wheel_msg.throttle = 0.0
                wheel_msg.angle = 0.0
                # Publish the message
                self.wheel_publisher.publish(wheel_msg)

        def change_angle(self, angle):
                # set the message
                wheel_msg = ServoCtrlMsg()
                wheel_msg.angle = angle
                # Publish the message
                self.wheel_publisher.publish(wheel_msg)

       

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()
#     motor_publisher = MotorPublisher()
    
    print("Start Program")
    # print("Starting motor")
    # motor_publisher.start_motor()


    print("Spinning")

    while(1):
        rclpy.spin_once(lidar_subscriber)
        if lidar_subscriber.stopped == 1:
            print("stop condition met")
            break
            # lidar_subscriber.threshold_met = False
    
    

    print("Stopping Motor")
    # motor_publisher.stop_motor()

if __name__ == '__main__':
    main()
