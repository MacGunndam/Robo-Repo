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

# ros imports
import rclpy
from rclpy.node import Node

# Messages Used
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan

THRESHOLD = 0.5

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan, '/rplidar_ros/scan', self.listener_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.threshold_met = 0 # boolean to stop motor 

    def listener_callback(self, msg):
        ranges = msg.ranges
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

        self.get_logger().info("Got %d range readings" % len(ranges))

        # couldn't figure out how to print multiple variables in one call so here we are
        self.get_logger().info("lidar_N: [%f]" % lidar_N)
        self.get_logger().info("lidar_E: [%f]" % lidar_E)
        self.get_logger().info("lidar_S: [%f]" % lidar_S)
        self.get_logger().info("lidar_W: [%f]" % lidar_W)

        self.get_logger().info("lidar_NE: [%f]" % lidar_NE)
        self.get_logger().info("lidar_SE: [%f]" % lidar_SE)
        self.get_logger().info("lidar_SW: [%f]" % lidar_SW)
        self.get_logger().info("lidar_NW: [%f]" % lidar_NW)
        
        # tests to make sure each direction works (they do)
        if lidar_N <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("N Threshold Met: %0.4f" % lidar_N)

        if lidar_E <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("E Threshold Met: %0.4f" % lidar_E)

        if lidar_S <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("S Threshold Met: %0.4f" % lidar_S)

        if lidar_W <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("W Threshold Met: %0.4f" % lidar_W)



        if lidar_NE <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("NE Threshold Met: %0.4f" % lidar_NE)

        if lidar_SE <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("SE Threshold Met: %0.4f" % lidar_SE)

        if lidar_SW <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("SW Threshold Met: %0.4f" % lidar_SW)

        if lidar_NW <= THRESHOLD:
                self.threshold_met = 1
                self.get_logger().info("NW Threshold Met: %0.4f" % lidar_NW)
        # for elt in ranges:
        #     if elt < THRESHOLD and elt > 0.33:
        #         self.threshold_met = 1
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
        wheel_msg.throttle = -0.8
        wheel_msg.angle = 0.0
        # Publish the message
        self.wheel_publisher.publish(wheel_msg)
        self.wheel_publisher.publish(wheel_msg)
    
    def stop_motor(self):
        # set the message
        wheel_msg = ServoCtrlMsg()
        wheel_msg.throttle = 0.0
        wheel_msg.angle = 0.0
        # Publish the message
        self.wheel_publisher.publish(wheel_msg)
       

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()
    motor_publisher = MotorPublisher()
    
    print("Start Program")
    print("Starting motor")
    # motor_publisher.start_motor()


    print("Spinning")

    while(1):
        rclpy.spin_once(lidar_subscriber)
        if lidar_subscriber.threshold_met == 1:
            print("Threshold met")
            break
            # lidar_subscriber.threshold_met = False
    
    

    print("Stopping Motor")
    # motor_publisher.stop_motor()

if __name__ == '__main__':
    main()
