#for the turtlebot/lidar
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

#for the servo motor
import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM) #Set pin numbering convention
servo_pin= 21 #Choose pwm channel to be used to control the servo
GPIO.setup(servo_pin, GPIO.OUT) #Set the pin as an output
p= GPIO.PWM(servo_pin, 50) #Initialise the servo to be controlled by pwm with 50Hz frequency
p.start(0.83) #set servo to 15 degrees as starting position

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub  # prevent unused variable warning

    def scan_callback(self, msg):
        ranges = msg.ranges  #gives range data
        angle_min = msg.angle_min #start angle of the scan (rad)
        angle_max = msg.angle_max #end angle of the scan (rad)
        angle_increment = msg.angle_increment #angular distance between measurements
        
        #Filters out all inf values
        ranges = [x for x in ranges if x != float('inf')]

        # Filter out the data in front of the lidar (between the 350ish to 10ish degree angle)
        front_ranges= ranges[:3]+ranges[-3:]
        front_distance= sum(front_ranges)/len(ranges)
        print(front_distance)


        # If the wall is 1m away
        if front_distance < 1.0:
            self.get_logger().info('Wall detected, stopping turtlebot')
            p.ChangeDutyCycle(9.2) #Change angle to 165 degrees

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

