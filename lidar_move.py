#for the turtlebot/lidar
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

#for the servo motor
import time
import RPi.GPIO as GPIO

servo_pin= 21 #Choose pwm channel to be used to control the servo

GPIO.setmode(GPIO.BCM) #Set pin numbering convention
GPIO.setup(servo_pin, GPIO.OUT) #Set the pin as an output
p= GPIO.PWM(servo_pin, 50) #Initialise the servo to be controlled by pwm with 50Hz frequency
p.start(5) #set servo to 15 degrees as starting position

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sub  # prevent unused variable warning

    def servo_movement(self, angle):
        relative_angle= (float(angle)/180)*10 +2.5
        p.ChangeDutyCycle(relative_angle)

    def scan_callback(self, msg):
        laser_range = np.array(msg.ranges) #create numpy array
        laser_range[laser_range==0] = np.nan #replace elements that are equal to 0 with nan 'Not A Number'
        front_laser_range = laser_range[360:720] #filters out the front of lidar
        
        if np.shape(front_laser_range)[0] > 0: #checks if shape is greater than 0, which indicates that it contains data
            closest_distance = np.nanmin(front_laser_range) #finds minimum value, ignoring NaN values
            if closest_distance <=1.0:
                self.servo_movement(15)
                time.sleep(1)
                self.servo_movement(165)
                time.sleep(1)

def main(args=None):   
    rclpy.init(args=args) #initalises the 'rclpy' library, passing command line arguments 'args' as an optional parameter
    wall_follower = WallFollower() #Creates instance of 'Wallfollower' object
    rclpy.spin(wall_follower) #starts execution of ROS2 node represented by 'WallFollower' class. Runs until node is shutdown
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follower.destroy_node()
    rclpy.shutdown()

    p.stop() #stops the PWM function
    GPIO.cleanup() #Function called at the end of program to reset GPIO pin state

if __name__ == '__main__':
    main()

