#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from stretch_body.hello_utils import deg_to_rad


#A ROS Node that rotates or move the base of the robot
class Move:
    def __init__(self):
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)
        '''
        This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=1) 
        declares that your node is publishing to the /stretch/cmd_vel topic using the message type Twist.
        The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
        '''
    def move_forward(self):
        """
        Function that publishes Twist messages
        :param self: The self reference.

        :publishes command: Twist message.
        """
        command = Twist()
        command.linear.x = -0.1
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        self.pub.publish(command)

    def rotate(self, degree):
        """
        Function that publishes Twist messages
        :param self: The self reference.

        :publishes command: Twist message.
        """
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = deg_to_rad(degree)
        print('ready to publish')
        self.pub.publish(command) #publish

def spin(spin_vel = 30, time = 4, disable_signals=False):
    # rospy.init_node('spin', anonymous=True, disable_signals= disable_signals)
    base_motion = Move()
    loop_frequency = 10
    rate = rospy.Rate(loop_frequency) #Going through the loop 10 times per second
    counter = 0
    while not rospy.is_shutdown():
        base_motion.rotate(spin_vel)
        rate.sleep()
        counter += 1
        print(counter)
        if counter>= loop_frequency * time:
            break

if __name__ == '__main__':
    spin()