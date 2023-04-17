import os
import sys
import time
import subprocess
import roslaunch
import rospy
import keyboard

from stretch_body.robot import Robot
from roslaunch.parent import ROSLaunchParent

sys.path.append('re1_utils')
from misc import retract_arm, reset_head_position, rotate_base
from base_move import spin
from navigation import Nav

class NavigationControl():
    def __init__(self):

        #1.Retract the lift and arm for easier mapping
        # retract_arm() 

        #2.Homing head position
        # reset_head_position()

        self.target_object_world_coord = None
        self.start_roscore()
        
        self.map_file = "/home/hello-robot/stretch_user/maps/py_temp"
    def start_roscore(self):
        # Start roscore
        print('Starting Master')

        self.parent = ROSLaunchParent('running_ros_master', [], is_core=True)     # run_id can be any string
        self.parent.start()

    def stop_roscore(self):
        print("Ending Master")
        self.parent.shutdown()

    def map(self):
        #3. Map the room
        #Init mapping node
        rospy.init_node('Mapping_Rotate', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hello-robot/catkin_ws/src/stretch_ros/stretch_navigation/launch/mapping.launch"])
        launch.start()
        rospy.loginfo("started")
        
        rospy.sleep(10)

        #Rotate robot    
        os.system("rosservice call /switch_to_navigation_mode")
        spin()

        rospy.sleep(15)

        #Save map
        subprocess.Popen(["rosrun", "map_server", "map_saver", '-f', self.map_file], stdout=subprocess.PIPE)
        rospy.sleep(10)
        
        launch.shutdown()    

    def nav(self, x, y, theta):
        '''
        Input x, y, and theta coordinate of the robot from the starting position
        Ox is straight forward positive (meter)
        Oy is left to right positive (meter)
        theta (degree)
        '''
        #4. Navigate to goal coordinate
        # os.system("roslaunch stretch_navigation navigation.launch map_yaml:={}.yaml".format(self.map_file))
        subprocess.Popen(["roslaunch", "stretch_navigation", "navigation.launch", "map_yaml:={}.yaml".format(self.map_file)])

        rospy.init_node('navigation', argv=sys.argv)
        nav = Nav()
        nav.go_to(x, y, theta)


if __name__ == '__main__':
    nav_con = NavigationControl()
    nav_con.map()
    # nav_con.nav(-0.25, 0, 0)
    nav_con.stop_roscore()
