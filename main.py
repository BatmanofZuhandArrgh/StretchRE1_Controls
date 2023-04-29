import os
import sys
import time
import subprocess
import roslaunch
import rospy
import keyboard

from stretch_body.robot import Robot
from roslaunch.parent import ROSLaunchParent
from stretch_body.hello_utils import deg_to_rad

sys.path.append('re1_utils')
from misc import retract_arm, reset_head_position, rotate_base
from base_move import spin
from navigation import Nav

class NavigationControl():
    def __init__(self):

        #1.Retract the lift and arm for easier mapping and navigating. Can ignore this if the lift is already in preferable position
        # retract_arm() 

        #2.Homing head position
        # reset_head_position()

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
        spin(spin_vel = 36, time = 10)
        # spin(spin_vel = -36, time =5)

        rospy.sleep(15)

        #Save map
        try:
            save_proc = subprocess.Popen(["rosrun", "map_server", "map_saver", '-f', self.map_file], stdout=subprocess.PIPE) #automatically shutdown when done
            rospy.sleep(10)
        except KeyboardInterrupt:
            print("Interrupt map-saving process")
            save_proc.terminate()

        launch.shutdown()    

    def nav(self, x, y, theta, full_map = True):
        '''
        Input x, y, and theta coordinate of the robot from the starting position
        Ox is straight forward positive (meter)
        Oy is left to right positive (meter)
        theta (degree)
        '''
        #4. Navigate to goal coordinate
        # os.system("roslaunch stretch_navigation navigation.launch map_yaml:={}.yaml".format(self.map_file))
        if full_map:
            self.map_file = "/home/hello-robot/stretch_user/maps/py_fullmap"
        try:
            nav_proc = subprocess.Popen(["roslaunch", "stretch_navigation", "navigation.launch", "map_yaml:={}.yaml".format(self.map_file)])
            print('subprocess ID --------------------------')
            print(nav_proc.pid)
            
            rospy.init_node('navigation', argv=sys.argv)
            nav = Nav()
            nav.go_to(x, y, theta)
            rospy.sleep(10)
        except KeyboardInterrupt:
            print("Interrupt navigating process")
            nav_proc.terminate()
        
        print('Finished moving')
        nav_proc.terminate()        
    
    def naive_nav(self, x, y, theta):
        self.stop_roscore() # Stopping after starting master automatically when init
        robot = Robot()
        robot.startup()

        # Move forward or backward
        if x!= 0:
            robot.base.translate_by(x_m=x) 
            robot.push_command()
            time.sleep(10)

        # Right is positive, left is negative, switch this once intergrate with the landmark-camera module
        # If the robot needs to go sideways, it will turn to the right (+Ox), and start to go front or back
        left_or_right_turn = -90 if y != 0 else 0
        # Turn
        if left_or_right_turn != 0:
            robot.base.rotate_by(deg_to_rad(left_or_right_turn))
            robot.push_command()
            time.sleep(5) 

        # Move left or right
        if y != 0:
            robot.base.translate_by(x_m=y) 
            robot.push_command()
            time.sleep(10) 

        print('Reach target location')
        
        if -left_or_right_turn + theta != 0:
            robot.base.rotate_by(deg_to_rad(-left_or_right_turn + theta))
            robot.push_command()
            time.sleep(5) 

        print('Reach target rotation')

        robot.stop() 


if __name__ == '__main__':
    x, y, theta = 0, 0, -35
    nav_con = NavigationControl()
    # nav_con.map()
    # nav_con.nav(1, 0, 0, full_map = True)
    nav_con.naive_nav(x, y, theta)
    # nav_con.stop_roscore()
