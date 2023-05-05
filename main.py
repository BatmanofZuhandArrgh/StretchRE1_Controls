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

        
        self.pre_map_file = "/home/hello-robot/stretch_user/maps/py_fullmap" # Map filepath when the room was mapped before navigation manually  
        self.cur_map_file = "/home/hello-robot/stretch_user/maps/py_temp"    # Nap filepath when NavigationControl.map() was used to do naive mapping
        self.map_file = None # Decide which map to use later

        #Navigation goal for naive navigation
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        
        #Angles used for naive returned
        self.final_rotation_angle = None
        self.left_or_right_turn = None

    def start_roscore(self):
        # Start roscore
        print('Starting Master')

        self.parent = ROSLaunchParent('running_ros_master', [], is_core=True)     # run_id can be any string
        self.parent.start()

    def stop_roscore(self):
        print("Ending Master")
        self.parent.shutdown()

    def map(self):
        #3. Map the room automatically. Naively map the room by only spinning once
        #Init mapping node
        self.start_roscore()

        rospy.init_node('Mapping_Rotate', anonymous=True)
        print('Inited')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hello-robot/catkin_ws/src/stretch_ros/stretch_navigation/launch/mapping.launch"])
        launch.start()
        rospy.loginfo("started")
        
        rospy.sleep(10)

        #Rotate robot    
        os.system("rosservice call /switch_to_navigation_mode")
        spin(spin_vel = 36, time = 5)
        spin(spin_vel = -36, time =5)

        rospy.sleep(15)

        #Save map
        try:
            save_proc = subprocess.Popen(["rosrun", "map_server", "map_saver", '-f', self.cur_map_file], stdout=subprocess.PIPE) #automatically shutdown when done
            rospy.sleep(10)
        except KeyboardInterrupt:
            print("Interrupt map-saving process")
            save_proc.terminate()
            rospy.signal_shutdown('Mapping Rotate is shutting down')
            self.stop_roscore()

        launch.shutdown()   
        rospy.signal_shutdown('Mapping Rotate is shutting down')
        print("Shutdown0: ", rospy.is_shutdown())
        self.stop_roscore()
        time.sleep(10)

    def nav(self, x, y, theta, full_map = True):
        '''
        Input x, y, and theta coordinate of the robot from the starting position
        Ox is straight forward positive (meter)
        Oy is left to right positive (meter)
        theta (degree)
        '''
        #4. Navigate to goal coordinate
        #Choose map
        # self.start_roscore()

        self.map_file = self.pre_map_file if full_map else self.cur_map_file
        
        # try:
            #Launch ros navigation 
            # nav_proc = subprocess.Popen(["roslaunch", "stretch_navigation", "navigation.launch", "map_yaml:={}.yaml".format(self.map_file)])
        os.system("roslaunch stretch_navigation navigation.launch map_yaml:={}.yaml".format(self.map_file))
        time.sleep(10)
            #Send base goal
            # rospy.init_node('Navigation', argv=sys.argv)

            # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            # roslaunch.configure_logging(uuid)
            # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hello-robot/catkin_ws/src/stretch_ros/stretch_navigation/launch/navigation.launch map_yaml:={}.yaml".format(self.map_file)])
            

# Create a ROSLaunchParent object for the navigation launch file
            # cli_args = ['stretch_navigation','navigation.launch',"map_yaml:={}.yaml".format(self.map_file)]
            # cli_args = [("/home/hello-robot/catkin_ws/src/stretch_ros/stretch_navigation/launch/navigation.launch", "map_yaml:={}.yaml".format(self.map_file))]
            # resolved_args = roslaunch.rlutil.resolve_launch_arguments(cli_args)
            # roslaunch_args = cli_args[2:]
            # roslaunch_files = [(resolved_args, roslaunch_args)]
            # print(roslaunch_files)
            # raise KeyboardInterrupt

            # launch = roslaunch.parent.ROSLaunchParent(run_id = uuid,roslaunch_files = cli_args)

            # launch.start()
            # rospy.loginfo("started")
            # rospy.sleep()

        print('Start navigating')
        nav = Nav()
        nav.go_to(x, y, theta)
        rospy.sleep(10)
        raise KeyboardInterrupt
            # launch.shutdown()

        # except KeyboardInterrupt:
        #     print("Interrupt navigating subprocess")
        #     # nav_proc.terminate()
        #     # launch.shutdown()
        #     # rospy.signal_shutdown('Navigation is shutting down')
        #     self.stop_roscore()
        #     time.sleep(10)

        print('Finished moving')
        # launch.shutdown()  
        # rospy.spin() # Keeps node running until shutdown is called       
        # rospy.signal_shutdown('Navigation is shutting down')
        print("Shutdown0: ", rospy.is_shutdown())
        time.sleep(10)
        print("Shutdown1: ", rospy.is_shutdown())
        rospy.wait_for_shutdown()
        self.stop_roscore()

    def naive_nav(self, x, y, theta):
        # Dead reckoning. No Map.

        #Assign so that the robot remember where it is
        self.target_x = x
        self.target_y = y
        self.target_theta = theta

        robot = Robot()
        robot.startup()

        # Move forward or backward
        if x!= 0:
            robot.base.translate_by(x_m=x) 
            robot.push_command()
            time.sleep(10)

        # Right is negative, left is positive, switch this once intergrate with the landmark-camera module
        # If the robot needs to go sideways, it will turn to the right (+Ox), and start to go front or back
        self.left_or_right_turn = -90 if y != 0 else 0
        # Turn
        if self.left_or_right_turn != 0:
            robot.base.rotate_by(deg_to_rad(self.left_or_right_turn))
            robot.push_command()
            time.sleep(5) 

        # Move left or right
        if y != 0:
            robot.base.translate_by(x_m=-y) 
            robot.push_command()
            time.sleep(10) 

        print('Status: Reach target location')
        
        self.final_rotation_angle = -self.left_or_right_turn + theta
        if self.final_rotation_angle != 0:
            robot.base.rotate_by(deg_to_rad(self.final_rotation_angle))
            robot.push_command()
            time.sleep(5) 

        print('Status: Reach target rotation')
        robot.stop() 
    
    def naive_return(self):
        #Tracing back to the orginal location and rotation
        robot = Robot()
        robot.startup()
        
        if self.final_rotation_angle != 0:
            robot.base.rotate_by(deg_to_rad(-self.final_rotation_angle))
            robot.push_command()
            time.sleep(5) 
        
        if self.target_y != 0:
            robot.base.translate_by(x_m=self.target_y) 
            robot.push_command()
            time.sleep(10)

        if self.left_or_right_turn != 0:
            robot.base.rotate_by(deg_to_rad(-self.left_or_right_turn))
            robot.push_command()
            time.sleep(5) 
        
        print('Status: Reach original rotation')

        # Move forward or backward
        if self.target_x!= 0:
            robot.base.translate_by(x_m=-self.target_x) 
            robot.push_command()
            time.sleep(10)
        
        print('Status: Reach original location')

if __name__ == '__main__':
    x, y, theta =0, -0.45, 45
    nav_con = NavigationControl()
    nav_con.naive_nav(x, y, theta)
    nav_con.naive_return()

    # x, y, theta =0, -0.45, 45
    # nav_con = NavigationControl()
    # nav_con.map()
    # nav_con.nav(0.5, 0, 0, full_map = True)
    # nav_con.stop_roscore()
