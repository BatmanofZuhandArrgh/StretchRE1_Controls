import os
import sys
import time
import subprocess
import roslaunch
import rospy
import stretch_body.robot

sys.path.append('re1_utils')
from controls import retract_arm, reset_head_position, rotate_base

class NavigationControl():
    def __init__(self):

        #1.Retract the lift and arm for easier mapping
        # retract_arm() 

        #2.Homing head position
        # reset_head_position()

        self.target_object_world_coord = None

        self.start_roscore()
    
    def start_roscore(self):
        # Start roscore
        print('Starting Master')
        self.roscore_process = subprocess.Popen('roscore')

    def stop_roscore(self):
        self.roscore_process.terminate()

    def roomscan_4_object(self):
        '''
        #. Scan the room for object (Not used)
        Rotate the robot at its position for 360 degrees
        Stop when the robot is facing the direction of the object
        Return the world coordinates of the target object
        '''
        pass

    def map(self):
        #3. Map the room
        rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hello-robot/catkin_ws/src/stretch_ros/stretch_navigation/launch/mapping.launch"])
        launch.start()
        rospy.loginfo("started")

        rospy.sleep(15)
        print('-----------------------------------------------------')
        #rotate base
        # rotate_base(90)

        rospy.sleep(15)
        subprocess.Popen(["rosrun", "map_server", "map_saver", '-f', "/home/hello-robot/stretch_user/maps/py_temp"], stdout=subprocess.PIPE)

        rospy.sleep(15)

        launch.shutdown()
        

if __name__ == '__main__':
    nav_con = NavigationControl()
    nav_con.map()

    nav_con.stop_roscore()
