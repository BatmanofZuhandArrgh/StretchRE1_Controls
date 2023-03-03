import stretch_body.robot
import time
import stretch_body.stretch_gripper 
from math import radians
#import mpldatacursor
print("Environment Ready")
'''
Guide:
- https://github.com/hello-robot/stretch_tutorials/blob/master/stretch_body/tutorial_introduction.md
- https://github.com/hello-robot/stretch_tutorials/blob/master/stretch_body/tutorial_stretch_body_api.md

The Robot API uses SI units of:

meters
radians
seconds
Newtons
Amps
Volts
Parameters may be named with a suffix to help describe the unit type. For example:

pos_m : meters
pos_r: radians
'''
#Head
def reset_head_position():
    '''
    Home head pan and head tilt to starting position 0 radians    
    '''
    robot=stretch_body.robot.Robot()
    robot.startup()
    
    robot.head.home()
    robot.stop()
    
def move_head(joint, x_r ):
    robot=stretch_body.robot.Robot()
    robot.startup()
    robot.head.move_to(joint, x_r)
    robot.stop()
    

if __name__ == '__main__':
#     robot=stretch_body.robot.Robot()
#     robot.startup()
#     # robot.stow() # Still a problem with out-of-date parameters
  
#     robot.base.pretty_print()
  
#     # robot.base.translate_by(0.25) #command is put on queue
#     # robot.push_command() #all queue up commands are executed at once
#     # time.sleep(1)

#     # robot.base.translate_by(0.1)
#     # robot.push_command()
#     robot.stop()

    # reset_head_position()
    move_head('head_tilt', radians(-20))