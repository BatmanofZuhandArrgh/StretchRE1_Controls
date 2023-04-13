import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad
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
    
#Arm
def retract_arm():
    #Retract arm and gripper

    #Homing lift
    import stretch_body.lift as lift
    l=lift.Lift()
    if not l.startup(threaded=False):
        exit()
    l.home()
    l.stop()

    #Retracting wrist=> Doesn't work for some reason, Ask Tian
    import stretch_body.wrist_yaw as wrist_yaw
    g=wrist_yaw.WristYaw()
    if not g.startup(threaded=False):
        exit()
    v_des=g.params['motion']['default']['vel']
    a_des=g.params['motion']['default']['accel']
    g.move_to(deg_to_rad(90), v_des, a_des)
    g.stop()

    #Homing arm
    import stretch_body.arm as arm
    a=arm.Arm()
    if not a.startup(threaded=False):
        exit()
    a.home()
    a.stop()

def rotate_base(degree):
    robot=stretch_body.robot.Robot()
    robot.startup()
    print('here')
    robot.base.rotate_by(deg_to_rad(degree))
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
    # move_head('head_tilt', radians(-20))

    # retract_arm()

    rotate_base(90)