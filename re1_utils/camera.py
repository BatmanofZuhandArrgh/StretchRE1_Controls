import os
import pyrealsense2 as rs
import numpy as np
from math import radians

from re1_utils.math_utils import get_rotation_mat

def get_rs_intrinsic_mat(width = 480, height = 640):
    '''
    Modified from fairo/droidlet/lowlevel/hello_robot/remote/remote_hello_robot.py
    '''
    config = rs.config()
    pipeline = rs.pipeline()
    config.enable_stream(rs.stream.color, height, width, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, height, width, rs.format.z16, 30)
    cfg = pipeline.start(config)

    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    i = color_profile.get_intrinsics()
    intrinsic_mat = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
    pipeline.stop()
    return intrinsic_mat

def get_cur_rs_frame(width = 480, height = 640):
    '''
    Get 1 image frame from realsense camera
    Shape should be (640, 480, 3)
    Output:
    color_frame: rgb image object from pyrealsense
    color: numpy array of rgb frame
    depth_frame: depth image object from pyrealsense
    depth: numpy array of depth frame in meters
    '''
    
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()

    pipe = rs.pipeline()

    cfg = rs.config()

    cfg.enable_stream(rs.stream.color, height, width, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, height, width, rs.format.z16, 30)

    try:
        profile = pipe.start(cfg)

        for x in range(5):
            pipe.wait_for_frames()

        frameset = pipe.wait_for_frames()
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        pipe.stop()

        print("Frames Captured")
        color = np.asanyarray(color_frame.get_data())
        color = color.transpose(1,0,2)
        color = np.fliplr(color)

        depth = np.asanyarray(depth_frame.get_data())
        depth = np.fliplr(depth)

        new_depth = []
        #Get distance in meters from the camera
        for h in range(depth.shape[0]):
            cur_depth = []
            for w in range(depth.shape[1]):
                cur_depth.append(depth_frame.get_distance(w, h))
            new_depth.append(cur_depth)    

        # depth = new_depth#.transpose(1,0)
        depth = np.reshape(new_depth, newshape=depth.shape).transpose(1,0)
        return color_frame, color, depth_frame, depth
    
    except RuntimeError as e:
        pipe.stop()
        raise e

def get_rs_colorized_depth(depth_frame):
    #Return visualized depth, shape (640, 480, 3)
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    colorized_depth = colorized_depth.transpose(1,0,2)
    colorized_depth = np.fliplr(colorized_depth)
    return colorized_depth

def get_cam_height_pan_tilt():
    '''
    Extrinsic matrix that transform coordinates from base coordinate system to camera coordinate system
    Modified from fairo/droidlet/lowlevel/hello_robot/remote/remote_hello_robot.py
    #Pan: Left is positive, origin is perpendicular straight forward
    #Tilt: Up is positive, origin is horizontal
    '''
    from stretch_body.robot import Robot
    #Initiate robot
    robot = Robot()
    robot.startup()

    from pytransform3d.urdf import UrdfTransformManager
    import pytransform3d.transformations as pt
    import pytransform3d.visualizer as pv

    urdf_path = os.path.join(os.getenv("HELLO_FLEET_PATH"), os.getenv("HELLO_FLEET_ID"), "exported_urdf", "stretch.urdf")
    mesh_path = os.path.join(os.getenv("HELLO_FLEET_PATH"), os.getenv("HELLO_FLEET_ID"), "exported_urdf")

    tm = UrdfTransformManager()
    with open(urdf_path, "r") as f:
        urdf = f.read()
        tm.load_urdf(urdf, mesh_path=mesh_path)

    s = robot.get_status()
    robot.stop()

    head_pan = s["head"]["head_pan"]["pos"]
    head_tilt = s["head"]["head_tilt"]["pos"]
    
    # # Get Camera transform
    tm.set_joint("joint_head_pan", head_pan)
    tm.set_joint("joint_head_tilt", head_tilt)

    source_coord, target_coord = 'camera_color_frame', 'link_right_wheel' 
    camera_transform = tm.get_transform(source_coord,target_coord)
    
    height_from_wheel_center = max(abs(camera_transform[:,3]))

    return head_pan, head_tilt, height_from_wheel_center

def get_rs_extrinsic_mat(type = 'cam2world'):
    '''
    Output 4x4 extrinsic matrix
    '''
    head_pan, head_tilt, height = get_cam_height_pan_tilt()
    
    
    rotation_mat = get_rotation_mat(x_angle=0, y_angle=-head_tilt, z_angle=-head_pan)
    inv_extrinsic_mat = np.concatenate((rotation_mat, np.array([0,0,height]).reshape(3,1)), axis=1)
    inv_extrinsic_mat = np.concatenate((inv_extrinsic_mat, np.array([0,0,0,1]).reshape(1,4)), axis=0)
    
    if type == 'cam2world' :
        return inv_extrinsic_mat
    elif type == 'world2cam':
        return np.linalg.inv(inv_extrinsic_mat)
    else:
        raise ValueError(type)  

if __name__ == "__main__":
    # get_rs_intrinsic_mat()
    # print(get_rs_extrinsic_mat())
    # get_cur_rs_frame()
    print(get_cam_height_pan_tilt())
    #0.10891263593988053, 0.027611654181941538
    
    #height:1.28575359