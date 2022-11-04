import os
import pyrealsense2 as rs
import numpy as np
from math import radians

'''
Coordinate system:
Image: Oxy like opencv
Camera: 
"specified in meters, with the coordinate [0,0,0] referring to the center of the physical imager"
" Within this space, the positive x-axis points to the right,
 the positive y-axis points down, and the positive z-axis points forward."
** Although we did find in 
Base:
'''


def _get_rs_intrinsic_mat():
    #Depreciated
    '''
    Intrinsic matrix to convert from camera coordinate system to image coordinate system
    '''
    d = rs.decimation_filter()
    p = rs.pipeline()
    p.start()
    fs = p.wait_for_frames()
    df = fs.get_depth_frame()
    processed = d.process(df)
    prof = processed.get_profile()
    video_prof = prof.as_video_stream_profile()
    intr = video_prof.get_intrinsics()
    
    intrinsic_mat_dict = {
        'width': intr.width,
        'height': intr.height,
        'ppx': intr.ppx,
        'ppy': intr.ppy,
        'fx': intr.fx,
        'fy': intr.fy,
        'coeffs': intr.coeffs
    }

    intrinsic_mat = np.array([intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0,0,1]).reshape((3,3))
    # intrinsic_mat = np.array([intr.fy, 0, intr.ppy, 0, intr.fx, intr.ppx, 0,0,1]).reshape((3,3))
    return intrinsic_mat, intrinsic_mat_dict

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

def get_rs_extrinsic_mat():
    '''
    Extrinsic matrix that transform coordinates from base coordinate system to camera coordinate system
    Modified from fairo/droidlet/lowlevel/hello_robot/remote/remote_hello_robot.py
    '''
    from stretch_body.robot import Robot
    #Initiate robot
    robot = Robot()
    robot.startup()

    #Are init in the droidlet
    # if not robot.is_calibrated():
    #     robot.home()
    # robot.stow()

    # #Load urdf 3d transform manager
    from pytransform3d.urdf import UrdfTransformManager
    import pytransform3d.transformations as pt
    import pytransform3d.visualizer as pv

    urdf_path = os.path.join(
    os.getenv("HELLO_FLEET_PATH"),
    os.getenv("HELLO_FLEET_ID"),
    "exported_urdf",
    "stretch.urdf",
    )
    mesh_path = os.path.join(os.getenv("HELLO_FLEET_PATH"), os.getenv("HELLO_FLEET_ID"), "exported_urdf")

    tm = UrdfTransformManager()
    with open(urdf_path, "r") as f:
        urdf = f.read()
        tm.load_urdf(urdf, mesh_path=mesh_path)

    s = robot.get_status()
    robot.stop()
    
    camera_transform = tm.get_transform('camera_link','base_link')
    # print(camera_transform)

    head_pan = s["head"]["head_pan"]["pos"]
    head_tilt = s["head"]["head_tilt"]["pos"]
    
    # # Get Camera transform
    tm.set_joint("joint_head_pan", head_pan)
    tm.set_joint("joint_head_tilt", head_tilt)
    camera_transform = tm.get_transform("camera_color_frame", "base_link")
    # print(camera_transform)
    # correct for base_link's z offset from the ground
    # at 0, the correction is -0.091491526943
    # at 90, the correction is +0.11526719 + -0.091491526943
    # linear interpolate the correction of 0.023775
    interp_correction = 0.11526719 * abs(head_tilt) / radians(90)
    # print('interp_correction', interp_correction)

    camera_transform[2, 3] += -0.091491526943 + interp_correction
    return camera_transform


def get_cur_rs_frame(width = 480, height = 640):
    '''
    Get 1 image frame from realsense camera
    Output:
    color_frame: rgb image object from pyrealsense
    color: numpy array of rgb frame
    depth_frame: depth image object from pyrealsense
    depth: numpy array of depth frame in meters
    '''
    pipe = rs.pipeline()

    cfg = rs.config()

    cfg.enable_stream(rs.stream.color, height, width, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, height, width, rs.format.z16, 30)

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

def get_rs_colorized_depth(depth_frame):
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    colorized_depth = colorized_depth.transpose(1,0,2)
    colorized_depth = np.fliplr(colorized_depth)
    return colorized_depth

if __name__ == "__main__":
    # get_rs_intrinsic_mat()
    # print(get_rs_extrinsic_mat())
    get_cur_rs_frame()
