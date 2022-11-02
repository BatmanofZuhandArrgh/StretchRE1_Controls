import pyrealsense2 as rs
import numpy as np

def get_rs_intrinsic_mat():
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
    return intrinsic_mat, intrinsic_mat_dict

def get_rs_extrinsic_mat():
    raise NotImplementedError

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
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    pipe.stop()
    print("Frames Captured")
    color = np.asanyarray(color_frame.get_data())
    color = color.transpose(1,0,2)
    depth = np.asanyarray(depth_frame.get_data())

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
    return colorized_depth

if __name__ == "__main__":
    # print(get_rs_intrinsic_mat()[0])
    get_cur_rs_frame()
