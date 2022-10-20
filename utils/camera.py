import pyrealsense2 as rs

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
    
    return {
        'width': intr.width,
        'height': intr.height,
        'ppx': intr.ppx,
        'ppy': intr.ppy,
        'fx': intr.fx,
        'fy': intr.fy,
        'coeffs': intr.coeffs
    }

# def get_cur_rs_frame():

if __name__ == "__main__":
    print(get_rs_intrinsic_mat())