Coordinate System in object of interest:
1. Image coordinates: (For vertical image from vertical camera)
- Origin at top left of the image
- Positive Ox points right
- Positive Oy points down
- Depth is positive outwards from the camera

2. Camera coordinates
POI.temp_cam_coord: This is the camera coordinate of original horizontal image
- Output from inv_intrinsic_matrix.dot(transposed_img_coord)
- Origin at center of image
- Positive Ox points left
- Positive Oy points down
- Positive Oz points outwards from camera, is depth

POI.cam_coord: Official coordinate of this repo and the vertical image (transposed from the original)
- Output = np.array([self.temp_cam_coord[2], -self.temp_cam_coord[1],self.temp_cam_coord[0]])
- Origin at center of image
- Positive Ox points outwards from camera, is depth
- Positive Oy points left
- Positive Oz points up

3. World coordinates
- Origin is at center of camera when taken the photo, drop down to the height of the axis of the right wheel, from the ground
- Positive Ox points outwards from camera, is depth
- Positive Oy points left
- Positive Oz points up