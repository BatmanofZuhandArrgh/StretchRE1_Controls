import numpy as np
 # Now, the points are in camera frame.
        # In camera frame
        # z is positive into the camera
        # (larger the z, more into the camera)
        # x is positive to the right
        # (larger the x, more right of the origin)
        # y is positive to the bottom
        # (larger the y, more to the bottom of the origin)
        #                                 /
        #                                /
        #                               / z-axis
        #                              /
        #                             /_____________ x-axis (640)
        #                             |
        #                             |
        #                             | y-axis (480)
        #                             |
        #                             |

        # We now need to transform this to pyrobot frame, where
        # x is into the camera, y is positive to the left,
        # z is positive upwards
        # https://pyrobot.org/docs/navigation
        #                            |    /
        #                 z-axis     |   /
        #                            |  / x-axis
        #                            | /
        #  y-axis        ____________|/
        #
        # If you hold the first configuration in your right hand, and
        # visualize the transformations needed to get to the second
        # configuration, you'll see that
        # you have to rotate 90 degrees anti-clockwise around the y axis, and then
        # 90 degrees clockwise around the x axis.
        # This results in the final configuration
        
class POI():
    #Point of Interest
    def __init__(self, img_coord, depth, eid = None) -> None:
        '''
        These are the np.array of coordinates of (the center of) the POI
        img_coord = [w, h, 1]. Origin #TODO define
        cam_coord = [x, y, depth]. Origin is camera
        base_coord= [x, y, z]. Origin is base of robot
        uuid: node if saved in the agent memory
        '''
        self.img_coord = np.concatenate((img_coord, np.array([1])), axis=0) 
        self.depth = depth
    
        self.cam_coord = [0,0,0] 
        self.base_coord = [0,0,0]

        self.eid = eid 

    def set_cam_coord(self, inv_cam_intrinsic_mat):
        '''
        inv_cam_extrinsic_mat:(3x3), convert from image coord
        '''
        img_coord = np.array([640 - self.img_coord[1], self.img_coord[0], 1]) #Since the output of obj det is from a transposed image, so we flipped the img_coord to be compatible with realsense intrinsic matrix #TODO refactor
        self.cam_coord = inv_cam_intrinsic_mat.dot(self.img_coord) * self.depth

    def set_world_coord(self, inv_cam_extrinsic_mat):
        '''
        inv_cam_extrinsic_mat:(4x4), convert from camera coord
        '''
        cam_coord_4x1 = np.concatenate((self.cam_coord, [1]), axis = 0)
        self.base_coord = inv_cam_extrinsic_mat.dot(cam_coord_4x1)[:-1]
        #Flip world coord again
        # self.base_coord = np.array([self.base_coord[0], self.base_coord[2], -self.base_coord[1]])
    
    def show(self):
        raise NotImplementedError

#Original Object class to implement along with memory  
# class WorldObject:
#     def __init__(self, label, center, rgb_depth, mask=None, xyz=None):
#         self.label = label
#         self.center = center
#         self.rgb_depth = rgb_depth
        
#         self.xyz = xyz #if xyz else rgb_depth.get_coords_for_point(self.center)
#         """get_coords_for_point: fetches xyz from the point cloud in pyrobot coordinates and converts it to
#         canonical world coordinates. Not available, in our current WIP
#         """
        
#         self.eid = None
#         self.feature_repr = None
        
#         self.mask = mask
#         self.bounds = rgb_depth.get_bounds_for_mask(self.mask)
#         """for all points in the mask, returns the bounds as an axis-aligned bounding box."""

#     def get_xyz(self):
#         """returns xyz in canonical world coordinates."""
#         return {"x": self.xyz[0], "y": self.xyz[1], "z": self.xyz[2]}

#     def get_bounds(self):
#         """returns bounding box as dict."""
#         return (
#             self.bounds[0],
#             self.bounds[1],
#             self.bounds[2],
#             self.bounds[3],
#             self.bounds[4],
#             self.bounds[5],
#         )

#     def get_masked_img(self):
#         raise NotImplementedError

#     def to_struct(self):
#         raise NotImplementedError

