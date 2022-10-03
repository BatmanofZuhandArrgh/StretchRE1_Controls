import numpy as np

class POI():
    #Point of Interest
    def __init__(self, img_coord, depth, uuid = None) -> None:
        '''
        These are the np.array of coordinates of (the center of) the POI
        img_coord = [w, h, 1]. Origin #TODO define
        cam_coord = [x, y, depth]. Origin is camera
        base_coord= [x, y, z]. Origin is base of robot
        uuid: node if saved in the agent memory
        '''
        self.img_coord = np.concatenate((img_coord, np.array([1])), axis=0) 
        self.depth = depth
    
        self.cam_coord = None
        self.base_coord = None

        self.uuid = uuid

    def set_cam_coord(self, inv_cam_intrinsic_mat):
        '''
        inv_cam_extrinsic_mat:(3x3)
        '''
        self.cam_coord = inv_cam_intrinsic_mat.dot(self.img_coord) * self.depth

    def set_world_coord(self, inv_cam_extrinsic_mat):
        '''
        inv_cam_extrinsic_mat:(4x4)
        '''
        self.world_coord = inv_cam_extrinsic_mat.dot(self.cam_coord)

    

