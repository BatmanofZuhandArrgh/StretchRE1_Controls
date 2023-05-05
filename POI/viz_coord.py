import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import IPython

class Visualizer():
    def __init__(self, landmark_dict):
        self.landmarks = landmark_dict
        self.graph = None
        self.viz_frame = None

    def build_graph(self):
        pass
    
    def viz_cam(self, width = 360, height = 640, max_x_range = 1, max_y_range = 2):
        '''
        Visualize landmarks in camera coordinates (for the purpose of testing)
        Strictly the image that the camera receives
        
        '''
        self.viz_frame = np.zeros(shape=(height, width, 3))
        
        for index in range(len(self.landmarks['objects'])):
            _, y, z = self.landmarks['objects'][index].cam_coord
            viz_x = width/2 - y/max_x_range *  width/2
            viz_y = height/2 - z/max_y_range * height/2

            center_coordinate = (int(viz_x), int(viz_y))
            cv2.circle(self.viz_frame, center_coordinate, 2, (0,255, 0), 2)
            text = '_' + str(np.round(y,2)) + '_' + str(np.round(z, 2)) 
            self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name + text, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (255, 0, 0), 1, cv2.LINE_AA)
            
        IPython.display.display(Image.fromarray(self.viz_frame.astype(np.uint8)))
    
    def convert_base2viz_coord(self, x, y,z,  max_x_range, max_y_range, max_z_range, height, width, coord_type='xy'):
        x = height - x/max_x_range* height
        y = width/2 - y/max_y_range* width/2
        z = height - z/max_z_range * height

        if coord_type == 'xy':
            center_coordinate = (int(y), int(x)) 
        elif coord_type == 'yz':
            center_coordinate = (int(y),int(z))
        else:
            raise ValueError(coord_type)

        return center_coordinate
        
    def viz_base(self, height = 640, width = 360, max_x_range = 3, max_y_range = 1, max_z_range = 2, coord_type='xy'):
        '''
        Visualize landmarks in base coordinates
        height, width: shape of the image
        maximum_range: (meter) max range of the viz_frame, the y distance is a fraction of this max_range
        '''
        self.viz_frame = np.zeros(shape=(height, width, 3))
        
        cv2.circle(self.viz_frame, (int(width//2)-1, height-1), 5, (0,255, 255), 5)

        for index in range(len(self.landmarks['objects'])):
            x, y, z = self.landmarks['objects'][index].base_coord
            # print(x,y,z)
            center_coordinate = self.convert_base2viz_coord(x, y,z,  max_x_range, max_y_range, max_z_range, height, width, coord_type = coord_type)
            # print(center_coordinate)
            cv2.circle(self.viz_frame, center_coordinate, 2, (0,255, 0), 2)
            self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (255, 0, 0), 1, cv2.LINE_AA)
        
        # for w in self.landmarks['locations'].keys():
        #     for h in self.landmarks['locations'][w].keys():
        #         if self.landmarks['locations'][w][h].active_landmark:
        #             x, y, z = self.landmarks['locations'][w][h].base_coord
        #             center_coordinate = self.convert_base2viz_coord(x, y,z,  max_x_range, max_y_range, max_z_range, height, width, coord_type = coord_type)
        #             cv2.circle(self.viz_frame, center_coordinate, 2, (0,0, 255), 2)
        #             self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1, (255, 0, 0), 1, cv2.LINE_AA)
        
        
        IPython.display.display(Image.fromarray(self.viz_frame.astype(np.uint8)))
        # plt.imshow(self.viz_frame) 
        # plt.show()

if __name__ == '__main__':
    viz = Visualizer()
    print(1)