import cv2
import numpy as np
import matplotlib.pyplot as plt

class PathPlanner():
    def __init__(self, landmark_dict):
        self.landmarks = landmark_dict
        self.graph = None
        self.viz_frame = None

    def build_graph(self):
        pass

    def convert_base2viz_coord(self, x, y,z,  max_x_range, max_y_range, max_z_range, height, width, plane_type='xy'):
        x = width/2 + x/max_x_range* width/2
        y = height + y/max_y_range * height
        
        z = height - z/max_z_range * height

        center_coordinate = (int(x),int(y)) if plane_type == 'xy' else (int(x), int(z)) 
        return center_coordinate

    def viz(self, height = 640, width = 360, max_x_range = 1, max_y_range = 2, max_z_range = 2, plane_type='xy'):
        '''
        height, width: shape of the image
        maximum_range: (meter) max range of the viz_frame, the y distance is a fraction of this max_range
        '''
        self.viz_frame = np.zeros(shape=(height, width, 3))
        
        cv2.circle(self.viz_frame, (int(width//2)-1, height-1), 5, (0,255, 255), 5)

        for index in self.landmarks['objects'].keys():
            x, y, z = self.landmarks['objects'][index].base_coord
            center_coordinate = self.convert_base2viz_coord(x, y,z,  max_x_range, max_y_range, max_z_range, height, width, plane_type = plane_type)
            cv2.circle(self.viz_frame, center_coordinate, 2, (0,255, 0), 2)
            self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (255, 0, 0), 1, cv2.LINE_AA)
        
        for w in self.landmarks['locations'].keys():
            for h in self.landmarks['locations'][w].keys():
                if self.landmarks['location'][w][h].active_landmark:
                    x, y, z = self.landmarks['location'][w][h].base_coord
                    center_coordinate = self.convert_base2viz_coord(x, y,z,  max_x_range, max_y_range, max_z_range, height, width, plane_type = plane_type)
                    cv2.circle(self.viz_frame, center_coordinate, 2, (0,0, 255), 2)
                    self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (255, 0, 0), 1, cv2.LINE_AA)

        plt.imshow(self.viz_frame) 
        plt.show()

if __name__ == '__main__':
    planner = PathPlanner()
    print(1)