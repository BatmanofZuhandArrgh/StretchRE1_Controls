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

    def convert_base2viz_coord(self, x, y, max_x_range, max_y_range, height, width):
        x = width/2 - x/max_x_range* width/2
        y = height + y/max_y_range * height
        return (int(x),int(y))

    def viz(self, height = 640, width = 360, max_x_range = 2, max_y_range = 4):
        '''
        height, width: shape of the image
        maximum_range: (meter) max range of the viz_frame, the y distance is a fraction of this max_range
        '''
        self.viz_frame = np.zeros(shape=(height, width, 3))
        
        cv2.circle(self.viz_frame, (int(width//2)-1, height-1), 5, (0,255, 255), 5)

        for index in self.landmarks['objects'].keys():
            x, y, _ = self.landmarks['objects'][index].base_coord
            center_coordinate = self.convert_base2viz_coord(self, x, y, max_x_range, max_y_range, height, width)
            cv2.circle(self.viz_frame, center_coordinate, 2, (0,255, 0), 2)
            self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (255, 0, 0), 1, cv2.LINE_AA)
        
        for w in self.landmarks['locations'].keys():
            for h in self.landmarks['locations'][w].keys():
                if self.landmarks['location'][w][h].active_landmark:
                    x, y, _ = self.landmarks['location'][w][h].base_coord
                    center_coordinate = self.convert_base2viz_coord(self, x, y, max_x_range, max_y_range, height, width)
                    cv2.circle(self.viz_frame, center_coordinate, 2, (0,0, 255), 2)
                    self.viz_frame = cv2.putText(self.viz_frame, self.landmarks['objects'][index].obj_name, center_coordinate,  cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (255, 0, 0), 1, cv2.LINE_AA)

        plt.imshow(self.viz_frame) 
        plt.show()

if __name__ == '__main__':
    planner = PathPlanner()
    print(1)