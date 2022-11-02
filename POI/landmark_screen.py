import numpy as np
import cv2
import matplotlib.pyplot as plt
from pprint import pprint
from POI.location_of_interest import LOI
from POI.object_of_interest import OOI
from re1_utils.objdet_utils import plot_all_boxes

class LandmarkScreen():
    def __init__(self, color_frame, depth_frame) -> None:
        self.height, self.width, _ = color_frame.shape
        self.depth_frame = cv2.medianBlur(np.float32(depth_frame),5)
        self.color_frame = color_frame

        #Assuming the height and width is dividable by 10
        self.grid_unit_h, self.grid_unit_w = int(self.height //10), int(self.width //10)
        #Grid representation for initation of grid, 10x10
        self.grid_repr = np.array([
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,1,0,0,1,0,0,1,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,1,0,0,1,0,0,1,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,1,0,0,1,0,0,1,0],
            [0,0,0,0,0,0,0,0,0,0]
        ])

        self.grid = {}
        for w in range(self.grid_repr.shape[0]):
            self.grid[w] = {}
            for h in range(self.grid_repr.shape[1]):
                img_coord = (int((w+.5)* self.grid_unit_w),int((h+.5)* self.grid_unit_h))
                bbox = ((int(w* self.grid_unit_w),int(h* self.grid_unit_h)),(int((w+1)* self.grid_unit_w),int((h+1)* self.grid_unit_h)))
                
                # print(w, h, self.grid_repr[w][h], img_coord, self.depth_frame.shape)
                # print(self.depth_frame[img_coord[1]][img_coord[0]])
                depth_grid_unit = self.depth_frame[bbox[0][1]: bbox[1][1], bbox[0][0]: bbox[1][0]] #index by h, w
                
                depth = np.median(depth_grid_unit)
                # print(depth_grid_unit, depth, bbox)
                
                self.grid[w][h] = LOI(
                    img_coord=img_coord,
                    depth=depth,
                    bbox = bbox,
                    active=self.grid_repr[w][h],
                )
        self.cur_preds = None #Current object detected

        self.landmarks = {}
        
        self.landmarks['locations'] = self.grid
        self.landmarks['objects'] = {} 

    def show_locations(self):
        for w in range(self.grid_repr.shape[0]):
            for h in range(self.grid_repr.shape[1]):
                if self.grid[w][h].active_landmark:
                    # cv2.circle(self.color_frame, self.grid[w][h].img_coord[:-1], radius= 2, color = (255,0,0), thickness = 2)
                    cv2.rectangle(self.color_frame, self.grid[w][h].bbox[0], self.grid[w][h].bbox[1], self.grid[w][h].landmark_color, 1)
                    self.color_frame = cv2.putText(self.color_frame, str(self.grid[w][h].depth), self.grid[w][h].img_coord[:-1], cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.5, (255,0,0), 1, cv2.LINE_AA)

    def show_objects(self):
        if self.cur_preds is not None:
            plot_all_boxes(self.cur_preds, self.color_frame)
    
    def show(self):
        self.show_locations()
        self.show_objects()

        # self.color_frame = cv2.cvtColor(self.color_frame, cv2.COLOR_RGB2BGR)
        # cv2.imwrite('./sample/selection_output.png', self.color_frame)
        plt.imsave('./sample/selection_output.png', self.color_frame)

        # cv2.imshow('',self.color_frame)
        
        # if cv2.waitKey(0) & 0xFF == ord('q'):
        #     #closing all open windows 
        #     cv2.destroyAllWindows() 

    def update_OOI(self, preds):
        self.cur_preds = preds
        for i in range(preds.shape[0]):
            coord = preds[i, :]
            bbox = ((int(coord[0]),int(coord[1])),(int(coord[2]),int(coord[3])))
            depth_grid_unit = self.depth_frame[bbox[0][1]: bbox[1][1], bbox[0][0]: bbox[1][0]] #index by h, w
            depth = np.median(depth_grid_unit)

            img_coord = np.array([(coord[2]-coord[0])/2,(coord[3]-coord[1])/2])
            self.landmarks['objects'][i] = OOI(
                img_coord = img_coord, 
                depth = depth,
                obj_class = int(coord[-1]),
                obj_atributes = 'None', 
                bbox = bbox,
                conf_score = coord[4],
                eid = i
            )

    def update_base_coords(self, inv_cam_intrinsic_mat, inv_cam_extrinsic_mat):
        for index in self.landmarks['objects'].keys():
            self.landmarks['objects'][index].set_cam_coord(inv_cam_intrinsic_mat)
            self.landmarks['objects'][index].set_world_coord(inv_cam_extrinsic_mat)
    
        for w in range(self.grid_repr.shape[0]):
            for h in range(self.grid_repr.shape[1]):
                self.landmarks['locations'][w][h].set_cam_coord(inv_cam_intrinsic_mat)
                self.landmarks['locations'][w][h].set_world_coord(inv_cam_extrinsic_mat)

    def update_cam_coords(self, inv_cam_intrinsic_mat):
        for index in self.landmarks['objects'].keys():
            self.landmarks['objects'][index].set_cam_coord(inv_cam_intrinsic_mat)     

        for w in range(self.grid_repr.shape[0]):
            for h in range(self.grid_repr.shape[1]):
                self.landmarks['locations'][w][h].set_cam_coord(inv_cam_intrinsic_mat)

    def get_landmarks(self):
        return self.landmarks
    
    def get_OOI(self):
        return self.landmarks['objects']
    
    def get_LOI(self):
        return self.landmarks['locations']

    def show_OOI(self):
        for i in range(self.cur_preds.shape[0]):
            self.landmarks['objects'][i].show()

if __name__ == '__main__':
    sample_img = cv2.imread('./sample/color_output.png')
    sample_img = cv2.cvtColor(sample_img, cv2.COLOR_BGR2RGB)
    sample_img = cv2.resize(sample_img, (480, 640))
    landmark_screen = LandmarkScreen(color_frame=sample_img, depth_frame=sample_img[:,:, 0])
    landmark_screen.show()

    preds = np.array(
        [
        [375, 470, 426,614,0.81986,39],
        [322, 467, 379,632,0.78286,39],
        [286, 280, 315,353,0.40197,39]
        ]
    )
    landmark_screen.update_OOI(preds)
    landmark_screen.show()

