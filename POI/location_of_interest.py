from POI.point_of_interest import POI

class LOI(POI):
    def __init__(self, img_coord, depth, bbox, active = False, uuid=None) -> None:
        super().__init__(img_coord, depth, uuid)
        
        # active here means it's shown to the user and can be chosen
        self.active_landmark = active 
        self.landmark_color =  (0, 0, 255) #Red for BGR
        self.bbox = bbox

    def show(self):
        print(f"Location {str(self.bbox)}")

    
