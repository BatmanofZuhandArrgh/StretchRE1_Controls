from POI.point_of_interest import POI
from re1_utils.objdet_utils import load_label

class OOI(POI):
    def __init__(
        self, img_coord, depth,
        obj_class, obj_atributes, 
        bbox, conf_score,
        eid = None) -> None:
        super().__init__(img_coord, depth, eid)
        '''
        obj_cls: str, class of object detected
        obj_atrs: list of str, atributes of objects, like "green", "tall",...
        bbox: tuple, bbox coord on image
        conf: float, [0,1], confidence score output of obj det model
        '''
        
        label_dict = load_label()
        
        self.obj_cls = obj_class
        self.obj_name = label_dict[obj_class]['label']
        self.obj_atrs= obj_atributes
        self.bbox = bbox
        self.conf = conf_score
        self.depth = depth
        
        self.active_landmark = True #If this isn't in the dictionary, it should be set to false
        self.landmark_color = (0,255,0) #Green
        
    def get_OOI_attrs(self):
        return self.obj_name, self.obj_atrs

    def show(self):
        print(self.obj_name + ' id_' + str(self.eid) + ' at ' + str(self.bbox) + f' depth {self.depth} meters')


