from POI.point_of_interest import POI

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
        self.obj_cls = obj_class
        self.obj_atrs= obj_atributes
        self.bbox = bbox
        self.conf = conf_score

    def get_OOI_attrs(self):
        return self.obj_cls, self.obj_atrs, self.bbox, self.conf
