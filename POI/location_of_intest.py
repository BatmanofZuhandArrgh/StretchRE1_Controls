from point_of_interest import POI

class LOI(POI):
    def __init__(self, img_coord, depth, uuid=None) -> None:
        super().__init__(img_coord, depth, uuid)
