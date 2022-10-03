import os

# Let us first create the default `AgentMemory` object for our Locobot using some pre-baked and thoughtful memory schemas.

from memory.sql_memory import AgentMemory
from memory.loco_memory_nodes import NODELIST, DetectedObjectNode

SQL_SCHEMAS = [
    os.path.join(os.getcwd(), "memory", "base_memory_schema.sql"),
    os.path.join(os.getcwd(), "memory", "loco_memory_schema.sql"),
]

class SampleObject: #A sample of WorldObject
    def __init__(self, label, center, rgb_depth, mask=None, xyz=None):
        self.label = label
        self.center = center
        self.rgb_depth = rgb_depth
        self.mask = mask
        self.xyz = xyz 
        self.eid = None
        self.feature_repr = None
        self.bounds = (0,0,0,0,0,0)

    def get_xyz(self):
        """returns xyz in canonical world coordinates."""
        return {"x": self.xyz[0], "y": self.xyz[1], "z": self.xyz[2]}

    def get_bounds(self):
        """returns bounding box as dict."""
        return (
            self.bounds[0],
            self.bounds[1],
            self.bounds[2],
            self.bounds[3],
            self.bounds[4],
            self.bounds[5],
        )

    def get_masked_img(self):
        raise NotImplementedError

    def to_struct(self):
        raise NotImplementedError

if __name__ == '__main__':
    sample = SampleObject(label='0', center=(0,0,0), rgb_depth=(0,0,0,0), xyz=(0,0,0))
    all_objects = [
        sample,
        sample
    ]
    memory = AgentMemory(db_file=":memory:", schema_paths=SQL_SCHEMAS, nodelist=NODELIST)

    #We can see the types of nodes that can be stored inside the memory
    print(memory.nodes)
    '''
    Let us store the previously detected objects into memory, using this new memory system.

    For this, we will use the `DetectedObjectNode`.
    A physical `object` is represented in memory as a `DetectedObjectNode`, which is thoughtfully annotated with properties such as it's `color` and it's detected `xyz` location.

    As a reminder, in the previous section, we deduplicated the objects detected in the scene, and stored them in the variable `previous_objects`.

    Now, we will store these `all_objects` into the memory.
    Let us start with storing and retreiving one object, and inspecting the results.
    '''

    memory_id = DetectedObjectNode.create(memory, all_objects[0])

    # Now, let us retreive the `DetectedObjectNode` from memory.
    memory.get_mem_by_id(memory_id)
    # The memory object is in it's raw packed form, and is not yet converted back to a dict with accessible fields.

    # We can access the detected objects back from memory as dicts using the `get_all` function:
    DetectedObjectNode.get_all(memory)

    # Now, let us store the rest of the detected objects into memory
    for obj in all_objects[1:]: # we already stored all_objects[0]
        DetectedObjectNode.create(memory, obj)