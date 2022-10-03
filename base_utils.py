from typing import Tuple
from collections import namedtuple

XYZ = Tuple[int, int, int]
POINT_AT_TARGET = Tuple[int, int, int, int, int, int]


Pos = namedtuple("pos", "x, y, z", defaults=(None,) * 3)
Look = namedtuple("look", "yaw, pitch")
Player = namedtuple("Player", "entityId, name, pos, look")


# TODO make this just a dict, and change in memory and agent
# eg in object_looked_at and PlayerNode
def to_player_struct(pos, yaw, pitch, eid, name):
    if len(pos) == 2:
        pos = Pos(pos[0], 0.0, pos[1])
    else:
        pos = Pos(pos[0], pos[1], pos[2])
    look = Look(yaw, pitch)
    return Player(eid, name, pos, look)