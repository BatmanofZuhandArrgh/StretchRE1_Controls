# Overview
Controls components for [RE1 Stretch](https://hello-robot.com/product) using the [stretch_body API](https://docs.hello-robot.com/0.2/).

<picture>
 <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/BatmanofZuhandArrgh/StretchRE1_Controls/main/ConferencePoster.jpg">
 <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/BatmanofZuhandArrgh/StretchRE1_Controls/main/ConferencePoster.jpg">
 <img alt="YOUR-ALT-TEXT" src="https://raw.githubusercontent.com/BatmanofZuhandArrgh/StretchRE1_Controls/main/ConferencePoster.jpg">
</picture>

The repository contains modules in folders in the main directory:
- memory   : SQL database 
- yolov7   : State-of-the-art Object Detection at the time (which was 1 month before yolov8 was release :))
- re1_utils: utils functions and classes
- POI      : landmark object classes for SceneMap
- main.py  : main Navigation and Controls
- control_flow.md: Full elaboration of the control flow and more notes for devs
- Other demo jupyter notebooks, guide mds, pdfs, graphs and posters

<picture>
 <source media="(prefers-color-scheme: dark)" srcset="hhttps://raw.githubusercontent.com/BatmanofZuhandArrgh/StretchRE1_Controls/main/BCI_2_RE1_flowchart_withoutmemory.png">
 <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/BatmanofZuhandArrgh/StretchRE1_Controls/main/BCI_2_RE1_flowchart_withoutmemory.png">
 <img alt="YOUR-ALT-TEXT" src="https://raw.githubusercontent.com/BatmanofZuhandArrgh/StretchRE1_Controls/main/BCI_2_RE1_flowchart_withoutmemory.png">
</picture>

**Memory**

Memory code heavily inspired and taken from fairo's droidlet's memory module 
[droidlet](https://github.com/facebookresearch/fairo/tree/main/droidlet/memory) provides a memory system that can store generic metadata. This memory system is used by the planner/controller to do tasks utilizing context provided by information stored in memory.

The memory is backed by an SQL database, and has schemas to represent common semantic information for robots and the environments

Added a BCIDetectedObjects table to base_memory_schema.sql
Added a BCIDetectedObjectFeatures table to loco_memory_schema.sql
Wrote note class for BCIDetectedObjectNode for OOI

Note: This was developed first before everything else, so may not be compatible with later OOI. As of May 2023, a memory module was not intergrated into the design nor the full flow.

See Demo_Droidlet_Memory.ipynb and Demo_ObjDet.ipynb

**Yolov7**

Cloned from [yolov7](https://github.com/BatmanofZuhandArrgh/yolov7/tree/a0bbdb1fd768663eb8ddcf6fb505801a14954264)

Please see yolov7/RE1_README.md to install and use

See Demo_CoordViz.ipynb for OOI visualization and Demo_ObjDet.ipynb for perception and image processing

**POI**

Objects of Interest (OOI) and Locations of Interest (LOI) inherit from Points of Interest (POI). After SceneMap, POI's locations are mapped onto the world coordinates for the user screen to be chosen as a target.

See Demo_CoordViz.ipynb for OOI visualization and Demo_ObjDet.ipynb for perception and image processing

**main.py**
Full controls class NavigationControl. Inputs are the world coordinate of the target and the robot would move to the target, and return to the original position.

- map() is an automatic naive mapping of the room by rotating the robot and lidar-scanning the room, then saving the map, save in self.cur_map_file. Otherwise, to use a full complete map, map the room manually beforehand using this [guide](https://github.com/hello-robot/stretch_ros/tree/noetic/stretch_navigation).
- amcl_nav() uses ROS navigation, includes the localization
and navigation in itself. Needs to be run untethered.
- naive_nav() and naive_return() are dead reckoning to and back from the target landmark, and does not require the map of the room, only the input objects and coordinates from the SceneMap


# Installation:

``
git clone https://github.com/BatmanofZuhandArrgh/RE1.git
cd RE1
pip install -r requirements.txt

<!-- git clone https://github.com/BatmanofZuhandArrgh/yolov7.git -->

git submodule update --init
pip install -r yolov7/requirements.txt
``



