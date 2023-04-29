Stretch RE1 flow in its non-memory form is divided in 4 parts
- Scene Mapping (SceneMap)
- Brain Computer Interface (BCI)
- Navigation Control (NavCon)
- Grasp

1. SceneMap
The Robot (RE1) will start in a position next to the user, and it will start by scanning the entire space by spinning the base in its place. RE1, in every discrete interval of 10 degrees, it sends the image captured by RE1's camera and its set of landmark to BCI. The user can choose a landmark to move to, or to keep spinning the robot. 

see RE1_memory_objdet_demo.ipynb

a) Image capturing
RE1 spins with RE1_utils/move_base.py. After it capture the color and depth image. The YOLOv7 model was called and loaded, the image is preprocessed and put through the computer vision (CV) model to get the prediction (xyxy image coordinate, confidence score, class). 

b) Landmark mapping

The color and depth images are put into the class object LandmarkScreen, and called update_OOI with the array of prediction. 

LandmarkScreen will get intrinsic and extrinsic matrices, and update_base_coords to convert the image coordinates to camera and base coordinates.

LandmarkScreen will populate the image with an array of 
landmarks, both Location-of-Interest (LOI) and Object-of-Interest (OOI), see self.grid and self.grid_repr. These will be sent to the BCI and ask for the user input.


2. BCI
(Sameer's TODO)
BCI will return to NavCon the base coordinate of the target object

3. NavCon
RE1 will be in the position and rotation at the end of SceneMap. NavCon contains 3 options for mapping and navigation, each can be called in NavigationControl:
a) Manual PreMapping:
- If RE1 is stationed in just one room, the user can manually run mapping with https://github.com/hello-robot/stretch_ros/blob/master/stretch_navigation/README.md to map the full room. The map's name should be pass into NavigationControl, or it should be named by the default string py_fullmap

b) Automatic Mapping:
- RE1 can call nav_con.map() to automatically map while spinning a full 360 degrees in its place. It will not produce a full map, and NavigationControl will not move into places that has not been mapped.

=> In both a and b, RE1 can call nav_con.nav to get to the position facing the target object. It will use the map produced and particle filter to slowly move while localizing.

After grasping, it will navigate back to its original position, at origin point (0,0,0)

c) Naive Navigation:
- RE1 will navigate with pure dead reckoning (assuming the most simple, trivial map, and move using python SDK). It will then position itself for grasping.
- It will also infer the original location from the target location, as python sdk does not automatically do this. 

4. Grasp
(Tian's TODO)