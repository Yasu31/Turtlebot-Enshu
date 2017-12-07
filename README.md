# Turtlebot class experiment
written in Euslisp
# demo video (YouTube)
[![video link](https://raw.githubusercontent.com/Yasu31/Turtlebot-Enshu/master/image.png)](https://www.youtube.com/watch?v=OJsFJWH9d7A)
# to run euslisp program
## run it inside catkin workspace
1. save program under src/ in beginner_tutorials
1. $chmod +x publish-to-topic.l subscribe-to-topic.l
1. rosrun beginner_tutorials publish-to-topic.l (or roseus publish-to-topic.l)
## or, more simply
1. $roseus playground.l

# chanbara.l
set variable "demo" to 1 for demo motion(as seen in video), 0 to recognize checkerboard(checkerboard should be placed at hilt of user's sword)

set variable "real" to 1 to move real robot, 0 for simulation only

minimal.launch and checkerboard-detector.launch must be running in the background (refer to class notes)

## flowchart
![flowchart](https://raw.githubusercontent.com/Yasu31/Turtlebot-Enshu/master/chanbara.png)
# playground.l
## (circle)
moves the hand around in circles using IK
## (listen)
listens to ps3 controller
## (pickup)
uses the ps3 controller to pick up something.
use arrows to move hand, circle button to open and close hand

# follow-checker.l
follows checkerboard, and when it gets near enough, picks it up.
based on display-checkerboard.l inside catkin_ws/src/robot-programming/dxl-armed-turtlebot/euslisp/

# simple-turtle.l
goes around avoiding obstacles and cliffs (using bumper & cliff sensor)
# memos
to show compressed image
```
rosrun image_view image_view image:=/camera/rgb/image_rect_color _image_transport:=compressed

roslaunch roseus_tutorials checkerboard-detector.launch rect0_size_x:=0.025 rect0_size_y:=0.025 grid0_size_x:=5 grid0_size_y:=4 translation0:="0 0 0" image:=image_rect_mono group:=/camera/rgb frame_id:=camera_rgb_frame

roslaunch dxl_armed_turtlebot hsi_color_filter.launch DEFAULT_NAMESPACE:=/camera/depth_registered INPUT:=points h_min:=-20 h_max:=50 s_min:=120
```

get position&orientation of end of arm
```lisp
(send *dxl-armed-turtlebot* :arm :end-coords :worldrot)
```
