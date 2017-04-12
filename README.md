# vision based quadrotor landing(ECEn 631 final project)
Jae Lee and Jesse Wynn

Using a camera, the landing pad is detected based on its distinct color(can be replaced with any other type of marker) and quadrotor moves so that it is above the landing pad and starts to descend. The frame work used is ROS.

install dronekit
install ardupilot

mavproxy.py --master=/dev/ttyUSB0 --baudrate 1500000 --aircraft MyCopter --out 127.0.0.1:14551

roslaunch vision_landing vision_landing.launch
