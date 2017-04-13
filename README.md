# vision based quadrotor landing(ECEn 631 final project)
Jae Lee and Jesse Wynn

Using a camera, the landing pad is detected based on its distinct color(can be replaced with any other type of marker) and quadrotor moves so that it is above the landing pad and starts to descend. The framework used is ROS.

install dronekit python api
install ardupilot SITL

Pixhawk Parameters to be set:
-SERIAL2_PROTOCOL = 1 (the default) to enable MAVLink on the serial port.
-SERIAL2_BAUD = 1500 so the Pixhawk can communicate with the Computer at 1500000 baud.

-WP_YAW_BEHAVIOR = 0 (in mavproxy execute 'param set WP_YAW_BEHAVIOR 0')  This ensures Pixhawk heading angle is fixed throughout flight

To launch MAVProxy and connect to Pixhawk:
1. Change connection string in file relative_tracking_downward_cam.py to connection_string = '127.0.0.1:14551'
2. mavproxy.py --master=/dev/ttyUSB0 --baudrate 1500000 --aircraft MyCopter --out 127.0.0.1:14551
3. roslaunch vision_landing vision_landing.launch

To launch Ardupilot SITL and test in simulation:
1. Change connection string in file relative_tracking_downward_cam.py to connection_string = '127.0.0.1:14550'
2. cd ~/ardupilot/ArduCopter/&& sim_vehicle.py --console --map --location=RockCanyon
3. roslaunch vision_landing vision_landing.launch

To change starting location in Ardupilot SITL edit locations.txt in ~/ardupilot/Tools/autotest
