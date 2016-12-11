# Roboy_Reflexes

We prevent Roboy from falling over. By mimicking the patella tendon reflex Roboy is able to detect a critical and ustable position and move back up to a more stable one.

Currently the critical values are recorded from a highly unstable knee position. This can be extended by providing more unstable positions and implementing machine learning, kinect vision detection, etc. to optimally calculate the critical position.

Furthermore, an Intel Edison is to be used to attach the whole functionality to Roboy. A whole PC setup to start the reflex is thereby avoided.

Our template is run with ROS catkin and can be extended to different reflexes.

##Prerequisites:

roboy_ros_control 
roboy_ros_trajectory

for recording data:
roboy_gui

##Building

To build the project you need to clone the project into the catkin workspace src folder

Example:

```
cd ~/catkin_ws/src
git clone https://github.com/valentindiehl/Roboy_Reflexes.git
cd  workspace src folder

Example:

```
cd ~/catkin_ws/src
git clone https://github.com/valentindiehl/Roboy_Reflexes.git
cd ..
catkin_make
```
