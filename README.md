morse-ros
=========

_ROS Package for MORSE, the Openrobots Simulator_

INSTALL
-------

```
sudo apt-get install python-pip
sudo pip install --upgrade rosinstall
mkdir -p ~/work/ros-addons
cd ~/work/ros-addons
rosws init
rosws merge /opt/ros/fuerte/.rosinstall
rosws merge http://pierriko.com/morse-ros/morse-ros.rosinstall
rosws update
source setup.bash
```

Orocos packages

```
sudo apt-get install ros-fuerte-orocos-toolchain ros-fuerte-rtt-*
```

Get MORSE at [morse.openrobots.org](http://morse.openrobots.org/)

[![morse-logo](http://www.openrobots.org/morse/doc/latest/_static/morse-logo.png)](http://morse.openrobots.org/)

[![ros-org-logo](http://www.ros.org/_wiki/images/ros_org.png)](http://www.ros.org/)
