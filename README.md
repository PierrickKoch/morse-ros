morse-ros
=========

_MORSE-ROS demo wander_

INSTALL
-------

```
sudo apt-get install python-pip
sudo pip install --upgrade rosinstall
mkdir -p ~/work/ros-addons
cd ~/work/ros-addons
rosws init
rosws merge /opt/ros/fuerte/.rosinstall
rosws merge https://raw.github.com/pierriko/morse-ros/master/.rosinstall
rosws update
source setup.bash
```

Orocos packages

```
sudo apt-get install ros-fuerte-orocos-toolchain ros-fuerte-rtt-*
```

Get MORSE at [morse.openrobots.org](http://morse.openrobots.org/)
