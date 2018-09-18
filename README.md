<div align="center">
<img src="illustration.png" width="400" alt="HoPE" />
</div>

**Horizontal Plane Extractor** (or **HoPE** for short) is a ROS package for
extracting horizontal planes given point cloud input. The planes can be for example the ground or a tabletop surface,
with which robotic tasks such as navigation or manipulation can be performed with ease.

## Installation
This package requests following dependences:
* ROS (tested on both Indigo and Kinetic)
* PCL 1.7
* OpenCV (tested on 2.4 and 3.3)
* Boost
* Eigen

On a fresh Ubuntu 14.04 or 16.04 System, you can install aformentioned packages by installing `ros-<distro>-desktop` using:

```
sudo apt-get install ros-<distro>-desktop ros-<distro>-compressed-*
```

Make sure to change `<distro>` to what your ROS distribution really is.

After these prerequests have been met, clone this repository into your /home/$USER/catkin_ws/src directory, and do:
```
cd hope
mkdir build & cd build
cmake ..
make -j

```

## Basic Usage
```
roscore
rosrun hope hope_ros
```

## Advanced Usage
To reproduce the experiments in our parper:

#### Test on RGB-D data (TUM)


#### Test on point cloud ()


#### Customize the parameters in ROS real-time usage


## License
MIT.

## Cite
If you refered or used HoPE in your article, please considering cite:
