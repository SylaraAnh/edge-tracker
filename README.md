# Modification

- enable LIVOX AVIA
- enable 6-axis livox IMU
  - give initial orientation according to instant acceleration and gravity (could be further improved)
  
- unify extrinsic parameter

# EDGE-TRACKER

This repository contains code for a lidar-visual-inertial odometry and mapping system, which combines the advantages of
---

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (Tested with kinetic and melodic)

---

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/
cd ..
catkin_make
```

---

## Datasets


The datasets used in the paper can be downloaded from Google Drive. The data-gathering sensor suite includes: Velodyne VLP-16 lidar, FLIR BFS-U3-04S2M-CS camera, MicroStrain 3DM-GX5-25 IMU, and Reach RS+ GPS.

```
https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV?usp=sharing
```

**Note** that the images in the provided bag files are in compressed format. So a decompression command is added at the last line of ```launch/module_sam.launch```. If your own bag records the raw image data, please comment this line out.


---

## Run the package

1. Configure parameters:

```
Configure sensor parameters in the .yaml files in the ```config``` folder.
```

2. Run the launch file:
```
roslaunch edge_tracker run_livox.launch
```


---

## Related Packages



---

## TODO

  - [ ] Update graph optimization using all three factors in imuPreintegration.cpp, simplify mapOptimization.cpp, increase system stability 

---

## Paper 


```
```

---

## Acknowledgement

  - The lidar-inertial odometry module is adapted from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/a246c960e3fca52b989abf888c8cf1fae25b7c25).
