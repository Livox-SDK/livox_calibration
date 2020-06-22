
# Note: This repository is no longer maintained，please use the new version https://github.com/Livox-SDK/Livox_automatic_calibration  

# LOAM-Horizon Calibration
## Horizon-Horizon / Horizon-Tele Extrinsic Calibration based on LOAM-Horizon

In the development of our package, we reference to LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).


**Developer:** [Livox](https://www.livoxtech.com)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://.../loam_horizon.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 4. Rosbag Example
### 4.1. **Horizon-Horizon Calibration Example**
Download [Our recorded rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/h_h.bag) and then
```
roslaunch loam_horizon loam_calib_hh.launch
rosbag play h_h.bag
```
After calculation, result will be saved in /tmp/mlcalib_1_TIMESTAMP.txt


### 4.2. **Horizon-Tele Calibration Example**
Download [Our recorded rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/h_t.bag) and then
```
roslaunch loam_horizon loam_calib_ht.launch
rosbag play h_t.bag
```
After calculation, result will be saved in /tmp/mlcalib_1_TIMESTAMP.txt

### 4.3 **Some Tips**
Please config the initial value for extrinsic (rotation and position for LiDAR#1 w.r.t LiDAR#0， ^{#0} tf_{#1}) in the launch file
(loam_calib_hh.launch or loam_calib_ht.launch), which look like these two lines:
```
<rosparam param="/mlcalib/initial_extrinsic_t">[0.00,-0.13, 0.00]</rosparam>
<rosparam param="/mlcalib/initial_extrinsic_ypr">[30.0, 0.0, 0.0]</rosparam><!-- in degrees -->
```

In Horizon-Horizon case, either LiDAR can be LiDAR#0. In Horizon-Tele case, it is recommanded to choose Horizon as LiDAR#0,
because larger FOV of Horizon will benefit the odometry and initial mapping process.

Be careful when handling the timestamp between different LiDAR. It is recommended to synchronize them to make sure that
the timestamp between them is less than 10ms.

Please set these two parameters to achieve a trade-off between calculation speed and accuracy
```
<param name="/mlcalib/gather_count" type="int" value="200" />
<param name="/mlcalib/gather_skip" type="int" value="5" />
```
**gather_count** means how many frames are used for calibration. It is recommend to gather enough data that contains
the whole surrounding 360 degrees.

**gather_skip** means the skip count in gathering. Only frames with index % gather_skip == 0 will be used for calibration.

For example, in the Horizon-Horizon case, gather_count = 200, gather_skip = 5 and lidar data frequency is 10Hz,
it means that the calibration program will save # 0 frame and drop #1-#4, save #5 and drop #6-#9 ....
It will gather 200 frames, which contains data lasting about 20 seconds (200 * (1 / 10) = 20) and 40 frames (200 / 5).
In the horizon-horizon dataset, 20 seconds with 40 frames contains enough data to register LiDAR#0 and LiDAR#1, and will not lasting
too long when optimizing (larger number of frames means longer optimizing time).

For your own dataset, it is advertised to keep a low rotating speed when acquiring, because aggressive motion will cause
motion blur for LiDAR points and resulting bad calibration results.

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).


