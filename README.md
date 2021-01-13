# Lidar-Lidar Calibration
Tools for testing extrinsic calibration between two rigidly mounted Lidar sensors on a ground vehicle for [DARPA Sub-T Challenge](https://www.subtchallenge.com/)

# Overview
This repository contains the set of scripts and tools to run and evaluate several different methods used to determine the extrinsic calibration between multiple rigidly mounted LiDAR sensors, as described in the included [paper](https://github.com/mschoder/lidar-lidar-calibration/blob/main/lidar-lidar-calibration-report.pdf). 

# Dependencies 
In addition to the specific external libraries linked in the `Methods` section below, some dependencies required to run the full suite of methods are as follows, tested on Ubuntu 18.04:

- MATLAB (tested with version 2020a) for evaluation and visualization
- ROS (melodic) with standard python dependencies
- Point Cloud Library (PCL) for NDT method
- [OctoMap](https://octomap.github.io/) for point cloud visualization

```
sudo apt-get install ros-melodic-octomap
sudo apt-get install libpcl-dev
sudo apt-get install pcl-tools
```

# Methods Evaluated

* Tsai-Lenz - an implementation of the foundational approach described in the 1989 paper [A new technique for fully autonomous and efficient 3D robotics hand/eye calibration](https://ieeexplore.ieee.org/document/34770), based on [Zoran Lazarevic's Matlab code](http://lazax.com/www.cs.columbia.edu/~laza/html/Stewart/matlab/handEye.m)
* [ETHZ Hand Eye Calibration Library](https://github.com/ethz-asl/hand_eye_calibration)
* [Lidar-Align Library](https://github.com/ethz-asl/lidar_align)
* [Normal Distributions Transform](https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/normal_distributions_transform.rstk) - [Implementation](https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html) from PCL

# Detailed Instructions
### Data Preprocessing
Preprocessing scripts to convert raw data from rosbag format to csv are located in `scripts/preprocessing`. Use the script `odom_to_csv.py` for Tsai-Lenz method and for use with ETHZ Hand-Eye Libarary. Use `odom_to_csv_la.py` for use with the Lidar-Align libarary.
```
python scripts/preprocessing/odom_to_csv.py --bag ./data/front_lidar_locus.bag --csv_output_file ./data/front_poses_timestamped.csv
```
This outputs a csv with columns formatted as: `t, x, y, z, q_x, q_y, q_z, q_w`

### Tsai-Lenz Method
Main script is `runHandEye.m`. This performs a few steps which can be customized:
- Load odometry data from csv format in preprocessing step to a 3D matrix in `load_odom_data.m` function (ensure paths match your system setup here)
- Sets parameters:
    - `consecutive` (true/false): whether to use consecutive timestamp pose pairs only (n-1 pairs) or all combinations of pose pairs ( n*(n-1) pairs). This is customizable for other pose selection as well
    - `use_nth_pose`: only use every nth pose pair to reduce size of computation and matrix inversion. Set as 1 for default behavior to use all pairs produced
- Script then calls Tsai-Lenz implementation via `handEye.m` and associated helper functions
- Output is a rigid calibration transformation in the form of a translation vector `[x y z]` and a rotation quaternion `[qw qx qy qz]`. This is the transformation from input frame 1 to input frame 2 (main to front in the example)


### ETHZ Hand Eye Calibration Library
- Clone [ETHZ Hand Eye Calibration Library](https://github.com/ethz-asl/hand_eye_calibration):

```
git clone git@github.com:ethz-asl/hand_eye_calibration.git
cd hand_eye_calibration
[Follow build/install instructions in library's readme]
```
- Use preprocessing output (above) as input to time alignment step in ETHZ library:
```
rosrun hand_eye_calibration compute_aligned_poses.py \
  --poses_B_H_csv_file  ../data/main_poses_timestamped.csv \
  --poses_W_E_csv_file ../data/front_poses_timestamped.csv \
  --aligned_poses_B_H_csv_file ../data/main_aligned.csv \
  --aligned_poses_W_E_csv_file ../data/front_aligned.csv \
  --time_offset_output_csv_file ../he_results/time_offset_MF.csv \
  --visualize True
  ```
- Run Dual Quaternion based hand-eye calibration as described in [ETHZ Paper](www.fsr.ethz.ch/papers/FSR_2017_paper_73.pdf):
```
rosrun hand_eye_calibration compute_hand_eye_calibration.py \
    --aligned_poses_B_H_csv_file ../data/main_aligned.csv  \
    --aligned_poses_W_E_csv_file ../data/front_aligned.csv \
    --time_offset_input_csv_file ../he_results/time_offset_MF.csv \
    --calibration_output_json_file ../he_results/calibration_MF.json \
    --visualize True \
    --plot_every_nth_pose 100
```

- Run batch optimization to refine calibration estimate, using dual-quaternion solution as an initial estimate:
```
rosrun hand_eye_calibration_batch_estimation batch_estimator \
   --v 1 \
   --pose1_csv ../data/main_poses_timestamped.csv \
   --pose2_csv ../data/front_poses_timestamped.csv \
   --init_guess_file ../he_results/calibration_MF.json \
   --output_file ../he_results/calibration_optimized_MF.json
```

### Lidar Align
- Clone [ETHZ Lidar Align Library](https://github.com/ethz-asl/lidar_align)
```
git clone git@github.com:ethz-asl/lidar_align.git
cd lidar_align
[follow build/install instructions in library's readme]
```

- Use `odom_to_csv_la.py` preprocessing script using steps described above to create properly formatted input csvs for this libarry
- Follow steps in library readme to run
- Visualize using `scripts/visualize_la.py` to plot output pointcloud of point correspondences (requires `open3d` dependency)


### Normal Distributions Transform
- Ensure the Point Cloud Library (PCL) is installed
- Identify the desired timestep from which to compute a transform (using a timestep with zero translational and rotational velocity is preferred to minimize error due to movement between frames)
- Use the following command to convert the desired topic containing LIDAR data from each sensor's bag to pointcloud data files. This will generate a `.pcd` file for each timestep in the bag file
```
rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
```
- Determine a "target cloud" and an "input cloud". The output of the normal distributions tranform will be the tranform from the input cloud to the target cloud.
- Within the `normal_distributions_tranform.cpp` file, edit the `target_cloud` and `input_cloud` paths to point to the `.pcd` files at the correct timesteps
- Edit the "initial guess" within `normal_distributions_tranform.cpp` to reflect the correct iniial guess
- Various other parameters can be tuned here if a solution does not converge
- Run the following command to generate the desired transform between the target and input pointclouds:
```
rosrun normal_distributions_transform normal_distributions_transform 
```
- A new window will pop-up after the transform has been generated allowing the user to visualize the targeting and input clouds (you may need to zoom out to properly view the clouds)

## Evaluation Criteria
- Main evaluation script is `eval_alignment.m` which computes pose errors for all calibration result inputs, and provides a set of plots showing 2D and 3D trajectory alignment of the two sensors and relative error. The script relies on one of two functions:
    - `eval_calib.m` - function to evalutate relative pose error (this is the implemented default)
    - `eval_calib_nonrel.m` - similar function but evaluates absolute pose error, and affected by odometric drift
    - Both above functions retunn a data structure containing time series of rotation and translation error at each aligned timestep, as well as the RMSE for series.

Other utilities included: 
- `husky_conversions.m` - conversions from RPY to quaternion representation for initial hand-calibration estimate provided in YAML file. Also provides example conversions math used for Lidar-Align and NDT inputs/results
- `load_Tcal.m` - Utility to load a caibration estimate from a standard format JSON file (example provided in `/results` directory) for comparison of baseline tranform and different methods' results 

## Visualization Tools
- Ensure Octomap is [properly installed](https://github.com/OctoMap/octomap/wiki/Compilation-and-Installation-of-OctoMap)
- Review `tf_transform.cpp` to ensure that the topics are properly mapped to the correct topics for the desired rosbags

### Visualizing a Single Sensor
- Use `visuals.launch` to visualize a single sensor
- Change the `sensor` argument to reflect the desired sensor (front, main, rear, etc.)
- Adjust RVIZ settings as necessary to view desired information

### Visualizing Multiple Sensors Simultaneously
- Use `compare_bags.launch` to view to lidar point cloud maps simultaneously
- This is useful to qualitatively evaluating the quality of a transform between these two sensors
- rosbags may need to be merged such that there are two PointCloud2 topics (one for each sensor) and an odom topic. Use `bagmerge.py` (in the "scripts" directory) to merge the desired topics from different bags into a single bag.
- Change the `sensor1` and `sensor2` arguments to reflect the desired sensors for comparison
- Change the `sensor_cal` transform to the desired extrinsic sensor calibration
- The `max_range` and color can also be adjusted here to change the resultant octomap.
