# Lidar-Lidar Calibration
#### Tools for testing extrinsic calibration between two rigidly mounted Lidar sensors on a ground vehicle

# Overview
This repository contains the set of scripts and tools to run and evaluate several different methods used to determine the extrinsic calibration between multiple rigidly mounted LiDAR sensors, as described in the included [paper](link). 

# Dependencies 
In addition to the specific external libraries linked in the `Methods` section below, some dependencies required to run the full suite of methods are as follows, tested on Ubuntu 18.04:

- MATLAB (tested with version 2020a) for evaluation and visualization
- ROS (melodic) with standard python dependencies
- Point Cloud Library (PCL) for NDT method
- OctoMap for point cloud visualization

```
sudo apt-get install ros-melodic-octomap
sudo apt-get install libpcl-dev
sudo apt-get install pcl-tools
```

# Methods Evaluated

* Tsai-Lenz - an implementation of the foundational approach described in the 1989 paper [A new technique for fully autonomous and efficient 3D robotics hand/eye calibration](https://ieeexplore.ieee.org/document/34770), based on [Zoran Lazarevic's Matlab code](http://lazax.com/www.cs.columbia.edu/~laza/html/Stewart/matlab/handEye.m)
* [ETHZ Hand Eye Calibration Library](https://github.com/ethz-asl/hand_eye_calibration)
* [Lidar-Align Library](https://github.com/ethz-asl/lidar_align)
* [Normal Distributions Transform](linhttps://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/normal_distributions_transform.rstk) - Implementation from PCL

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
[TODO - Phil]





## Evaluation Criteria
- Main evaluation script is `eval_alignment.m` which computes pose errors for all calibration result inputs, and provides a set of plots showing 2D and 3D trajectory alignment of the two sensors and relative error. The script relies on one of two functions:
    - `eval_calib.m` - function to evalutate relative pose error (this is the implemented default)
    - `eval_calib_nonrel.m` - similar function but evaluates absolute pose error, and affected by odometric drift
    - Both above functions retunn a data structure containing time series of rotation and translation error at each aligned timestep, as well as the RMSE for series.

Other utilities included: 
- `husky_conversions.m` - conversions from RPY to quaternion representation for initial hand-calibration estimate provided in YAML file. Also provides example conversions math used for Lidar-Align and NDT inputs/results
- `load_Tcal.m` - Utility to load a caibration estimate from a standard format JSON file (example provided in `/results` directory) for comparison of baseline tranform and different methods' results 



## Visualization Tools
- [TODO] Description and instructions for running Octomap visualization
