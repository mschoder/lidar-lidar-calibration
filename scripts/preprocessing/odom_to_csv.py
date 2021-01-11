#!/usr/bin/env python

## Script adapted from ETHZ-ASL hand-eye-calibration library
## for use with lidar sensor and nav_msgs/odometry message type
## https://github.com/ethz-asl/hand_eye_calibration/blob/5d5077572d650a5491040a4a90d98850df4cf068/hand_eye_calibration/bin/tf_to_csv.py
## Mike Schoder | 2020-11-14

import argparse
import math
import sys
import time

import numpy as np

import rosbag
import rospy
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import warnings


def write_transformation_to_csv_file(bag_file, csv_file_name):

  bag = rosbag.Bag(bag_file)

  init = True
  pose_counter = 0
  pose_frequency_estimated = 0.
  pose_interval_estimated = 0.
  pose_previous_timestamp = 0.
  start_time_message = rospy.Time()
  end_time_message = rospy.Time()
  csv_file = open(csv_file_name, 'w')

  print("Parsing bag messages...")

  for topic, msg, t in bag.read_messages():
    if topic == "/husky4/lo_frontend/odometry" and msg is not None:

      # Initialize start time to first successful message lookup.
      if init:
        start_time_message = msg.header.stamp
        init = False

      position = msg.pose.pose.position
      orientation = msg.pose.pose.orientation
      # Write to csv file.
      # quaternion = np.array(hamilton_quaternion)
      csv_file.write(
          str(msg.header.stamp.to_sec()) + ', ' +
          str(position.x) + ', ' + str(position.y) + ', ' +
          str(position.z) + ', ' + str(orientation.x) + ', ' +
          str(orientation.y) + ', ' + str(orientation.z) + ', ' +
          str(orientation.w) + '\n')

      # # Update end time.
      end_time_message = msg.header.stamp
      pose_counter += 1

  # Output final estimated frequency.
  if pose_counter > 3:
    tf_frequency_estimated = pose_counter / (
        end_time_message - start_time_message).to_sec()
    print("Final estimate of tf topic frequency: ", "{0:.2f}".format(
        tf_frequency_estimated), "Hz")

  print("Exported ", pose_counter, " odometry poses.")


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--bag', required=True, help='Rosbag to parse.')
  parser.add_argument(
      '--csv_output_file', required=True, help='Path to output csv file')

  args = parser.parse_args()

  print("tf_to_csv.py: export tf to csv from bag: ", args.bag, "...")

  write_transformation_to_csv_file(args.bag,
                                   args.csv_output_file)