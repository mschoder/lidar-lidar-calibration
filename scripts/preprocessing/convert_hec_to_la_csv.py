#!/usr/bin/env python

## Takes a file of type required by hand-eye-calibration library and outputs a file for lidar-align lib
# Input csv file format: t, x, y, z, q_x, q_y, q_z, q_w
# Output csv file format: 
    # 1   timestamp ns
    # 2 	vertex index (not used)
    # 3 	position x
    # 4 	position y
    # 5 	position z
    # 6 	orientation quaternion w
    # 7 	orientation quaternion x
    # 8 	orientation quaternion y
    # 9 	orientation quaternion z

import argparse 
import pandas as pd

def do_conversion(infile, outfile):
    
    df = pd.read_csv(infile, names=['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df['t'] = df['t'] * 1e9  # convert sec to ns
    df['vidx'] = 0
    outcols = ['t', 'vidx', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
    df = df[outcols]
    df.to_csv(outfile, header=False, index=False)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input_csv', required=True, help='Rosbag to parse.')
  parser.add_argument(
      '--output_csv', required=True, help='Path to output csv file')

  args = parser.parse_args()

  print("convert_hac_to_la_csv:  ", "...")

  do_conversion(args.input_csv, args.output_csv)