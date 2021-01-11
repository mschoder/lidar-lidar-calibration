## Must be running python3 for this

import numpy as np
import open3d as o3d

def main():
    # cloud = o3d.io.read_point_cloud("/home/mschoder/vnav_proj/data/la_data/main-front-fused-002.ply") # Read the point cloud
    cloud = o3d.io.read_point_cloud("/Users/mschoder/vnav/TEAM1-VNAV-project/data/la_data/test_002.ply") 
    o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud     

if __name__ == "__main__":
    main()
