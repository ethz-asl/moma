#!/usr/bin/env python3

import argparse
import open3d as o3d


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("pcdfile", type=str)
    parser.add_argument("pcdfileout", type=str)
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.pcdfile)
    print(pcd)
    pcd = pcd.voxel_down_sample(voxel_size=0.25)
    print(pcd)
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud(args.pcdfileout, pcd)
