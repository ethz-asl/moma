import open3d as o3d
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])

print("1. Load two point clouds and show initial pose")
# source = o3d.io.read_point_cloud("/root/data/run_1/rs_435_2.pcd")
# target = o3d.io.read_point_cloud("/root/data/run_1/rs_435_3.pcd")
# source = o3d.io.read_point_cloud("/root/data/run_2/rs_435_2.pcd")
# target = o3d.io.read_point_cloud("/root/data/run_2/rs_435_3.pcd")
source = o3d.io.read_point_cloud("/root/data/run_3/rs_435_2.pcd")
target = o3d.io.read_point_cloud("/root/data/run_3/rs_435_3.pcd")

# draw initial alignment
translation = [-0.340, -0.096, 0.014]  # Example translation
quaternion = [-0.028, 0.063, 0.997, -0.039]  # Example quaternion (x, y, z, w)
rotation_matrix = R.from_quat(quaternion).as_matrix()
current_transformation = np.eye(4)
current_transformation[:3, :3] = rotation_matrix
current_transformation[:3, 3] = translation
# current_transformation = np.identity(4)

draw_registration_result_original_color(source, target, current_transformation)

# initial_pcd = o3d.geometry.PointCloud(source)  # Copy using the constructor
# initial_pcd = initial_pcd.transform(current_transformation)
# draw_registration_result_original_color(source, target, current_transformation)

# # point to plane ICP
# print("2. Point-to-plane ICP registration is applied on original point")
# print("   clouds to refine the alignment. Distance threshold 0.02.")
# result_icp = o3d.pipelines.registration.registration_icp(
#     source, target, 0.02, current_transformation,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint())
# print(result_icp)
# draw_registration_result_original_color(source, target,
#                                         result_icp.transformation)

print("3. Colored point cloud registration")
voxel_radius = [0.04, 0.02, 0.01, 0.05]
max_iter = [50, 30, 14, 50]
for scale in range(3):
    iter = max_iter[scale]
    radius = voxel_radius[scale]
    print([iter, radius, scale])

    print("3-1. Downsample with a voxel size %.2f" % radius)
    source_down = source.voxel_down_sample(radius)
    target_down = target.voxel_down_sample(radius)

    print("3-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    print("3-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iter))
    
    # result_icp = o3d.pipelines.registration.registration_icp(
    #     source_down, target_down, 0.02, current_transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())


    current_transformation = result_icp.transformation
    print(result_icp)
draw_registration_result_original_color(source, target,
                                        result_icp.transformation)
