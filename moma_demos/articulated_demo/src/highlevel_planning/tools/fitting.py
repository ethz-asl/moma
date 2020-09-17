import numpy as np


def fit_line_3d(points, first_point_fixed=False):
    # Based on document on least squares fitting.

    num_points = points.shape[0]
    if first_point_fixed:
        start_point = points[0]
    else:
        start_point = 1.0/num_points * np.sum(points, axis=0)

    y = points - start_point

    m = np.zeros((3,3))
    for i in range(num_points):
        point = y[i]
        m += np.dot(point, point) * np.eye(3)
        m -= np.matmul(point.reshape(-1,1), np.array([point]))

    ew, ev = np.linalg.eig(m)

    min_idx = np.argmin(ew)

    return start_point, ev[:, min_idx]

def alt_fit_line_3d(points, first_point_fixed=False):

    # This is an alternative implementation to fit a line to 3D points
    # using singular value decomposition.

    num_points = points.shape[0]
    if first_point_fixed:
        start_point = points[0]
    else:
        start_point = 1.0/num_points * np.sum(points, axis=0)
    
    _, _, vv = np.linalg.svd(points - start_point)

    return start_point, vv[0]

