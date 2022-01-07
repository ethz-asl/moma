import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from numpy import linalg
from numpy.linalg.linalg import inv

from scipy.spatial.transform import Rotation as R
from scipy.optimize import NonlinearConstraint
from scipy.optimize import minimize

import cv2

# Functions from @Mateen Ulhaq and @karlo
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

def to_homogeneous(v):
    return np.hstack((v, 1.0))
    
class Camera:
    # taken from https://github.com/demul/extrinsic2pyramid
    def __init__(self, K, resolution):
        self.K = K
        self.resolution = resolution
        self.extrinsic = np.eye(4)

    def extrinsic2pyramid(self, ax, color='r', focal_len_scaled=5, aspect_ratio=0.3):
        vertex_std = np.array([[0, 0, 0, 1],
                               [focal_len_scaled * aspect_ratio, -focal_len_scaled * aspect_ratio, focal_len_scaled, 1],
                               [focal_len_scaled * aspect_ratio, focal_len_scaled * aspect_ratio, focal_len_scaled, 1],
                               [-focal_len_scaled * aspect_ratio, focal_len_scaled * aspect_ratio, focal_len_scaled, 1],
                               [-focal_len_scaled * aspect_ratio, -focal_len_scaled * aspect_ratio, focal_len_scaled, 1]])
        vertex_transformed = vertex_std @ self.extrinsic.T
        meshes = [[vertex_transformed[0, :-1], vertex_transformed[1][:-1], vertex_transformed[2, :-1]],
                            [vertex_transformed[0, :-1], vertex_transformed[2, :-1], vertex_transformed[3, :-1]],
                            [vertex_transformed[0, :-1], vertex_transformed[3, :-1], vertex_transformed[4, :-1]],
                            [vertex_transformed[0, :-1], vertex_transformed[4, :-1], vertex_transformed[1, :-1]],
                            [vertex_transformed[1, :-1], vertex_transformed[2, :-1], vertex_transformed[3, :-1], vertex_transformed[4, :-1]]]
        ax.add_collection3d(
            Poly3DCollection(meshes, facecolors=color, linewidths=0.3, edgecolors=color, alpha=0.35))

    def project(self, points):
        proj_points, _ = cv2.projectPoints(points, 
                                           rvec=self.extrinsic[:3, :3], 
                                           tvec=self.extrinsic[:3, 3], 
                                           cameraMatrix=self.K, 
                                           distCoeffs=np.array([]))
        proj_points = proj_points.reshape((-1, 2))
        
        # remove points outside of the camera view
        mask = (proj_points[:, 0] >= 0) & (proj_points[:, 1] >=0) & (proj_points[:, 0] < self.resolution[0]) & (proj_points[:, 1] < self.resolution[1])
        proj_points = proj_points[mask, :] 

        # the returned variable is a list of N 2d points -> shape is (N, 1, 2) -> squeeze to (N, 2)
        return proj_points

    def customize_legend(self, list_label):
        list_handle = []
        for idx, label in enumerate(list_label):
            color = plt.cm.rainbow(idx / len(list_label))
            patch = Patch(color=color, label=label)
            list_handle.append(patch)
        plt.legend(loc='right', bbox_to_anchor=(1.8, 0.5), handles=list_handle)


class ValvePerceptionModel:
    def __init__(self, c=np.array([0.0, 0.0, 0.0]), r=0.12, v1=np.array([1.0, 0.0, 0.0]), v2=np.array([0.0, 1.0, 0.0]), k=3, delta=0.02):
        """
        c: center
        r: radius
        v1: first axis
        v2: second axis
        k: number of rims
        delta: distance center from wheel plane
        """
        self.c = c
        self.r = r
        self.v1 = v1
        self.v2 = v2
        self.k = k
        self.n = np.cross(v1, v2)
        self.delta = delta
        self.camera = None

        self.observed_center = None
        self.observed_points = None

        # the circle points
        N = 50 # number of points used for the discretization of the circle
        theta = np.linspace(0, 2.0 * np.pi, N)
        self.circle = np.zeros((3, N))      
        for i, th in enumerate(theta):
            self.circle[:, i] = self.c + self.delta * self.n +  r * self.v1 * np.cos(th) + r * self.v2 * np.sin(th)

        self.delta_angle = 2 * np.pi / self.k
        self.keypoints = np.zeros((3, k+1)) # 3 keypoints and the center
        self.keypoints[:, 0] = self.c
        for i in range(k):
            theta = 2 * i * np.pi / self.k
            self.keypoints[:, i+1] = self.c + self.delta * self.n + r * self.v1 * np.cos(theta) + r * self.v2 * np.sin(theta)
    
    def set_camera(self, camera):
        self.camera = camera

    def transform(self, dx=0.0, dy=0.0, dz=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        r = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
        t = np.array([dx, dy, dz])
        
        self.c = r @ self.c + t
        self.v1 = r @ self.v1 + t
        self.v2 = r @ self.v2 + t
        self.keypoints = ((r @ self.keypoints).T + t).T
        self.circle = ((r @ self.circle).T + t).T
        
        
    def draw(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d', proj_type='ortho')
        ax.scatter3D(self.keypoints[0, :], self.keypoints[1, :], self.keypoints[2, :])
        ax.plot3D(self.circle[0, :], self.circle[1, :], self.circle[2, :])
        
        if self.camera is not None: 
            self.camera.extrinsic2pyramid(ax, focal_len_scaled=0.3)
        
        set_axes_equal(ax)        

        if self.camera is not None:
            proj_circle = self.camera.project(self.circle)
            proj_keypoints = self.camera.project(self.keypoints)
            
            fig, ax = plt.subplots()
            ax.plot(proj_circle[:, 0], proj_circle[:, 1])
            ax.scatter(proj_keypoints[:, 0], proj_keypoints[:, 1])
            ax.set_xlim([0, self.camera.resolution[0]])
            ax.set_ylim([0, self.camera.resolution[1]])
            ax.set_aspect('equal')

        plt.show()

    def set_observed_keypoints(self, center, other):
        """
        center: the observed center pixel
        other: the other k observed keypoints
        """
        self.observed_center = center
        self.observed_points = other
    
    def residual_fun(self, x):
        C = x[:3]
        v1 = x[3:6]
        v2 = x[6:9]
        delta = x[9]
        r = x[10]

        K_inv = np.linalg.inv(self.camera.K)
        cp = K_inv @ to_homogeneous(self.observed_center)
        kp = [K_inv @ to_homogeneous(point) for point in self.observed_points]

        n = np.cross(v1, v2)
        P = []
        for i in range(self.k):
            theta = 2 * i * np.pi / self.k
            P.append(C + delta * n + r * v1 * np.cos(theta) + r * v2 * np.sin(theta))

        lambda_c = C[2] / cp[2]
        lambda_p = [P[i][2] / kp[i][2] for i in range(self.k)]

        dCx = lambda_c * cp[0] - C[0]
        dCy = lambda_c * cp[1] - C[1]

        dPx = np.array([lambda_p[i] * kp[i][0] - P[i][0] for i in range(self.k)])
        dPy = np.array([lambda_p[i] * kp[i][1] - P[i][1] for i in range(self.k)])

        residual = dCx * dCx + dCy * dCy
        residual += np.sum(np.square(dPx) + np.square(dPy))
        return residual

    def optimize(self):
        
        # define the constraint
        def constraint_fun(x):
            # r > 0
            # delta > 0
            # norm v1 = 1
            # norm v2 = 1
            # v1 perpendicular to v2
    
            C = x[:3]
            v1 = x[3:6]
            v2 = x[6:9]
            delta = x[9]
            r = x[10]

            con = np.zeros((5, ))
            con[0] = r
            con[1] = delta
            con[2] = np.linalg.norm(v1)
            con[3] = np.linalg.norm(v2)
            con[4] = np.sum(v1 * v2)

            return con

        lower_bound = np.array([0.0, 0.0, 1.0, 1.0, 0.0])
        upper_bound = np.array([np.inf, np.inf, 1.0, 1.0, 0.0])

        constraint = NonlinearConstraint(constraint_fun, lower_bound, upper_bound)

        initial_guess = np.zeros((11,))
        res = minimize(self.residual_fun, x0=initial_guess, constraints=constraint)
        return res

if __name__ == "__main__":
    k = 3
    valve = ValvePerceptionModel(r=0.1, k=k, delta=-0.02)
    valve.transform(dz=0.8, roll_deg=50, pitch_deg=40)
    #valve.transform(dz=0.8, roll_deg=0, pitch_deg=0)

    camera_intrinsics = np.array([[695.49, 0.0, 646.83],
                                  [0.0, 694.75, 367.62],
                                  [0.0, 0.0, 1.0]])
    camera_resolution = np.array([1280, 720])
    camera = Camera(camera_intrinsics, camera_resolution)
    
    valve.set_camera(camera)
    valve.draw()

    proj_keypoints = camera.project(valve.keypoints)
    valve.set_observed_keypoints(center=proj_keypoints[0, :],
                                 other=[proj_keypoints[i, :] for i in range(k)])
    res = valve.optimize()
    print(res)
