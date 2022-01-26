from copy import deepcopy
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from numpy import linalg
from numpy.core.fromnumeric import shape
from numpy.linalg.linalg import inv

from scipy.spatial.transform import Rotation as R
from scipy.optimize import NonlinearConstraint, LinearConstraint
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

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def to_homogeneous(v):
    if len(v.shape) > 1:
        return np.vstack((v, np.ones((v.shape[1],))))
    return np.hstack((v, 1.0))

def add_noise(v, low, high):
    return v + np.random.randint(low=low, high=high, size=v.shape)

def point_line_distance(p, l):
    return np.abs(np.dot(to_homogeneous(p), l))/np.sqrt(l[0]**2 + l[1]**2)

class Camera:
    # taken from https://github.com/demul/extrinsic2pyramid
    def __init__(self, K, D, resolution):
        self.K = K
        self.D = D
        self.resolution = resolution
        self.T = np.eye(4)
        self.M = self.K @ self.T[:3, :]

    def extrinsic2pyramid(self, ax, color='r', focal_len_scaled=5, aspect_ratio=0.3):
        vertex_std = np.array([[0, 0, 0, 1],
                               [focal_len_scaled * aspect_ratio, -focal_len_scaled * aspect_ratio, focal_len_scaled, 1],
                               [focal_len_scaled * aspect_ratio, focal_len_scaled * aspect_ratio, focal_len_scaled, 1],
                               [-focal_len_scaled * aspect_ratio, focal_len_scaled * aspect_ratio, focal_len_scaled, 1],
                               [-focal_len_scaled * aspect_ratio, -focal_len_scaled * aspect_ratio, focal_len_scaled, 1]])
        vertex_transformed = vertex_std @ self.T.T
        meshes = [[vertex_transformed[0, :-1], vertex_transformed[1][:-1], vertex_transformed[2, :-1]],
                            [vertex_transformed[0, :-1], vertex_transformed[2, :-1], vertex_transformed[3, :-1]],
                            [vertex_transformed[0, :-1], vertex_transformed[3, :-1], vertex_transformed[4, :-1]],
                            [vertex_transformed[0, :-1], vertex_transformed[4, :-1], vertex_transformed[1, :-1]],
                            [vertex_transformed[1, :-1], vertex_transformed[2, :-1], vertex_transformed[3, :-1], vertex_transformed[4, :-1]]]
        ax.add_collection3d(
            Poly3DCollection(meshes, facecolors=color, linewidths=0.3, edgecolors=color, alpha=0.35))

    def project(self, points):
        proj_points, _ = cv2.projectPoints(points, 
                                           rvec=self.T[:3, :3], 
                                           tvec=self.T[:3, 3], 
                                           cameraMatrix=self.K, 
                                           distCoeffs=self.D)
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

    def transform(self, dx=0.0, dy=0.0, dz=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        r = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
        t = np.array([dx, dy, dz])
        self.T[:3, :3] = r
        self.T[:3, 3] = t
        self.M = self.K @ self.T[:3, :]

    def set_transform(self, T):
        assert T.shape == (4, 4)
        self.T = T
        self.M = self.K @ self.T[:3, :]

class ValveFitter:
    def __init__(self, k) -> None:
        self.k = k
        self.observations = []
    
    def add_observation(self, camera, keypoints, weights):
        """
        keypoints: assumed to be ordered as center, other points on the circle
        """

        self.observations.append([camera, keypoints, weights])

    def triangulate(self):
        if len(self.observations) < 2:
            raise NameError("Two observations needed to triangulate points")

        M1 = self.observations[0][0].M
        p1 = self.observations[0][1].T
        
        M2 = self.observations[1][0].M
        p2 = self.observations[1][1].T  # dim = 2 x N

        N = p1.shape[1]
        assert N == p2.shape[1]
        points_3d = np.zeros((3, N))

        for i in range(N):
            A = np.zeros((6, 4))
            
            p1x = skew(to_homogeneous(p1[:, i]))
            p2x = skew(to_homogeneous(p2[:, i]))
            A[:3, :] = p1x @ M1
            A[3:6, :] = p2x @ M2
    
            u, s, vh = np.linalg.svd(A, full_matrices=True)

            # the solution is the eigenvector corresponding to the minimum non zero eigenvalue
            # which is the corresponding column in v
            p_3d = vh[-1, :].T

            print("Points 3d homogenous are: {}".format(p_3d))

            p_3d /= p_3d[3] # make the points homogeneous
            points_3d[:, i] = p_3d[:3] # save in unhomogeneous format
            print("Computed 3d points are: {}".format(points_3d))
        return points_3d    
        
    def triangulate_cv(self):
        if len(self.observations) < 2:
            raise NameError("Two observations needed to triangulate points")

        M1 = self.observations[0][0].M
        M2 = self.observations[1][0].M
        p1 = self.observations[0][1].T
        p2 = self.observations[1][1].T
        
        p_3d = cv2.triangulatePoints(projMatr1=M1, projMatr2=M2, projPoints1=p1, projPoints2=p2, )
        return p_3d[:3, :]

    def _pnp(self, radius_hypotesis):
        points_3d =  np.zeros((self.k+1, 3))
        for i in range(self.k):
            theta = 2 * i * np.pi / self.k
            points_3d[i+1, :] = radius_hypotesis * np.array([1.0, 0.0, 0.0]) * np.cos(theta) + radius_hypotesis * np.array([0.0, 1.0, 0.0]) * np.sin(theta)
        
        camera_matrix = self.observations[0][0].K
        points_2d = self.observations[0][1]
        success, rotation_vector, translation_vector = cv2.solvePnP(points_3d, points_2d, camera_matrix, np.zeros((4,1)))
        
        # print(success)
        # print(rotation_vector)
        # print(translation_vector)

        T_valve_cam = np.eye(4)
        T_valve_cam[:3, :3] = R.from_rotvec(rotation_vector.reshape(-1)).as_matrix()
        T_valve_cam[:3, 3] = translation_vector.reshape(-1)

        T_cam_valve = np.linalg.inv(T_valve_cam)
        
        c = T_cam_valve[:3, 3]
        v1 = T_cam_valve[:3, 0]
        v2 = T_cam_valve[:3, 1]
        
        r = radius_hypotesis
        return ValveModel(c=c, r=r, v1=v1, v2=v2, k=self.k, delta=0.0)

        
    def residual_fun(self, x):
        v1 = x[3:6]
        v2 = x[6:9]
        delta = x[9]
        r = x[10]
        n = np.cross(v1, v2)
        
        C = to_homogeneous(x[:3]) 
        P = []
        for i in range(self.k):
            theta = 2 * i * np.pi / self.k
            P.append(C[:3] + delta * n + r * v1 * np.cos(theta) + r * v2 * np.sin(theta))
            P[i] = to_homogeneous(P[i])

        residual = 0.0
        for cam, kpts, w in self.observations:
            proj_c_hom = cam.K @ cam.T[:3, :] @  C 
            proj_p_hom = [cam.K @ cam.T[:3, :] @  P[i] for i in range(self.k)]

            proj_c = proj_c_hom[:2] / proj_c_hom[2]
            proj_p = [proj_p_hom[i][:2] / proj_p_hom[i][2] for i in range(self.k)]

            residual += w[0] * np.linalg.norm(proj_c - kpts[0, :])
            residual += sum([w[i+1] * np.linalg.norm(proj_p[i] - kpts[i+1, :]) for i in range(self.k)])
        return residual

    def optimize(self, method='non_linear', *args, **kwargs):
        if method =='non_linear':
            return self._non_linear_optimization()
        elif method == 'triangulation':
            return self._triangulation()
        elif method == 'pnp':
            return self._pnp(kwargs['radius_hypothesis'])
        else:
            raise NameError(f"Unknown method {method}")
        
    def _triangulation(self):
        points_3d = self.triangulate()
        C = points_3d[:, 0]

        # get normal from 3 keypoints
        P1 = points_3d[:, 1]
        P2 = points_3d[:, 2]
        P3 = points_3d[:, 3]
        n = np.cross((P2-P1), (P3-P1))
        n = n / np.linalg.norm(n)
        
        # get geometric center as the mean
        k = points_3d.shape[1] - 1
        Cg = np.sum(points_3d[:, 1:], axis=1) / k

        # get v1 and v2 from Cg and P1, v2 from n and v1
        v1 = P1 - Cg
        v1 = v1 / np.linalg.norm(v1)
        v2 = np.cross(n, v1)

        # get delta as the distance along n between Cg and C
        delta = np.sum(n * (Cg - C)) # dot product

        # get radius 
        r  = np.sum([np.linalg.norm(points_3d[:, i+1]- Cg) for i in range(k)]) / k

        return ValveModel(c=C, r=r, v1=v1, v2=v2, k=k, delta=delta)

    def _non_linear_optimization(self):
        
        # define the constraint
        def constraint_fun(x):
            # delta > 0
            # norm v1 = 1
            # norm v2 = 1
            # v1 perpendicular to v2
    
            v1 = x[3:6]
            v2 = x[6:9]
    
            con = np.zeros((3, ))
            con[0] = np.linalg.norm(v1)
            con[1] = np.linalg.norm(v2)
            con[2] = np.sum(v1 * v2)
            return con
        
        # Make sure vectors are normal and orthogonal
        vector_constraint = NonlinearConstraint(constraint_fun, np.array([1.0, 1.0, 0.0]), np.array([1.0, 1.0, 0.0]))

        # Make sure radius and delta are positive and withing prior bounds
        A = np.zeros((2, 11))
        A[0, 9] = 1
        A[1, 10] = 1
        geometry_constraint = LinearConstraint(A, np.array([0.0, 0.05]), np.array([0.03, 0.2]))
        

        initial_guess = np.zeros((11,))
        initial_guess[:3] = np.array([0.0, 0.0, 0.5])
        initial_guess[3:6] = np.array([1.0, 0.0, 0.0])
        initial_guess[6:9] = np.array([0.0, 0.0, 1.0])
        initial_guess[9] = 0.0
        initial_guess[10] = 0.1
    
        res = minimize(self.residual_fun, x0=initial_guess, constraints=[vector_constraint, geometry_constraint], options={'maxiter': 10e4})
        print(res)
        print(self.res_to_string(res.x))
        return self.valve_from_x(res.x)

    @staticmethod
    def res_to_string(x):
        print(f"""
center = {x[:3]}
axis 1 = {x[3:6]}
axis 2 = {x[6:9]}
delta = {x[9]}
radius = {x[10]}
""")

    def valve_from_x(self, x):
        c = x[:3]
        v1 = x[3:6]
        v2 = x[6:9]
        delta = x[9]
        r = x[10]
        return ValveModel(c=c, r=r, v1=v1, v2=v2, k=self.k, delta=delta)

class ValveModel:
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

        self.observations = []
        
        # the circle points
        N = 50 # number of points used for the discretization of the circle
        theta = np.linspace(0, 2.0 * np.pi, N)
        self.circle = np.zeros((3, N))      
        for i, th in enumerate(theta):
            self.circle[:, i] = self.c + self.delta * self.n +  r * self.v1 * np.cos(th) + r * self.v2 * np.sin(th)

        self.delta_angle = 2 * np.pi / self.k
        self.keypoints = np.zeros((3, k+1)) # k keypoints and the center
        self.keypoints[:, 0] = self.c
        for i in range(k):
            theta = 2 * i * np.pi / self.k
            self.keypoints[:, i+1] = self.c + self.delta * self.n + r * self.v1 * np.cos(theta) + r * self.v2 * np.sin(theta)

    def transform(self, dx=0.0, dy=0.0, dz=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        r = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
        t = np.array([dx, dy, dz])
        
        self.c = r @ self.c + t
        self.v1 = r @ self.v1
        self.v2 = r @ self.v2
        self.n = np.cross(self.v1, self.v2)
        self.keypoints = ((r @ self.keypoints).T + t).T
        self.circle = ((r @ self.circle).T + t).T
        
    def __str__(self):
        return f"""
center = {self.c}
axis 1 = {self.v1}
axis 2 = {self.v2}
delta = {self.delta}
radius = {self.r}
"""

class FeatureMatcher:
    def __init__(self, acceptance_ratio=np.inf):
        self.acceptance_ratio = acceptance_ratio
    
    @staticmethod
    def _compute_epilines(camera1, camera2, points1, points2):
        # https://docs.opencv.org/4.x/da/de9/tutorial_py_epipolar_geometry.html
        T_cam2_cam1 = np.linalg.inv(camera1.T) @ camera2.T
        E =  skew(T_cam2_cam1[:3, 3]) @ T_cam2_cam1[:3, :3]
        F = np.linalg.inv(camera1.K).T @ E @ np.linalg.inv(camera2.K)  
        
        # get the epipolar lines in the second image
        lines2 = cv2.computeCorrespondEpilines(points1.reshape(-1,1,2), 1, F)
        return lines2.reshape(-1,3)

    def match(self, camera1, camera2, points1, points2):
        lines = self._compute_epilines(camera1, camera2, points1, points2)
        points1 = points1.reshape((-1, 2))
        points2 = points2.reshape((-1, 2))
        N = points1.shape[0]
        assert N == len(lines) and "Need as many epipolar lines as keypoints"
        
        distances = []
        for l in lines:
            distances.append([point_line_distance(points2[i, :], l) for i in range(N)]) 

        # find first and second best matches for each line
        matches = -np.ones(N)
        for i, d in enumerate(distances):
            d_np = np.asarray(d)
            min1 = np.min(d_np)

            
            d_np2 = np.delete(d_np, d_np.argmin())
            min2 = np.min(d_np2)

            if (min1 / min2) < self.acceptance_ratio:
                point2_indx = d_np.argmin()
                point1_idx = i

                # point in image 2 at point2_idx matches point in image 1 at point1_idx
                matches[point2_indx] = point1_idx 
        return matches

    def draw_epilines(self, camera1, camera2, points1, points2):
        lines = self._compute_epilines(camera1, camera2, points1, points2)
        fig, ax = plt.subplots(2, 1)

        import matplotlib.colors as mcolors
        colors = list(mcolors.BASE_COLORS.keys())

        for i in range(points1.shape[0]):
            ax[0].scatter(points1[i, 0], points1[i, 1], label=f'point {i}', c=colors[i])
            ax[1].scatter(points2[i, 0], points2[i, 1], label=f'point {i}', c=colors[i])

        c, _ = camera2.resolution 
        for i, r in enumerate(lines):
            color = colors[i]
            x0,y0 = map(int, [0, -r[2]/r[1] ])
            x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
            ax[1].plot([x0, x1], [y0, y1], color=color, label=f"epiline {i}")

        ax[0].set_xlim([0, camera1.resolution[0]])
        ax[0].set_ylim([0, camera1.resolution[1]])
        ax[1].set_xlim([0, camera2.resolution[0]])
        ax[1].set_ylim([0, camera2.resolution[1]])
        ax[0].grid(True)
        ax[1].grid(True)
        ax[0].set_title("camera 1")
        ax[1].set_title("camera 2")
        ax[0].legend()
        ax[1].legend()

class ValveVisualizer:
    def __init__(self):
        self.fig_3d = plt.figure()
        self.ax_3d = plt.axes(projection='3d', proj_type='ortho')
        
        self.fig_cam, self.ax_cam = plt.subplots()
        self.cameras = []
        self.valves = []
        self.labels = []
        self.points = []
        self.points_labels = []

    def add_item(self, camera, valve, label):
        self.cameras.append(camera)
        self.valves.append(valve)
        self.labels.append(label)
    
    def add_points(self, points, points_label):
        self.points.append(points)
        self.points_labels.append(points_label)
    
    def draw(self):
        for camera, valve, label in zip(self.cameras, self.valves, self.labels):
            self.ax_3d.scatter3D(valve.keypoints[0, :], valve.keypoints[1, :], valve.keypoints[2, :], label=label)
            self.ax_3d.plot3D(valve.circle[0, :], valve.circle[1, :], valve.circle[2, :], label=label)
  
            camera.extrinsic2pyramid(self.ax_3d, focal_len_scaled=0.3)

            proj_circle = camera.project(valve.circle)
            proj_keypoints = camera.project(valve.keypoints)
            
            self.ax_cam.plot(proj_circle[:, 0], proj_circle[:, 1], label=label)
            self.ax_cam.scatter(proj_keypoints[:, 0], proj_keypoints[:, 1])
            self.ax_cam.set_xlim([0, camera.resolution[0]])
            self.ax_cam.set_ylim([0, camera.resolution[1]])
            self.ax_cam.set_aspect('equal')
        
        for p, l in zip(self.points, self.points_labels):
            if p.shape[0] == 3:
                self.ax_3d.scatter3D(p[0, :], p[1, :], p[2, :], label=l)
            elif p.shape[0] == 2:
                self.ax_cam.scatter(p[0, :], p[1, :], label=l)
                
        self.ax_cam.grid(True)
        self.ax_cam.legend()
        self.ax_3d.legend()
        set_axes_equal(self.ax_3d)        



if __name__ == "__main__":
    
    # define some camera views
    camera_intrinsics = np.array([[695.49, 0.0, 646.83],
                                  [0.0, 694.75, 367.62],
                                  [0.0, 0.0, 1.0]])
    camera_resolution = np.array([1280, 720])
    camera_distortion = np.array([])
    camera1 = Camera(camera_intrinsics, camera_distortion, camera_resolution)
    
    camera2 = deepcopy(camera1)
    camera2.transform(dz=0.4, dx=0.8, pitch_deg=-40)
    
    camera3 = deepcopy(camera1)
    camera3.transform(dx=0.4, pitch_deg=-40)

    
    # get ground truth valve
    k = 3
    valve = ValveModel(r=0.1, k=k, delta=-0.02)
    valve.transform(dz=0.8, roll_deg=50, pitch_deg=40)
    
    # get simulated observations
    proj_keypoints1 = camera1.project(valve.keypoints)
    proj_keypoints2 = camera2.project(valve.keypoints)
    proj_keypoints3 = camera3.project(valve.keypoints) 
    
    # define a problem, add observations and
    problem = ValveFitter(k=k)

    
    proj_keypoints1_n = add_noise(proj_keypoints1, low=0, high=20)
    proj_keypoints2_n = add_noise(proj_keypoints2, low=0, high=20)
    
    print("============================")
    print(proj_keypoints1_n)
    problem.add_observation(camera1, proj_keypoints1_n, np.array([0.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    problem.add_observation(camera2, proj_keypoints2_n, np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    #problem.add_observation(camera3, proj_keypoints3, np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    

    valve_opt = problem.optimize(method='non_linear')
    valve_trn = problem.optimize(method='triangulation')
    # valve_pnp = problem.optimize(method='pnp', radius_hypothesis=0.1)


    # try triangulation
    points_triangulated = problem.triangulate()
    

    # create visualizer to see points and projections
    visualizer = ValveVisualizer()
    visualizer.add_item(camera1, valve, "gt camera1")
    visualizer.add_item(camera2, valve, "gt camera 2")
    visualizer.add_item(camera1, valve_trn, "trn camera1")
    #visualizer.add_item(camera, valve_opt, "fit in camera")
    
    #visualizer.add_item(camera2, valve_opt, "fit opt in camera2")
    
    #visualizer.add_item(camera, valve_pnp, "fit pnp in camera")
    visualizer.add_points(points_triangulated, "triangulation")

    p1 = camera1.project(points_triangulated)
    p2 = camera2.project(points_triangulated)

    visualizer.add_points(p1.T, "backprojection 1")
    visualizer.add_points(p2.T, "backprojection 2")

    # by hand backprojection
    # p1_hom_test = camera.M @ to_homogeneous(valve.keypoints)
    # p2_hom_test = camera2.M @ to_homogeneous(valve.keypoints)

    # p1_test = p1_hom_test[:2] / p1_hom_test[2]
    # p2_test = p2_hom_test[:2] / p2_hom_test[2]
    
    # visualizer.add_points(p1_test, "backprop test 1")
    # visualizer.add_points(p2_test, "backprop test 2")
    
    #visualizer.add_item(camera3, valve, "gt camera 3")
    #visualizer.add_item(camera3, valve_opt, "fit in camera3")
    
    visualizer.draw()


    matcher = FeatureMatcher()
    matcher.draw_epilines(camera1, camera2, proj_keypoints1_n, proj_keypoints2_n)

    print("proj keypoints before swap")
    print(proj_keypoints2)

    # swap keypoints to test matching strategy
    proj_keypoints2_n[[0, 1, 2, 3], :] = proj_keypoints2_n[[3, 0, 2, 1], :] 
    
    # point 1 is the closest to line 0 == point0 in first image
    print("proj keypoints after swap")
    print(proj_keypoints2)
    
    matches = matcher.match(camera1, camera2, proj_keypoints1_n, proj_keypoints2_n)

    # reshuffle keypoints to match
    # map 0, 1, 2, 3 to their location in the original image
    proj_keypoints2_n[matches.astype(int), :] = proj_keypoints2_n[[0, 1, 2, 3], :] 
    print("matches are")
    print(matches)
    print("proj keypoints after match")
    print(proj_keypoints2)
    plt.show()
    
