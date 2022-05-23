#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from copy import deepcopy

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from matplotlib.patches import Patch
from moma_mission.missions.piloting.valve_model import ValveModel
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.optimize import LinearConstraint
from scipy.optimize import minimize
from scipy.optimize import NonlinearConstraint
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo

# Functions from @Mateen Ulhaq and @karlo


def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array(
        [
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ]
    )
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)


def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def to_homogeneous(v):
    if len(v.shape) > 1:
        return np.vstack((v, np.ones((v.shape[1],))))
    return np.hstack((v, 1.0))


def add_noise(v, low, high):
    return v + np.random.randint(low=low, high=high, size=v.shape)


def point_line_distance(p, l):
    return np.abs(np.dot(to_homogeneous(p), l)) / np.sqrt(l[0] ** 2 + l[1] ** 2)


class Camera:
    # taken from https://github.com/demul/extrinsic2pyramid
    def __init__(self, K=np.zeros((3, 3)), D=[], resolution=[600, 300]):
        self.K = K
        self.D = D
        self.resolution = resolution
        self.T = np.eye(4)
        self.M = self.K @ self.T[:3, :]

    def extrinsic2pyramid(self, ax, color="r", focal_len_scaled=5, aspect_ratio=0.3):
        vertex_std = np.array(
            [
                [0, 0, 0, 1],
                [
                    focal_len_scaled * aspect_ratio,
                    -focal_len_scaled * aspect_ratio,
                    focal_len_scaled,
                    1,
                ],
                [
                    focal_len_scaled * aspect_ratio,
                    focal_len_scaled * aspect_ratio,
                    focal_len_scaled,
                    1,
                ],
                [
                    -focal_len_scaled * aspect_ratio,
                    focal_len_scaled * aspect_ratio,
                    focal_len_scaled,
                    1,
                ],
                [
                    -focal_len_scaled * aspect_ratio,
                    -focal_len_scaled * aspect_ratio,
                    focal_len_scaled,
                    1,
                ],
            ]
        )
        vertex_transformed = vertex_std @ self.T.T
        meshes = [
            [
                vertex_transformed[0, :-1],
                vertex_transformed[1][:-1],
                vertex_transformed[2, :-1],
            ],
            [
                vertex_transformed[0, :-1],
                vertex_transformed[2, :-1],
                vertex_transformed[3, :-1],
            ],
            [
                vertex_transformed[0, :-1],
                vertex_transformed[3, :-1],
                vertex_transformed[4, :-1],
            ],
            [
                vertex_transformed[0, :-1],
                vertex_transformed[4, :-1],
                vertex_transformed[1, :-1],
            ],
            [
                vertex_transformed[1, :-1],
                vertex_transformed[2, :-1],
                vertex_transformed[3, :-1],
                vertex_transformed[4, :-1],
            ],
        ]
        ax.add_collection3d(
            Poly3DCollection(
                meshes, facecolors=color, linewidths=0.3, edgecolors=color, alpha=0.35
            )
        )

    def project(self, points):
        proj_points, _ = cv2.projectPoints(
            points,
            rvec=self.T[:3, :3],
            tvec=self.T[:3, 3],
            cameraMatrix=self.K,
            distCoeffs=self.D,
        )
        proj_points = proj_points.reshape((-1, 2))

        # remove points outside of the camera view
        mask = (
            (proj_points[:, 0] >= 0)
            & (proj_points[:, 1] >= 0)
            & (proj_points[:, 0] < self.resolution[0])
            & (proj_points[:, 1] < self.resolution[1])
        )
        proj_points = proj_points[mask, :]

        # the returned variable is a list of N 2d points -> shape is (N, 1, 2) -> squeeze to (N, 2)
        return proj_points

    def customize_legend(self, list_label):
        list_handle = []
        for idx, label in enumerate(list_label):
            color = plt.cm.rainbow(idx / len(list_label))
            patch = Patch(color=color, label=label)
            list_handle.append(patch)
        plt.legend(loc="right", bbox_to_anchor=(1.8, 0.5), handles=list_handle)

    def transform(
        self, dx=0.0, dy=0.0, dz=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0
    ):
        r = R.from_euler(
            "xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True
        ).as_matrix()
        t = np.array([dx, dy, dz])
        self.T[:3, :3] = r
        self.T[:3, 3] = t
        self.M = self.K @ self.T[:3, :]

    def set_transform(self, T):
        assert T.shape == (4, 4)
        self.T = T
        self.M = self.K @ self.T[:3, :]

    def set_intrinsics_from_camera_info(self, msg: CameraInfo):
        self.K = np.asarray(msg.K).reshape(3, 3)
        self.D = np.asarray(msg.D)
        self.resolution = np.array([msg.width, msg.height])
        self.M = self.K @ self.T[:3, :]

    def set_extrinsics_from_pose(self, msg: Pose):
        r = R.from_quat(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        ).as_matrix()
        t = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.T[:3, :3] = r
        self.T[:3, 3] = t
        self.M = self.K @ self.T[:3, :]


class ValveFitter:
    def __init__(self, num_spokes) -> None:
        self.num_spokes = num_spokes
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

            p_3d /= p_3d[3]  # make the points homogeneous
            points_3d[:, i] = p_3d[:3]  # save in unhomogeneous format
            print("Computed 3d points are: {}".format(points_3d))
        return points_3d

    def triangulate_cv(self):
        if len(self.observations) < 2:
            raise NameError("Two observations needed to triangulate points")

        M1 = self.observations[0][0].M
        M2 = self.observations[1][0].M
        p1 = self.observations[0][1].T
        p2 = self.observations[1][1].T

        p_3d = cv2.triangulatePoints(
            projMatr1=M1,
            projMatr2=M2,
            projPoints1=p1,
            projPoints2=p2,
        )
        return p_3d[:3, :]

    def _pnp(self, radius_hypotesis):
        points_3d = np.zeros((self.num_spokes + 1, 3))
        for i in range(self.num_spokes):
            theta = 2 * i * np.pi / self.num_spokes
            points_3d[i + 1, :] = radius_hypotesis * np.array([1.0, 0.0, 0.0]) * np.cos(
                theta
            ) + radius_hypotesis * np.array([0.0, 1.0, 0.0]) * np.sin(theta)

        camera_matrix = self.observations[0][0].K
        points_2d = self.observations[0][1]
        success, rotation_vector, translation_vector = cv2.solvePnP(
            points_3d, points_2d, camera_matrix, np.zeros((4, 1))
        )

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
        return ValveModel(
            center=c, radius=r, axis_1=v1, axis_2=v2, num_spokes=self.num_spokes
        )

    def residual_fun(self, x):
        v1 = x[3:6]
        v2 = x[6:9]
        delta = x[9]
        r = x[10]
        n = np.cross(v1, v2)

        C = to_homogeneous(x[:3])
        P = []
        for i in range(self.num_spokes):
            theta = 2 * i * np.pi / self.num_spokes
            P.append(
                C[:3] + delta * n + r * v1 * np.cos(theta) + r * v2 * np.sin(theta)
            )
            P[i] = to_homogeneous(P[i])

        residual = 0.0
        for cam, kpts, w in self.observations:
            proj_c_hom = cam.K @ cam.T[:3, :] @ C
            proj_p_hom = [cam.K @ cam.T[:3, :] @ P[i] for i in range(self.num_spokes)]

            proj_c = proj_c_hom[:2] / proj_c_hom[2]
            proj_p = [
                proj_p_hom[i][:2] / proj_p_hom[i][2] for i in range(self.num_spokes)
            ]

            residual += w[0] * np.linalg.norm(proj_c - kpts[0, :])
            residual += sum(
                [
                    w[i + 1] * np.linalg.norm(proj_p[i] - kpts[i + 1, :])
                    for i in range(self.num_spokes)
                ]
            )
        return residual

    def optimize(self, method="non_linear", *args, **kwargs):
        if method == "non_linear":
            return self._non_linear_optimization()
        elif method == "triangulation":
            points_3d = self.triangulate()
            return self.estimate_from_3d_points(points_3d)[1]
        elif method == "pnp":
            return self._pnp(kwargs["radius_hypothesis"])
        else:
            raise NameError(f"Unknown method {method}")

    def estimate_from_3d_points(
        self, points_3d, camera_pose, frame, handle_radius=0.0, error_threshold=0.003
    ):
        C = points_3d[:, 0]

        # get normal from 3 keypoints
        P1 = points_3d[:, 1]
        P2 = points_3d[:, 2]
        P3 = points_3d[:, 3]
        n = np.cross((P2 - P1), (P3 - P1))
        n = n / np.linalg.norm(n)

        # get geometric center as the mean
        k = points_3d.shape[1] - 1
        Cg = np.mean(points_3d[:, 1:], axis=1)

        camera_position = np.array(
            [camera_pose.position.x, camera_pose.position.y, camera_pose.position.z]
        )
        # Valve should always face towards camera,
        # such that the turning direction is consistent
        if np.dot(camera_position - Cg, n) < 0:
            n = -n

        # get v1 and v2 from Cg and P1, v2 from n and v1
        v1 = P1 - Cg
        v1 = v1 / np.linalg.norm(v1)
        v2 = np.cross(n, v1)

        # get delta as the distance along n between Cg and C
        delta = np.sum(n * (Cg - C))  # dot product

        # get radius
        radii = [np.linalg.norm(points_3d[:, i + 1] - Cg) for i in range(k)]
        r = np.mean(radii)
        r_with_handle = r + handle_radius

        # TODO validity check if n * (Cg - C) is roughly equal to (Cg - C)

        residual_error = max([abs(r - radius) for radius in radii])
        # we care about a keypoint being very off
        rospy.loginfo(f"Valve fitting residual error is {residual_error}")

        if residual_error <= error_threshold:
            return residual_error, ValveModel(
                frame=frame,
                center=C,
                radius=r_with_handle,
                axis_1=v1,
                axis_2=v2,
                num_spokes=k,
                depth=delta,
            )
        else:
            rospy.logerr("Could not fit the 3d points to a circular valve wheel")
            return np.inf, None

    def _non_linear_optimization(self):

        # define the constraint
        def constraint_fun(x):
            # delta > 0
            # norm v1 = 1
            # norm v2 = 1
            # v1 perpendicular to v2

            v1 = x[3:6]
            v2 = x[6:9]

            con = np.zeros((3,))
            con[0] = np.linalg.norm(v1)
            con[1] = np.linalg.norm(v2)
            con[2] = np.sum(v1 * v2)
            return con

        # Make sure vectors are normal and orthogonal
        vector_constraint = NonlinearConstraint(
            constraint_fun, np.array([1.0, 1.0, 0.0]), np.array([1.0, 1.0, 0.0])
        )

        # Make sure radius and delta are positive and withing prior bounds
        A = np.zeros((2, 11))
        A[0, 9] = 1
        A[1, 10] = 1
        geometry_constraint = LinearConstraint(
            A, np.array([0.0, 0.05]), np.array([0.03, 0.2])
        )

        initial_guess = np.zeros((11,))
        initial_guess[:3] = np.array([0.0, 0.0, 0.5])
        initial_guess[3:6] = np.array([1.0, 0.0, 0.0])
        initial_guess[6:9] = np.array([0.0, 0.0, 1.0])
        initial_guess[9] = 0.0
        initial_guess[10] = 0.1

        res = minimize(
            self.residual_fun,
            x0=initial_guess,
            constraints=[vector_constraint, geometry_constraint],
            options={"maxiter": 10e4},
        )
        print(res)
        print(self.res_to_string(res.x))
        return self.valve_from_x(res.x)

    @staticmethod
    def res_to_string(x):
        print(
            f"""
center = {x[:3]}
axis 1 = {x[3:6]}
axis 2 = {x[6:9]}
delta = {x[9]}
radius = {x[10]}
"""
        )

    def valve_from_x(self, x):
        c = x[:3]
        v1 = x[3:6]
        v2 = x[6:9]
        delta = x[9]
        r = x[10]
        return ValveModel(
            center=c,
            radius=r,
            axis_1=v1,
            axis_2=v2,
            num_spokes=self.num_spokes,
            depth=delta,
        )


class FeatureMatcher:
    def __init__(self, acceptance_ratio=np.inf):
        self.acceptance_ratio = acceptance_ratio

    @staticmethod
    def _compute_epilines(camera1, camera2, points1, points2):
        # https://docs.opencv.org/4.x/da/de9/tutorial_py_epipolar_geometry.html
        T_cam2_cam1 = np.linalg.inv(camera1.T) @ camera2.T
        E = skew(T_cam2_cam1[:3, 3]) @ T_cam2_cam1[:3, :3]
        F = np.linalg.inv(camera1.K).T @ E @ np.linalg.inv(camera2.K)

        # get the epipolar lines in the second image
        lines2 = cv2.computeCorrespondEpilines(points1.reshape(-1, 1, 2), 1, F)
        return lines2.reshape(-1, 3)

    def match(self, camera1, camera2, points1, points2, mask=None):
        lines = self._compute_epilines(camera1, camera2, points1, points2)
        points1 = points1.reshape((-1, 2))
        points2 = points2.reshape((-1, 2))
        print(f"Matching\n{points1}\n with points\n{points2}")
        N = points1.shape[0]
        assert N == len(lines) and "Need as many epipolar lines as keypoints"

        distances = []
        for l in lines:
            distances.append([point_line_distance(points2[i, :], l) for i in range(N)])

        distances = np.asarray(distances)
        print(f"distances from epipolar lines are:\n{distances}")

        # find closest epipolar line to the current point
        matches = -np.ones(N)
        for i in range(distances.shape[1]):

            if mask is not None and i in mask:
                matches[i] = i
                distances[i, :] = np.max(distances)
            else:
                idx_first_min = np.argmin(distances[:, i])
                first_min = distances[idx_first_min, i]

                distances[idx_first_min, :] = np.max(
                    distances
                )  # avoid this line to be found again
                idx_second_min = np.argmin(distances[:, i])
                second_min = distances[idx_second_min, i]

                if (first_min / second_min) < self.acceptance_ratio:
                    point2_idx = i
                    point1_idx = idx_first_min

                    # point in image 2 at point2_idx matches point in image 1 at point1_idx
                    matches[point1_idx] = point2_idx

        print(f"Matches are: {matches}")
        if len(np.unique(matches[matches > -1])) != len(matches):
            print("There is an ambiguity in the found mathces.")
            matches = -np.ones(N)
        return matches.astype(int)

    def draw_epilines(self, camera1, camera2, points1, points2):
        lines = self._compute_epilines(camera1, camera2, points1, points2)
        fig, ax = plt.subplots(2, 1)

        import matplotlib.colors as mcolors

        colors = list(mcolors.BASE_COLORS.keys())

        for i in range(points1.shape[0]):
            ax[0].scatter(points1[i, 0], points1[i, 1], label=f"point {i}", c=colors[i])
            ax[1].scatter(points2[i, 0], points2[i, 1], label=f"point {i}", c=colors[i])

        c, _ = camera2.resolution
        for i, r in enumerate(lines):
            color = colors[i]
            x0, y0 = map(int, [0, -r[2] / r[1]])
            x1, y1 = map(int, [c, -(r[2] + r[0] * c) / r[1]])
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


class RansacMatcher:
    """
    Filter 3d observations based on matches between sets of 2d observation.
    Takes one set as a reference observation to match agains others using the epipolar
    line constraint. The mathc also reorder observations such that they are paired consistently
    The observation with most matches and corresponindg matches is returned with the filter method

    """

    def __init__(self, acceptance_ratio=0.6, min_consensus=2, mask=None):
        self.feature_matcher = FeatureMatcher(acceptance_ratio=acceptance_ratio)
        self.mask = mask
        self.min_consensus = min_consensus
        self.cameras = []
        self.observations = []
        self.observations3d = []

    def add_observation(
        self, camera: Camera, observation: np.ndarray, observation3d: np.array
    ):
        self.cameras.append(camera)
        self.observations.append(observation)
        self.observations3d.append(observation3d)

    def filter(self):
        """Return the filtered and matched 2d and 3d features"""
        if len(self.observations) < self.min_consensus:
            raise NameError(
                "Cannot filter with less observations than minimum required consensus"
            )

        success = True
        max_matches = -1
        max_reference = 0
        best_matched_observations = None
        best_matched_observations3d = None
        best_matched_observations_cam = None

        for i, (cam_ref, obs_ref) in enumerate(zip(self.cameras, self.observations)):
            matched_total = 0
            matched_observations = deepcopy(self.observations)
            matched_observations3d = deepcopy(self.observations3d)
            matched_observations_cams = deepcopy(self.cameras)
            for j, (cam, obs, obs3d) in enumerate(
                zip(self.cameras, self.observations, self.observations3d)
            ):
                if j != i:
                    # first reorder features according the epipolar constraint
                    matches = self.feature_matcher.match(
                        cam_ref, cam, obs_ref, obs, mask=self.mask
                    )
                    # if some feature was not matched because of the acceptance ration the index is set to -1
                    if np.all(matches >= 0):
                        matched_observations[j] = obs[matches, :]
                        matched_observations3d[j] = obs3d[matches, :]
                        matched_observations_cams[j] = cam
                        matched_total += 1  # we mathced all features with respect to the reference observation
                    else:
                        matched_observations[j] = None
                        matched_observations3d[j] = None
                        matched_observations_cams[j] = None

            if matched_total > max_matches:
                max_reference = i
                max_matches = matched_total
                best_matched_observations = matched_observations
                best_matched_observations3d = matched_observations3d
                best_matched_observations_cam = matched_observations_cams
        max_matches += 1  # count also the reference observation as "match with itself"
        if max_matches < self.min_consensus:
            success = False
        print(
            f"Max #matches: {max_matches}, with ref#{max_reference} (min consensus={self.min_consensus})"
        )
        return (
            success,
            best_matched_observations,
            best_matched_observations3d,
            best_matched_observations_cam,
        )


class ValveVisualizer:
    def __init__(self):
        self.fig_3d = plt.figure()
        self.ax_3d = plt.axes(projection="3d", proj_type="ortho")

        self.fig_cam, self.ax_cam = plt.subplots()
        self.cameras = {}
        self.valves = {}
        self.points2d = {}

    def add_camera(self, label, camera: Camera):
        self.cameras[label] = camera

    def add_estimation(self, label: str, valve: ValveModel):
        self.valves[label] = valve

    def add_points(self, label, points):
        self.points2d[label] = points.reshape((-1, 2))

    def draw(self):
        for label, camera in self.cameras.items():
            if label in self.valves:
                sc = self.ax_3d.scatter3D(
                    self.valves[label].keypoints[0, :],
                    self.valves[label].keypoints[1, :],
                    self.valves[label].keypoints[2, :],
                )

                points_on_wheel = self.valves[label].get_points_on_wheel()
                self.ax_3d.plot3D(
                    points_on_wheel[0, :],
                    points_on_wheel[1, :],
                    points_on_wheel[2, :],
                    label=label,
                )

                camera.extrinsic2pyramid(
                    self.ax_3d, focal_len_scaled=0.3, color=sc.get_facecolor()
                )

                proj_circle = camera.project(points_on_wheel)
                self.ax_cam.plot(proj_circle[:, 0], proj_circle[:, 1])

                proj_keypoints = camera.project(self.valves[label].keypoints)
                self.ax_cam.scatter(
                    proj_keypoints[:, 0], proj_keypoints[:, 1], label=label
                )

                self.ax_cam.set_xlim([0, camera.resolution[0]])
                self.ax_cam.set_ylim([0, camera.resolution[1]])
                self.ax_cam.set_aspect("equal")

                markers = ["x", "o", "+", "s", "<", ">", "*"]
                if label in self.points2d.keys():
                    for i, p in enumerate(self.points2d[label]):
                        self.ax_cam.scatter(
                            p[0], p[1], marker=markers[i], color=sc.get_facecolor()
                        )

        self.ax_cam.grid(True)
        self.ax_cam.legend()
        self.ax_3d.legend()
        set_axes_equal(self.ax_3d)


if __name__ == "__main__":

    rospy.init_node("valve_fitting_node")

    # Define dummy camera intrinsics
    camera_intrinsics = np.array(
        [[695.49, 0.0, 646.83], [0.0, 694.75, 367.62], [0.0, 0.0, 1.0]]
    )
    camera_resolution = np.array([1280, 720])
    camera_distortion = np.array([])

    # Look at the valve from different points of view
    camera1 = Camera(camera_intrinsics, camera_distortion, camera_resolution)
    camera2 = deepcopy(camera1)
    camera2.transform(dz=0.02, dx=0.1, pitch_deg=-10)
    camera3 = deepcopy(camera1)
    camera3.transform(dz=0.03, dx=0.2, pitch_deg=-20)
    camera4 = deepcopy(camera1)
    camera4.transform(dz=0.02, dx=-0.1, pitch_deg=10)
    camera5 = deepcopy(camera1)
    camera5.transform(dz=0.03, dx=-0.2, pitch_deg=20)

    # camera2.transform(dz=0.4, dx=0.8, pitch_deg=-40)
    # camera3 = deepcopy(camera1)
    # camera3.transform(dx=0.4, pitch_deg=-40)
    # camera4 = deepcopy(camera1)
    # camera4.transform(dx=0.5, pitch_deg=-50)
    # camera5 = deepcopy(camera1)
    # camera5.transform(dx=-0.4, pitch_deg=40)
    cameras = [camera1, camera2, camera3, camera4, camera5]
    cameras_labels = ["cam1", "cam2", "cam3", "cam4", "cam5"]

    # Generate the ground truth valve model
    valve = ValveModel(radius=0.1, num_spokes=3, depth=-0.02)
    valve.transform(dz=0.8, roll_deg=50, pitch_deg=40)

    # Simulate inaccurrate perception

    def shuffle_keypoints(kpts: np.ndarray):
        """Shuffle the keypoints from 1 to N-1 as center is always the first"""
        shuffled_kpts = deepcopy(kpts)
        np.random.shuffle(shuffled_kpts[:, 1:].T)
        return shuffled_kpts

    # Get simulated observations
    observations = []
    for camera in cameras:
        shuffled_keypoints = shuffle_keypoints(valve.keypoints)
        proj_keypoints = camera.project(shuffled_keypoints)
        proj_keypoints = add_noise(proj_keypoints, low=0, high=10)
        observations.append(proj_keypoints)

    # Use epipolar line to find correspondences and first camera as reference
    # Do a sort of outlier rejection through RANSAC
    # Take each camera view as reference to find matches. The camera with the
    # largest amount of matches is the one to go with

    rmatcher = RansacMatcher(acceptance_ratio=0.5, mask=[0])
    obs3d = np.random.rand(4, 3)
    for cam, obs in zip(cameras, observations):
        rmatcher.add_observation(cam, obs, obs3d)
    success, mathced_2d, matched_3d = rmatcher.filter()

    # Draw observations
    visualizer = ValveVisualizer()
    for camera, camera_label, matched_obs in zip(cameras, cameras_labels, mathced_2d):
        visualizer.add_camera(camera_label, camera)
        visualizer.add_estimation(camera_label, valve)
        if matched_obs is not None:
            visualizer.add_points(camera_label, matched_obs)
    visualizer.draw()

    # # define a problem, add observations and
    # problem = ValveFitter(k=k)

    # print(f"keypoints 1 before adding noise\n{valve.keypoints}")

    # print(f"proj keypoints 1 before adding noise\n{proj_keypoints1}")
    # proj_keypoints1_n = add_noise(proj_keypoints1, low=0, high=20)
    # proj_keypoints2_n = add_noise(proj_keypoints2, low=0, high=20)

    # print(f"proj keypoints 1 after adding noise\n{proj_keypoints1_n}")
    # print(proj_keypoints1_n)
    # problem.add_observation(camera1, proj_keypoints1_n, np.array([0.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    # problem.add_observation(camera2, proj_keypoints2_n, np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    # #problem.add_observation(camera3, proj_keypoints3, np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))

    # valve_trn = problem.optimize(method='triangulation')
    # # valve_pnp = problem.optimize(method='pnp', radius_hypothesis=0.1)

    # # try triangulation
    # points_triangulated = problem.triangulate()

    plt.show()
