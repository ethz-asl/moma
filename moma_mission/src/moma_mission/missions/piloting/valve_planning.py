import rospy
import tf2_ros
import numpy as np
import pinocchio as pin
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Pose
from nav_msgs.msg import Path

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.utils.transforms import se3_to_pose_ros, tf_to_se3
from moma_mission.utils.rotation import CompatibleRotation as R
from moma_mission.missions.piloting.valve_fitting import ValveModel

class ValvePlanner(object):
    """
    Utilities class to generate graps and paths given a valve model
    """

    def __init__(self):
        self.frames = Frames
        self.valve_model = ValveModel()

  
    def set_valve(self, valve_model: ValveModel):
        self.valve_model = valve_model

    def reset(self):
        self.theta_desired = 0.0
        self.theta = 0.0

    def advance(self, dt):
        """
        Increment theta according to current velocity and step size and computes the new
        target pose for the end effector
        :param dt:
        :return:
        """
        self.theta_desired += self.theta_dot * dt
        target = self.compute_target(self.theta_desired)
        self.target_pose_pub.publish(target)

    def _get_all_grasping_poses(self):
        radius = self.valve_model.r
        center = self.valve_model.c 
        axis_1 = self.valve_model.v1
        axis_2 = self.valve_model.v2

        samples = 100
        thetas = [i * 2 * np.pi / samples for i in range(samples)]
        points = [center + np.cos(theta) * radius * axis_1 + np.sin(theta) * radius * axis_2 for theta in thetas]

        zdes = np.array([0.0, 0.0, -1.0])
        xs = [-np.sin(theta) * axis_1 + np.cos(theta) * axis_2 for theta in thetas]
        zs = [zdes - np.dot(zdes, x) * x for x in xs]
        zs = [z / np.linalg.norm(z) for z in zs]
        ys = [np.cross(z, x) for x, z in zip(xs, zs)]
        rot = [np.array([x, y, z]).transpose() for x, y, z, in zip(xs, ys, zs)]
        quat = [R.from_matrix(r).as_quat() for r in rot]
        poses = [{"position": p, "orientation": q} for p, q in zip(points, quat)]
        return poses

    def _filter_radial_grasps(self, grasps):
        """ 
        Filter the grasps such that they are pointing to the center
        """
        filtered_grasps = []
        for grasp in grasps:
            radial = self.valve_model.c - grasp["position"]
            z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
            if np.dot(z_axis, radial) > 0:
                filtered_grasps.append(grasp)
        return filtered_grasps

    def _filter_singular_grasps(self, grasps):
        """
        Filter grasps where the tangential direction is basically parallel to the z axis and would 
        force a very unnatural and difficult grasp

        """
        filtered_grasps = []
        threshold = 1e-3
        for grasp in grasps:
            z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
            if np.dot(z_axis, np.array([0, 0, -1])) > threshold:
                filtered_grasps.append(grasp)
        return filtered_grasps

    def _filter_graspable_ranges(self, grasps):
        radius = self.valve_model.r
        center = self.valve_model.c 
        axis_1 = self.valve_model.v1
        axis_2 = self.valve_model.v2
        num_spokes = self.valve_model.k

        # find where the spokes are
        spokes_width = 0.04 # TODO set this from param or valve model
        spokes_angles = [2*k*np.pi /num_spokes for k in range(num_spokes)]
        spokes_position = [center + radius * (axis_1 * np.cos(a) + axis_2 * np.sin(a)) for a in spokes_angles]

        grasps_ranges = []
        grasps_filtered = []
        creating_new_range = False
        for idx, g in enumerate(grasps):
            
            if np.all((np.linalg.norm(g["position"] - spokes_position, axis=1) - spokes_width) > 0):
                grasps_filtered.append(g)

                if not creating_new_range:
                    range_start = idx
                    creating_new_range = True

            elif creating_new_range:
                range_end = idx-1
                grasps_ranges.append([range_start, range_end])
                creating_new_range = False
    
        return grasps_ranges, grasps_filtered
    
    def _grasps_to_angle(self, grasps):
        """
        Given a grasp position tells to which angle this corresponds
        """
        angles = []
        for grasp in grasps:
            vector = grasp["position"] - self.valve_model.c
            cos_angle = np.dot(vector, self.valve_model.v1) / np.linalg.norm(vector)
            sin_angle = np.dot(vector, self.valve_model.v2) / np.linalg.norm(vector)
            angles.append(np.arctan2(sin_angle, cos_angle))
        return angles

    def get_path(self, max_angle=np.pi/2.0):
        """
        Given a maximum angle that we can achieve withing a single manipulation step
        extrct a path with the following properties
        1. the first grasp does not intersect with a spoke
        2. all the poses along the path points toward the center (if possible)
        3. all poses along the path are continuous
        4. motion from the last to 
        """

        grasps = self._get_all_grasping_poses()            # get all grasps geometrycally feasible
        grasps = self._filter_radial_grasps(grasps)        # get only grasps pointing to the center
        grasps = self._filter_singular_grasps(grasps)      # grasps where we the z axis cannot point down at all  
        grasps_angles = self._grasps_to_angle(grasps)      # compute inverse -> corresponding angle in the current valve model
        grasps_ranges, grasps_feasible = self._filter_graspable_ranges(grasps) # filter what is not currently graspable as there is a spoke there

        # get the longest range
        current_range = 0
        max_range = None
        for grasp_range in grasps_ranges:
            if abs(grasp_range[1] - grasp_range[0]) > current_range:
                current_range = abs(grasp_range[1]- grasp_range[0])
                max_range = grasp_range

        path = []
        for i in range(max_range[0], max_range[1]):
            
            path.append(grasps[i])
            # TODO this does not properly work as the angle computed in the previous function is also negative...
            # if abs(grasps_angles[i] - grasps_angles[max_range[0]]) > max_angle:
            #     break
        return path

    def poses_to_ros(self, poses):
        posesa = PoseArray()
        posesa.header.frame_id = self.frames.map_frame
        posesa.header.stamp = rospy.get_rostime()
        for p in poses:
            pose = Pose()
            pose.position.x = p["position"][0]
            pose.position.y = p["position"][1]
            pose.position.z = p["position"][2]
            pose.orientation.x = p["orientation"][0]
            pose.orientation.y = p["orientation"][1]
            pose.orientation.z = p["orientation"][2]
            pose.orientation.w = p["orientation"][3]
            posesa.poses.append(pose)
        return posesa


            
if __name__ == "__main__":
    rospy.init_node("valve_planner_test")
    
    poses_pub = rospy.Publisher("/plan", PoseArray, queue_size=1)
    marker_pub = rospy.Publisher("/valve_marker", MarkerArray, queue_size=1)
    
    valve_model = ValveModel()
    valve_planner = ValvePlanner()
    valve_planner.set_valve(valve_model)

    while not rospy.is_shutdown():
        valve_model.transform(pitch_deg=1)
        valve_planner.set_valve(valve_model)

        poses = valve_planner._get_all_grasping_poses()
        poses = valve_planner._filter_radial_grasps(poses)
        poses = valve_planner._filter_singular_grasps(poses)
        _, poses = valve_planner._filter_graspable_ranges(poses)
        poses = valve_planner.get_path()

        poses_ros = valve_planner.poses_to_ros(poses)
        poses_pub.publish(poses_ros)
    
        valve_marker = valve_model.get_markers(frame_id=Frames.map_frame)
        marker_pub.publish(valve_marker)
        rospy.sleep(0.02)
    