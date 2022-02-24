import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray, Pose

from moma_mission.missions.piloting.frames import Frames
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

    def _get_all_grasping_poses(self, samples=100):
        """
        Get all potential grasping poses
        """
        thetas = [i * 2 * np.pi / samples for i in range(samples)]
        points = [self.valve_model.get_point_on_wheel(theta) for theta in thetas]

        zdes = np.array([0.0, 0.0, -1.0])
        xs = [self.valve_model.get_tangent_on_wheel(theta) for theta in thetas]
        zs = [zdes - np.dot(zdes, x) * x for x in xs]
        zs = [z / np.linalg.norm(z) for z in zs]
        ys = [np.cross(z, x) for x, z in zip(xs, zs)]
        rot = [np.array([x, y, z]).transpose() for x, y, z, in zip(xs, ys, zs)]
        quat = [R.from_matrix(r).as_quat() for r in rot]
        # Also store the index and the total count to be able
        # to restore continuous sequences after filtering grasps
        poses = [{"index": i, "samples": samples, "angle": t, "position": p, "orientation": q} for i, (p, q, t) in enumerate(zip(points, quat, thetas))]
        return poses

    def _is_radial_grasp(self, grasp):
        """ 
        Check if grasp is pointing to the center
        """
        radial = self.valve_model.c - grasp["position"]
        z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
        return np.dot(z_axis, radial) > 0

    def _is_non_singular_grasp(self, grasp, threshold=0.1):
        """
        Check if the tangential direction of the grasp is basically parallel to the z axis and would
        force a very unnatural and difficult grasp

        :param threshold: Projection to z-axis threshold
        """
        z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
        return np.dot(z_axis, np.array([0, 0, -1])) > threshold

    def _is_non_obstructed_grasp(self, grasp, safety_distance=-1):
        """
        Check if grasp is non obstructed, for example obstruction by spokes

        :param safety_distance: Minimum distance to spokes
        """
        if safety_distance < 0:
            safety_distance = self.valve_model.s

        spokes_radius = self.valve_model.s
        spokes_positions = self.valve_model.get_spokes_positions()
        return np.all((np.linalg.norm(grasp["position"] - spokes_positions, axis=1) - spokes_radius) > safety_distance)

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
    
    valve_model = ValveModel(c=[0.5, 0.5, 0.5])
    valve_planner = ValvePlanner()
    valve_planner.set_valve(valve_model)

    while not rospy.is_shutdown():
        valve_model.transform(pitch_deg=1)
        valve_planner.set_valve(valve_model)

        poses = valve_planner._get_all_grasping_poses()
        poses = filter(valve_planner._is_radial_grasp, poses)
        poses = filter(valve_planner._is_non_singular_grasp, poses)
        poses = filter(valve_planner._is_non_obstructed_grasp, poses)
        #poses = valve_planner.get_path()

        poses_ros = valve_planner.poses_to_ros(poses)
        poses_pub.publish(poses_ros)
    
        valve_marker = valve_model.get_markers(frame_id=Frames.map_frame)
        marker_pub.publish(valve_marker)
        rospy.sleep(0.02)
    