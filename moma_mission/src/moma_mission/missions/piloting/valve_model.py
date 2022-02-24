import numpy as np
from moma_mission.missions.piloting.valve import Valve
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R


class ValveModel:
    """
    Nomenclature:

    valve: The entire structure
    wheel: The main torus
    spoke: The inner connection between torus and center
    """

    def __init__(self, c=np.array([0.0, 0.0, 0.0]), r=0.12, s=0.01, v1=np.array([1.0, 0.0, 0.0]), v2=np.array([0.0, 1.0, 0.0]), k=3, delta=0.0):
        """
        c: center
        r: valve radius
        s: spoke radius
        k: number of spokes
        v1: first axis
        v2: second axis
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
        self.s = s

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
        
        self.c = self.c + t
        self.v1 = r @ self.v1
        self.v2 = r @ self.v2
        self.n = np.cross(self.v1, self.v2)
        self.keypoints = ((r @ self.keypoints).T + t).T
        self.circle = ((r @ self.circle).T + t).T

    def get_point_on_wheel(self, angle):
        """
        Get a centered point on the wheel at a given angle
        """
        return self.c + self.r * (np.cos(angle) * self.v1 + np.sin(angle) * self.v2)

    def get_tangent_on_wheel(self, angle):
        """
        Get the tangent on the wheel at a given angle
        """
        return -np.sin(angle) * self.v1 + np.cos(angle) * self.v2

    def get_spokes_angles(self):
        """
        Get the angles of all spokes
        """
        return [2 * np.pi * k / self.k for k in range(self.k)]

    def get_spokes_positions(self):
        return [self.get_point_on_wheel(angle) for angle in self.get_spokes_angles()]

    def get_markers(self, frame_id):
        markers = MarkerArray()
        wheel_marker = Marker()
        wheel_marker.header.frame_id = frame_id
        wheel_marker.id = 0
        wheel_marker.action = Marker.ADD
        wheel_marker.type = Marker.CYLINDER
        wheel_marker.scale.x = 2 * self.r
        wheel_marker.scale.y = 2 * self.r
        wheel_marker.scale.z = 2 * self.s
        
        rot = np.array([self.v1, self.v2,  self.n]).T
        q = R.from_matrix(rot).as_quat()
        wheel_marker.pose.orientation.x = q[0]
        wheel_marker.pose.orientation.y = q[1]
        wheel_marker.pose.orientation.z = q[2]
        wheel_marker.pose.orientation.w = q[3]
        wheel_marker.pose.position.x = self.c[0]
        wheel_marker.pose.position.y = self.c[1]
        wheel_marker.pose.position.z = self.c[2]
        wheel_marker.color.r = 1.0
        wheel_marker.color.g = 0.0
        wheel_marker.color.b = 0.0
        wheel_marker.color.a = 0.1
        
        markers.markers.append(wheel_marker)
        
        for i in range(self.k):
            angle = 2 * i * np.pi / self.k
            spoke_position = self.get_point_on_wheel(angle)
            spoke_marker = Marker()
            spoke_marker.header.frame_id = frame_id
            spoke_marker.id = i + 1
            spoke_marker.action = Marker.ADD
            spoke_marker.type = Marker.CYLINDER
            spoke_marker.scale.x = 2 * self.s
            spoke_marker.scale.y = 2 * self.s
            spoke_marker.scale.z = self.r
            spoke_marker.color.r = 1.0
            spoke_marker.color.g = 0.0
            spoke_marker.color.b = 0.0
            spoke_marker.color.a = 0.4
            
            # z axis of the spoke cylinder points as P - c
            z = spoke_position - self.c 
            z = z / np.linalg.norm(z)
            x = np.array([-z[1], z[0], 0.0])
            y = np.cross(z, x)
            spoke_rotation = np.array([x, y, z]).T
            spoke_marker_position = (self.c + spoke_position)/2.0 
            q = R.from_matrix(spoke_rotation).as_quat()
            spoke_marker.pose.orientation.x = q[0]
            spoke_marker.pose.orientation.y = q[1]
            spoke_marker.pose.orientation.z = q[2]
            spoke_marker.pose.orientation.w = q[3]
            spoke_marker.pose.position.x = spoke_marker_position[0]
            spoke_marker.pose.position.y = spoke_marker_position[1]
            spoke_marker.pose.position.z = spoke_marker_position[2]
            markers.markers.append(spoke_marker)
        return markers
        
    def __str__(self):
        return f"""
center = {self.c}
axis 1 = {self.v1}
axis 2 = {self.v2}
delta = {self.delta}
radius = {self.r}
"""


if __name__ == "__main__":
    rospy.init_node("valve_model_test")
    valve = ValveModel()
    valve.transform(roll_deg=00.0, yaw_deg=40)
    markers = valve.get_markers(frame_id="world")
    markers_pub = rospy.Publisher("valve_marker", MarkerArray, queue_size=1)
    while not rospy.is_shutdown():
        markers_pub.publish(markers)

    