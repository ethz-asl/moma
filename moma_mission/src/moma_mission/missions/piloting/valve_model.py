import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R
from moma_mission.missions.piloting.frames import Frames


class ValveModel:
    """
    A mechanical valve, consisting of a wheel and several spokes
    connecting the wheel with the valve center.

    Nomenclature:

    - valve: The entire structure
    - wheel: The main torus
    - spoke: The inner connection between torus and center
    """

    def __init__(self, frame=Frames.map_frame, center=np.array([0.0, 0.0, 0.0]), radius=0.12, spoke_radius=0.01, axis_1=np.array([1.0, 0.0, 0.0]), axis_2=np.array([0.0, 1.0, 0.0]), num_spokes=3, depth=0.0):
        """
        @param frame: frame in which valve is defined
        @param center: valve center point
        @param radius: valve radius
        @param spoke_radius: spoke radius
        @param num_spokes: number of spokes
        @param axis_1: first axis
        @param axis_2: second axis
        @param depth: distance center from wheel plane
        """

        # Keeping internal representation separate from interface
        # to be able to exchange internals without breaking interface
        self.__frame = frame
        self.__c = center
        self.__r = radius
        self.__s = spoke_radius
        self.__k = num_spokes
        self.__v1 = axis_1 / np.linalg.norm(axis_1)
        self.__v2 = axis_2 / np.linalg.norm(axis_2)
        self.__d = depth

        self.marker_pub = rospy.Publisher("/valve_marker", MarkerArray, queue_size=1, latch=True)
        try:
            self.publish_markers()
        except:
            pass

    def transform(self, dx=0.0, dy=0.0, dz=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        r = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
        t = np.array([dx, dy, dz])
        
        self.__c = self.__c + t
        self.__v1 = r @ self.__v1
        self.__v2 = r @ self.__v2

        self.publish_markers()

    def turn(self, angle_deg):
        """
        Turn the valve by a given angle
        """
        r = R.from_rotvec(self.normal * angle_deg, degrees=True).as_matrix()

        self.__v1 = r @ self.__v1
        self.__v2 = r @ self.__v2

        self.publish_markers()

    @property
    def frame(self):
        """
        Valve frame
        """
        return self.__frame

    @property
    def center(self):
        """
        Valve center, point where all spokes coincide
        """
        return self.__c

    @property
    def wheel_center(self):
        """
        Wheel center, virtual center point in the wheel plane
        """
        return self.__c + self.normal * self.__d

    @property
    def axis_1(self):
        return self.__v1

    @property
    def axis_2(self):
        return self.__v2

    @property
    def radius(self):
        """
        Wheel radius (major wheel torus radius)
        """
        return self.__r

    @property
    def handle_radius(self):
        """
        Handle radius (minor wheel torus radius)
        """
        return self.spoke_radius

    @property
    def spoke_radius(self):
        """
        Spoke radius, assuming cylindrical spokes
        """
        return self.__s

    @property
    def spoke_length(self):
        """
        Spoke length from the valve center to the center of the wheel torus intersection
        """
        return np.sqrt(np.power(self.__r, 2) + np.power(self.__d, 2))

    @property
    def normal(self):
        """
        Vector normal to the wheel plane
        """
        return np.cross(self.__v1, self.__v2)

    @property
    def spokes_angles(self):
        """
        Angles of all spokes
        """
        return [2 * np.pi * k / self.__k for k in range(self.__k)]

    @property
    def spokes_positions(self):
        """
        Positions of all spokes
        """
        return [self.get_point_on_wheel(angle) for angle in self.spokes_angles]

    @property
    def keypoints(self):
        """
        All keypoints (including center one)
        """
        keypoints = np.zeros((3, self.__k + 1)) # k keypoints and the center
        keypoints[:, 0] = self.__c
        keypoints[:, 1:] = self.spokes_positions
        return keypoints

    def get_point_on_wheel(self, angle):
        """
        Get a centered point on the wheel at a given angle
        """
        return self.__c + self.__r * (np.cos(angle) * self.__v1 + np.sin(angle) * self.__v2) + self.__d * self.normal

    def get_tangent_on_wheel(self, angle):
        """
        Get the tangent on the wheel at a given angle
        """
        return -np.sin(angle) * self.__v1 + np.cos(angle) * self.__v2

    def get_markers(self):
        markers = MarkerArray()
        wheel_marker = Marker()
        wheel_marker.header.frame_id = self.__frame
        wheel_marker.id = 0
        wheel_marker.action = Marker.ADD
        wheel_marker.type = Marker.CYLINDER
        wheel_marker.scale.x = 2 * self.__r
        wheel_marker.scale.y = 2 * self.__r
        wheel_marker.scale.z = 2 * self.__s
        
        rot = np.array([self.__v1, self.__v2, self.normal]).T
        q = R.from_matrix(rot).as_quat()
        wheel_marker.pose.orientation.x = q[0]
        wheel_marker.pose.orientation.y = q[1]
        wheel_marker.pose.orientation.z = q[2]
        wheel_marker.pose.orientation.w = q[3]
        wheel_marker.pose.position.x = self.wheel_center[0]
        wheel_marker.pose.position.y = self.wheel_center[1]
        wheel_marker.pose.position.z = self.wheel_center[2]
        wheel_marker.color.r = 1.0
        wheel_marker.color.g = 0.0
        wheel_marker.color.b = 1.0
        wheel_marker.color.a = 0.2
        
        markers.markers.append(wheel_marker)
        
        for i in range(self.__k):
            angle = 2 * i * np.pi / self.__k
            spoke_position = self.get_point_on_wheel(angle)
            spoke_marker = Marker()
            spoke_marker.header.frame_id = self.__frame
            spoke_marker.id = i + 1
            spoke_marker.action = Marker.ADD
            spoke_marker.type = Marker.CYLINDER
            spoke_marker.scale.x = 2 * self.__s
            spoke_marker.scale.y = 2 * self.__s
            spoke_marker.scale.z = self.spoke_length
            spoke_marker.color.r = 1.0
            spoke_marker.color.g = 0.0
            spoke_marker.color.b = 1.0
            spoke_marker.color.a = 0.4
            
            # z axis of the spoke cylinder points as P - c
            z = spoke_position - self.__c
            z = z / np.linalg.norm(z)
            x = np.array([-z[1], z[0], 0.0])
            y = np.cross(z, x)
            spoke_rotation = np.array([x, y, z]).T
            spoke_marker_position = (self.__c + spoke_position) / 2.0
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

    def publish_markers(self):
        self.marker_pub.publish(self.get_markers())

    def __str__(self):
        return f"""
center = {self.__c}
axis 1 = {self.__v1}
axis 2 = {self.__v2}
depth = {self.__d}
radius = {self.__r}
"""


if __name__ == "__main__":
    rospy.init_node("valve_model_test")
    valve = ValveModel()
    valve.transform(roll_deg=00.0, yaw_deg=40)
    markers = valve.get_markers(frame_id="world")
    markers_pub = rospy.Publisher("valve_marker", MarkerArray, queue_size=1)
    while not rospy.is_shutdown():
        markers_pub.publish(markers)

    