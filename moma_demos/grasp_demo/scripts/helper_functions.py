# Helper functions for Reactive Decision Making
# Used for Pose and Point transformations functions

from geometry_msgs.msg import Pose, PoseStamped

def transform2position(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        # print("F1")
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        # print("F2")
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

def create_pose(trans,rot):
    pose=geometry_msgs.msg.Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def position2imgPoints(position):
    position_transformed = np.array([[position.x,position.y,position.z]])
    position_transformed = position_transformed.T
    cameraMatrix = np.array([602.1849879340944, 0.0, 320.5, 0.0, 602.1849879340944, 240.5, 0.0, 0.0, 1.0]).reshape(3,3)
    distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    rvec = np.array([0,0,0], np.float) # rotation vector
    tvec = np.array([0,0,0], np.float) # translation vector
    imagePoints, jac = cv.projectPoints(position_transformed, rvec=rvec, tvec=tvec, cameraMatrix=cameraMatrix,distCoeffs=distCoeffs)
    return [imagePoints[0][0][0],imagePoints[0][0][1]]
