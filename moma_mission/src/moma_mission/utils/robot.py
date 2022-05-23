import pinocchio as pin
import rospy


class Robot:
    def __init__(self, description_name):
        robot_urdf = rospy.get_param(description_name)
        self.model = pin.buildModelFromXML(robot_urdf)
        self.data = self.model.createData()

    def get_frame_placement(self, frame, q):
        frame_id = self.model.getFrameId(frame)
        pin.framesForwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[frame_id]
