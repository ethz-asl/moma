#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <franka/gripper.h>
#include <franka/exception.h>

#include <panda_test/PandaGripperSrv.h>

class PandaGripperService {

    private:

    franka::Gripper* gripper_ptr;
    bool gripper_closed = false;

    ros::ServiceServer gripper_srv;

    public:

    PandaGripperService(franka::Gripper* g, ros::NodeHandle *nh){

        gripper_ptr = g;
        gripper_srv = nh->advertiseService("/robot_gripper_srv", &PandaGripperService::gripper_clb, this);
        ROS_INFO("Panda gripper service is ready!");
    }

    bool gripper_clb(panda_test::PandaGripperSrv::Request &req, panda_test::PandaGripperSrv::Response &res){

        try{

            //----- Perform homing if requested -----

            if (req.gripper_homing){
                bool homing_success = gripper_ptr->homing();
                if (!homing_success){
                    ROS_INFO("Failed to perform homing");
                    return false;
                }
            }

            //----- Move the gripper to a specific width with specified velocity and force -----

            if (req.gripper_close){
                franka::GripperState gripper_state = gripper_ptr->readOnce();
                if (gripper_state.max_width < req.grasping_width) {
                    ROS_INFO("Object is too big for the current finger configuration");
                    return false;
                }
                if (!gripper_ptr->grasp(req.grasping_width, req.grasping_speed, req.grasping_force)) {
                    ROS_INFO("Failed to grasp object");
                    return false;
                }

//                if (!gripper_ptr->move(req.grasping_width, req.grasping_speed)) {
//                    ROS_INFO("Failed to gmove properly");
//                    return false;
//                }

                gripper_closed = true;
            }

            //----- Release the object -----

            if (!req.gripper_close && gripper_closed){
                gripper_ptr->stop();
                gripper_closed = false;
            }

            res.success = true;

        } catch (const franka::Exception& ex){

            ROS_ERROR_STREAM("Failed to perform gripper request: "<<ex.what());
            return false;
        }
        return true;
    }
};

int main (int argc, char **argv){

    ros::init(argc, argv, "robot_gripper_service");
    ros::NodeHandle nh;

    franka::Gripper robot_gripper(argv[1]);

    PandaGripperService p = PandaGripperService(&robot_gripper, &nh);
    ros::spin();
}
