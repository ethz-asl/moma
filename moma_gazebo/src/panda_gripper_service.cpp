#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <moma_gazebo/PandaGripperSrv.h>

class PandaGripperService {

    private:

    bool gripper_closed = false;
    bool homing_finished = false;


    ros::ServiceServer gripper_srv;

    public:

    PandaGripperService(ros::NodeHandle *nh){

        gripper_srv = nh->advertiseService("/robot_gripper_srv", &PandaGripperService::gripper_clb, this);
        ROS_INFO("Panda gripper service is ready!");
    }

    bool gripper_clb(moma_gazebo::PandaGripperSrv::Request &req, moma_gazebo::PandaGripperSrv::Response &res){

        try{

            //----- Perform homing if requested -----

            if (req.gripper_homing){
                homing_finished = true;
                ROS_INFO("Homing completed!");
            }

            //----- Move the gripper to a specific width with specified velocity and force -----

            if (req.gripper_close){
                gripper_closed = true;
                ROS_INFO("Gripper closed!");
            }

            //----- Release the object -----

            if (!req.gripper_close && gripper_closed){
                ROS_INFO("Object released!");
                gripper_closed = false;
            }

            res.success = true;

        } catch (const std::exception& e){

            ROS_ERROR_STREAM("Failed to perform gripper request");
            return false;
        }
        return true;
    }
};

int main (int argc, char **argv){

    ros::init(argc, argv, "robot_gripper_service");
    ros::NodeHandle nh;

    PandaGripperService p = PandaGripperService(&nh);
    ros::spin();
}
