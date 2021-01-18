#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <franka/robot.h>
#include <franka/exception.h>

#include <panda_control_door_opening/PandaInitSrv.h>

class PandaInitService {

    private:

    franka::Robot robot;

    ros::ServiceServer panda_init_srv;

    public:

    PandaInitService(const std::string& franka_address, ros::NodeHandle *nh) : robot(franka_address){

        panda_init_srv = nh->advertiseService("/panda_init_srv", &PandaInitService::init_clb, this);
        ROS_INFO("Panda init service is ready!");
    }

    bool init_clb(panda_control_door_opening::PandaInitSrv::Request &req, panda_control_door_opening::PandaInitSrv::Response &res){

        try{

            std::array<double, 16> NE_T_EE;
            std::array<double, 16> EE_T_K;

            for (size_t i=0; i<16; ++i){

                NE_T_EE[i] = req.NE_T_EE[i];
                EE_T_K[i] = req.EE_T_K[i];
            }

            robot.setEE(NE_T_EE);
            robot.setK(EE_T_K);
            res.success = true;


        } catch (const franka::Exception& ex){

            res.success = false;
            ROS_ERROR_STREAM("Failed to init: "<<ex.what());
            return false;
        }
        return true;
    }
};

int main (int argc, char **argv){

    ros::init(argc, argv, "panda_init_service");
    ros::NodeHandle nh;
    PandaInitService p = PandaInitService(argv[1], &nh);
    ros::spin();
}
