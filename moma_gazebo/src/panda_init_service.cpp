#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <moma_gazebo/PandaInitSrv.h>

//----- Description -----

// This service mimics the non-real time commands that
// will have to be completed before the automatic control
// starts. These mainly correspond to setting the appropriate
// end effector and stiffness frames.

//-----------------------

class PandaInitService {

    private:

    std::array<double, 16> setEE;
    std::array<double, 16> setK;

    ros::ServiceServer panda_init_srv;

    public:

    PandaInitService(ros::NodeHandle *nh){

        //----- Advertise the Init service -----

        panda_init_srv = nh->advertiseService("/panda_init_srv", &PandaInitService::init_clb, this);
        ROS_INFO("Panda init service is ready!");
    }

    bool init_clb(moma_gazebo::PandaInitSrv::Request &req, moma_gazebo::PandaInitSrv::Response &res){

        try{

            std::array<double, 16> NE_T_EE;
            std::array<double, 16> EE_T_K;

            for (size_t i=0; i<16; ++i){

                NE_T_EE[i] = req.NE_T_EE[i];
                EE_T_K[i] = req.EE_T_K[i];
            }

            res.success = true;


        } catch (const std::exception& e){

            res.success = false;
            ROS_ERROR_STREAM("Failed to init!");
            return false;
        }
        return true;
    }
};

int main (int argc, char **argv){

    ros::init(argc, argv, "panda_init_service");
    ros::NodeHandle nh;
    PandaInitService p = PandaInitService(&nh);
    ros::spin();
    return 0;
}
