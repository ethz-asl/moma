#include "moma_ocs2_ros/admittance_controller.h"

using namespace moma_controllers;


int main(int argc, char** argv){
	ros::init(argc, argv, "admittance_controller");
	ros::NodeHandle nh;
    AdmittanceController controller(nh);
    ros::spin();
	return 0;
}
