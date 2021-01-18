#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <moma_gazebo/PandaStateSrv.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

class PandaStateService {

    private:

    ros::ServiceServer get_robot_state_srv;

    ros::Subscriber joint_state_subscriber;
    ros::Subscriber eeforce_subscriber;

    std::array<double, 7> q_d;
    std::array<double, 7> dq_d;
    std::array<double, 7> ddq_d;
    std::array<double, 7> q;
    std::array<double, 7> dq;
    std::array<double, 7> tau_J_d;
    std::array<double, 7> tau_J;
    std::array<double, 7> tau_ext_hat_filtered;

    std::array<double, 42> jacobian;
    std::array<double, 49> mass_matrix;
    std::array<double, 7> coriolis;
    std::array<double, 7> gravity;

    std::array<double, 16> EE_T_K;
    std::array<double, 16> O_T_EE;

    std::array<double, 6> K_F_ext_hat_K;

    bool processing = false;

    public:

    PandaStateService(ros::NodeHandle *nh){

        joint_state_subscriber = nh->subscribe("/joint_states", 10, &PandaStateService::joint_state_clb, this);
        eeforce_subscriber = nh->subscribe("/eeforce", 10, &PandaStateService::eeforce_clb, this);

        get_robot_state_srv = nh->advertiseService("/get_panda_state_srv", &PandaStateService::state_clb, this);
        ROS_INFO("Panda state service is ready!");

        for (size_t i=0; i<42; ++i){
            jacobian[i] = i;
        }

        for (size_t i=0; i<49; ++i){
            mass_matrix[i] = i;
        }

        for (size_t i=0; i<7; ++i){
            q_d[i] = i;
            dq_d[i] = i;
            ddq_d[i] = i;
            q[i] = i;
            dq[i] = i;
            tau_J[i] = i;
            tau_J_d[i] = i;
            tau_ext_hat_filtered[i] = i;
            coriolis[i] = i;
            gravity[i] = i;
        }
        EE_T_K = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 1};
        O_T_EE = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.3, 0.2, 0.5, 1};
        K_F_ext_hat_K = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    }

    bool state_clb(moma_gazebo::PandaStateSrv::Request &req, moma_gazebo::PandaStateSrv::Response &res){

        processing = true;
        try{

            for (size_t i=0; i<7; ++i){

                res.q_d[i] = q_d[i];
                res.dq_d[i] = dq_d[i];
                res.ddq_d[i] = ddq_d[i];
                res.q[i] = q[i];
                res.dq[i] = dq[i];

                res.tau_d_no_gravity[i] = tau_J_d[i];
                res.tau[i] = tau_J[i];
                res.tau_ext[i] = tau_ext_hat_filtered[i];

                res.coriolis[i] = coriolis[i];
                res.gravity[i] = gravity[i];
            }

            for (size_t i=0; i<42; ++i){

                res.jacobian[i] = jacobian[i];
            }

            for (size_t i=0; i<49; ++i){

                res.mass_matrix[i] = mass_matrix[i];
            }

            for (size_t i=0; i<16; ++i){

                res.EE_T_K[i] = EE_T_K[i];
                res.O_T_EE[i] = O_T_EE[i];
            }

            for (size_t i=0; i<6; ++i){

                res.K_F_ext_hat_K[i] = K_F_ext_hat_K[i];
                }

        } catch (const std::exception& e){
            processing = false;
            ROS_ERROR_STREAM("Failed to get the robot state with error");
            return false;
        }
        processing = false;
        return true;
    }

    void joint_state_clb(const sensor_msgs::JointState& msg) {

        if (~processing){

            if (msg.position.size() == 12){
                for (size_t i=3; i<10; ++i){
                    q_d[i - 3] = msg.position[i];
                    dq_d[i -3] = msg.velocity[i];
                    tau_J[i - 3] = msg.effort[i];
                }
            }
        }
    }
//
    void eeforce_clb(const geometry_msgs::WrenchStamped& msg){

        if (~processing){
            K_F_ext_hat_K[0] = msg.wrench.force.x;
            K_F_ext_hat_K[1] = msg.wrench.force.y;
            K_F_ext_hat_K[2] = msg.wrench.force.z;

            K_F_ext_hat_K[3] = msg.wrench.torque.x;
            K_F_ext_hat_K[4] = msg.wrench.torque.y;
            K_F_ext_hat_K[5] = msg.wrench.torque.z;
        }
    }
};

int main (int argc, char **argv){

    ros::init(argc, argv, "robot_state_service");
    ros::NodeHandle nh;
    PandaStateService p = PandaStateService(&nh);
    ros::spin();
    return 0;
}
