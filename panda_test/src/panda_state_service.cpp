#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>

#include <panda_test/PandaStateSrv.h>

class PandaStateService {

    private:

    franka::Robot* robot_ptr;
    franka::Model* robot_model_ptr;

    ros::ServiceServer get_robot_state_srv;

    public:

    PandaStateService(franka::Robot* r, franka::Model* m, ros::NodeHandle *nh){

        robot_ptr = r;
        robot_model_ptr = m;
        get_robot_state_srv = nh->advertiseService("/panda_state_srv", &PandaStateService::state_clb, this);
        ROS_INFO("Panda state service is ready!");
    }

    bool state_clb(panda_test::PandaStateSrv::Request &req, panda_test::PandaStateSrv::Response &res){

        try{

            if (req.set_frames){

                std::array<double, 16> NE_T_EE;
                std::array<double, 16> EE_T_K;

                for (size_t i=0; i<16; ++i){

                    NE_T_EE[i] = req.NE_T_EE[i];
                    EE_T_K[i] = req.EE_T_K[i];
                }

                robot_ptr->setEE(NE_T_EE);
                robot_ptr->setK(EE_T_K);

            }

            franka::RobotState current_state = robot_ptr->readOnce();

            std::array<double, 7> coriolis = robot_model_ptr->coriolis(current_state);
            std::array<double, 7> gravity = robot_model_ptr->gravity(current_state);
            std::array<double, 42> jacobian = robot_model_ptr->zeroJacobian(franka::Frame::kEndEffector, current_state);
            std::array<double, 49> mass_matrix = robot_model_ptr->mass(current_state);

            for (size_t i=0; i<7; ++i){

                res.q_d[i] = current_state.q_d[i];
                res.dq_d[i] = current_state.dq_d[i];
                res.ddq_d[i] = current_state.ddq_d[i];
                res.q[i] = current_state.q[i];
                res.dq[i] = current_state.dq[i];

                res.tau_d_no_gravity[i] = current_state.tau_J_d[i];
                res.tau[i] = current_state.tau_J[i];
                res.tau_ext[i] = current_state.tau_ext_hat_filtered[i];

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

                res.EE_T_K[i] = current_state.EE_T_K[i];
                res.O_T_EE[i] = current_state.O_T_EE[i];
            }

            for (size_t i=0; i<6; ++i){

                res.K_F_ext_hat_K[i] = current_state.K_F_ext_hat_K[i];
            }

        } catch (const franka::Exception& ex){

            ROS_ERROR_STREAM("Failed to get the robot state with error: "<<ex.what());
            return false;
        }
        return true;
    }
};

int main (int argc, char **argv){

    ros::init(argc, argv, "robot_state_service");
    ros::NodeHandle nh;

    franka::Robot robot(argv[1]);
    franka::Model model = robot.loadModel();

    PandaStateService p = PandaStateService(&robot, &model, &nh);
    ros::spin();
}
