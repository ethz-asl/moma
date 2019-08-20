#ifndef UTILITIES_H__
#define UTILITIES_H__

#include <Eigen/Dense>

namespace util {
    std::vector<double> eigenvec_to_stdvec(const Eigen::Matrix<double,Eigen::Dynamic,1> &eig_vec) {
        std::vector<double> std_vec(eig_vec.data(), eig_vec.data() + eig_vec.size());
        return std_vec;
    }

    Eigen::Matrix<double,7,1> stdvec_to_eigenvec(const std::vector<double> &std_vec) {
        ROS_ASSERT(std_vec.size() == 7);
        Eigen::Matrix<double,7,1> eig_vec(std_vec.data());
        return eig_vec;
    }

    std::string stdVecToStr(const std::vector<double> &vec) {
        std::stringstream ss;
        ss.str(std::string());
        for (const double& val: vec) {
            ss << val << " ";
        }
        return ss.str();
    }
}

#endif
