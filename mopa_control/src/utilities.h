#ifndef UTILITIES_H__
#define UTILITIES_H__

#include <Eigen/Dense>

namespace util {
    std::vector<double> eigenvec_to_stdvec(const Eigen::Matrix<double,Eigen::Dynamic,1> &eig_vec) {
        std::vector<double> stdvec(eig_vec.data(), eig_vec.data() + eig_vec.size());
        return stdvec;
    }
}



#endif
