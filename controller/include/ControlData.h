#ifndef ARM_CTRLDATA_H
#define ARM_CTRLDATA_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "cmath"
#include <vector>

struct ArmParameter
{
    Eigen::Matrix<double, 5, 1> torquelimit;
    std::vector<double> jointlimit;
    std::vector<double> Kp;
    std::vector<double> Kd;
    std::vector<double> des_rotation;
    std::vector<double> des_translation;
    double gripper_des_q;
};

struct ArmData
{
    Eigen::Matrix<double, 5, 1> arm_tau;

    Eigen::Matrix<double, 5, 1> arm_q;
    Eigen::Matrix<double, 5, 1> arm_dq;

    Eigen::MatrixXd J;
    Eigen::MatrixXd dJ;

    Eigen::MatrixXd M;
	Eigen::MatrixXd C;
	Eigen::MatrixXd G;
};

struct ArmCmd
{
    Eigen::Matrix<double, 5, 1> full_tau;
    Eigen::Matrix<double, 5, 1> full_q;
    Eigen::Matrix<double, 5, 1> full_dq;
};

struct LastState
{
    Eigen::Matrix<double, 5, 1> full_tau_last = Eigen::Matrix<double, 5, 1>::Zero();
    Eigen::Matrix<double, 5, 1> full_q_last = Eigen::Matrix<double, 5, 1>::Zero();
    Eigen::Matrix<double, 5, 1> full_dq_last = Eigen::Matrix<double, 5, 1>::Zero();    
};

#endif
