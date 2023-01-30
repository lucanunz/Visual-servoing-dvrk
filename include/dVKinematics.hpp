#pragma once
#include <Eigen/Dense>

#define PSM_JOINTS_NUM 7
#define ECM_JOINTS_NUM 4

#define VEL_DIM 6
#define LIN_VEL_DIM 3
#define ANG_VEL_DIM 3
#define ARMS 2

#define RIFT_VIEW 0

enum ARM_SIDE {LEFT, RIGHT};
//enum ARM_SIDE { RIGHT, LEFT };

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

typedef Eigen::Matrix<int, 4, 1> Vector4i;
typedef Eigen::Matrix<float, 2, 1> Vector2f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 2, 6> Matrix26f;

Matrix6f PSM_Jacobian(const Vector6f& q);
Eigen::Matrix<float, 3, 4> ECM_Jacobian(const Eigen::Vector4f& q);
Eigen::Matrix4f PSM1DirKin(const Vector6f& q);
Eigen::Matrix4f forwardKinematicsECM(const Eigen::Vector4f &q);