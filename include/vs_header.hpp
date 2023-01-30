#pragma once
#include "dVKinematics.hpp"
extern "C" {
#include <extApi.h>
#include <extApiPlatform.h>
}
#include <math.h>
#define M_PI 3.14159265359

//handle
extern simxInt vsClient;

//simulation time step to command in position
extern float Ts;

extern bool running;
void vs_loop();

void set_handles(Eigen::Vector4i& ecmJointHandles, int& eye_L, int& eye_R);

Eigen::Vector4f compute_vel3R(const Eigen::Vector4f q, Eigen::Vector2f p_ref, Eigen::Vector2f p, const float& Z, const float& k, const float& lambda);
float compute_vel1P(const Eigen::Vector2f p1, const Eigen::Vector2f p2, const float& md, const float& k_prism);

Matrix26f get_interaction_matrix(const Eigen::Vector2f p, const float& Z, const float& lambda);

Eigen::Vector2f compute_features_error(const Eigen::Vector2f p_ref, const Eigen::Vector2f p);

void get_blobs(const simxFloat* auxValues, const simxInt* auxValuesCount, Eigen::Vector2f& p1, Eigen::Vector2f& p2);

float retrieve_cam_params(const int& eye_L, const int& eye_R, float* cam_pos);