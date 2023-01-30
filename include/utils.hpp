#pragma once

// Project header files
#include "dVKinematics.hpp"
#include "Timer.hpp"

// Eigen Header files
//#include <Eigen/Dense>

// Standard Header files
#include <vector>
#include <iostream>
#include <fstream>
#include <conio.h>
#include <string>

// Windows includes
#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <conio.h>
#include <ctime>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <direct.h>

// Boost includes
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


#define _USE_MATH_DEFINES
#include <math.h>


#define M_PI 3.14159265358979323846
#define MAX_FPS 120
#define FULLSCREEN
#define CUTOFF 100
#define SAMPLE_RATE 5000
#define SPACE_DIM 3

// V-REP includes
//#include <v_repLib.h>
extern "C" {
#include <extApi.h>
#include <extApiPlatform.h>
}

namespace Eigen {
	
	typedef Eigen::Matrix<float, 7, 1> Vector7f;
	typedef Eigen::Matrix<double, 7, 1> Vector7d;
	typedef Eigen::Matrix<float, 6, 1> Vector6f;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}


/**
* @brief Load function
* Load and extract data from the input log
* @param file: the log file
* @return a structure containing the loaded data
*/
std::vector < Eigen::VectorXf > loadCSVLog(const std::string& file);

/**
* @brief Read function
* Read the conent of the file, assuming it's writte in the CSV format
* @ return a vector of string containing the set of values of the read CSV structure
*/
std::vector < std::string > readCSVFile(const char* filename);

/**
* @brief Utility function
* parse the input string line into an array of doubled-precision floating values
* @param the input string line
* @return the corresponding vector of double-precision floating values
*/
std::vector < double > parseCSVLine(const std::string& line);

/**
* @brief Filter function
* Filter the input velocity vector with a low-pass filter
* @param velocity: the input velocity vector to be filtered
* @param velocity_state: the latest filteret velocity
* @return the filtered velocity
*/
Eigen::VectorXf filter_velocity(Eigen::VectorXf velocity, Eigen::Ref<Eigen::VectorXf> velocity_state);

/**
* @brief Filter function
* Filter the input velocity vector with a low-pass filter
* @param velocity: the input velocity vector to be filtered
* @param velocity_state: the latest filteret velocity
* @return the filtered velocity
*/
Eigen::VectorXf filter_accelleration(Eigen::VectorXf accelleration, Eigen::Ref<Eigen::VectorXf> accelleration_state);

/**
* @brief Filter function
* Filter the input velocity vector with a low-pass filter
* @param velocity: the input velocity vector to be filtered
* @param velocity_state: the latest filteret velocity
* @return the filtered velocity
*/
Eigen::VectorXf filter_Torques(Eigen::VectorXf torques, Eigen::Ref<Eigen::VectorXf> torques_state);


/**
* @brief Rotation matrix 2 Roll Pitch Yaw angles
* Convert the input rotation matrix in the corresponding triple of roll pitch yaw angles
*/
Eigen::Vector3f rot2rpy(const Eigen::Matrix3f& R);


float lowPassFrequency(float v_new, float v);

void saveToFile(std::stringstream& buff, std::string name);

Eigen::Matrix3f eulAng2Rot(const float& alpha, const float& beta, const float& gamma);

float clamp(const float& in, const float& min, const float& max);

Eigen::Vector3d clampVec(const Eigen::Vector3d& in, const float& min, const float& max);

Eigen::Vector3f projVec(const Eigen::Vector3f& v, const Eigen::Vector3f& p);

void usleep(DWORD waitTime);

