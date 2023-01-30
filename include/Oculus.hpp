#pragma once

// Oculus includes
#include <GL/glew.h>
#include <Extras/OVR_Math.h>
#include <OVR_CAPI_GL.h>
#include <OVR_CAPI.h>
#include <SDL.h>
#include <SDL_syswm.h>
#include "Shader.hpp"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils.hpp"

using namespace cv;
using namespace OVR;

extern simxInt ecmClient;
extern simxFloat Ts;
extern bool running;

// Oculus stringstream
extern std::stringstream vtrSS;				//!< 6D Velocity of the rift display expressed in the tracker reference frame
extern std::stringstream vrrSS;				//!< 6D Velocity of the rift display expressed in the rift reference frame
extern std::stringstream Ttr;					//!< Homogeneous transformation matrix expressing the pose of the Oculus rift wrt the tracker reference frame

// ECM stringstream
extern std::stringstream qdotECMSS;			//!< 4x1 Vector of ECM joint velocities
extern std::stringstream qECMSS;				//!< 4x1 Vector of ECM joint position
extern std::stringstream TbcSS;				//!< Homogeneous transformation matrix expressing the pose of the left camera wrt the base frame of the ECM
extern std::stringstream vcamLSS;				//!< Velocity vector of the left camera of the ECM
extern std::stringstream vcamRSS;				//!< Velocity vector of the right camera of the ECM


void oculusLoop();