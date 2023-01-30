#pragma once
#ifndef GEOMAGIC_HPP_
#define GEOMAGIC_HPP_

#include "utils.hpp"
#include <HD\hd.h>
#include <HDU\hduError.h>
#include <HDU\hduMatrix.h>
#include <HDU\hduVector.h>

#define GEOMAGIC_HAPTIC_DOFS 3

enum BUTTONS { GEOMAGIC_LOW_BUTTON, GEOMAGIC_HIGH_BUTTON, GEOMAGIC_BUTTONS_NUM };
enum PRESSED_BUTTONS { NO_PRESSED, PRESSED_LOW, PRESSED_HIGH, PRESSED_BOTH };

extern simxInt psmLClient;
extern simxInt psmRClient;
extern bool running;
extern simxFloat Ts;


HDCallbackCode HDCALLBACK updateGeoStateCallback(void* data);
HDCallbackCode HDCALLBACK forceFeedbackCallback(void* data);


struct HDUtilityData {

	hduVector3Dd prvPos;
	hduVector3Dd lstPos;

	hduVector3Dd prvAng;
	hduVector3Dd lstAng;

	hduVector3Dd prvInputVel;
	hduVector3Dd lstInputVel;
	hduVector3Dd vrLstInputVel;
	hduVector3Dd prvOutVel;
	hduVector3Dd lstOutVel;
	hduVector3Dd vrLstOutVel;

	hduVector3Dd prvInputAngVel;
	hduVector3Dd lstInputAngVel;
	hduVector3Dd vrLstInputAngVel;
	hduVector3Dd prvOutAngVel;
	hduVector3Dd lstOutAngVel;
	hduVector3Dd vrLstOutAngVel;

	hduVector3Dd lvelocity;
	hduVector3Dd lvelocityTemp;

	hduVector3Dd avelocity;
	hduVector3Dd avelocityTemp;


};


struct GeoTeleopState{
	
	int asyncTriggerState;				//!< State corresponding to asyncronous triggered events (e.g., stylus buttons)
	bool buttonState[GEOMAGIC_BUTTONS_NUM];					//!< Array with the state of the buttons placed on the device
	Eigen::Vector3f hipPosition;		//!< Vector of the 3D position of the HIP
	Eigen::Vector3f hipGimbalAngles;	//!< Vector of the 3D orientation of the HIP
	Eigen::Vector6f hipVelocity;		//!< Vector of the 6D velocity of the HIP
	Eigen::Vector3f cmdForce;				//!< Vector of the 3D force to be applied

	// make this private later
	int stylusButtons;												//!< Status of the buttons on the Geomagic stylus
	bool action[GEOMAGIC_BUTTONS_NUM];								//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evHoldButton[GEOMAGIC_BUTTONS_NUM];						//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evRaiseEdge[GEOMAGIC_BUTTONS_NUM];							//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evTrailEdge[GEOMAGIC_BUTTONS_NUM];							//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)

	HDUtilityData hdUtils;
	HHD handle;
};



void hapticLoop();

hduVector3Dd eigen2hdu3D(const Eigen::Vector3d& eig_in); 


Eigen::Vector3d hdu2Eigen3D(const hduVector3Dd& hdu_in);


/**
* @brief Update function
* Update the linear and angular velocities of the Geomagic stylus
* Set internally stylusLinearVelocity and stylusAngularVelocity
*/
void updateGeomagicVelocities(GeoTeleopState& gts);


/**
* @brief Event catch function
* Catch the event when the input button has been pressed, computed on the consecutive raising and trailing edges of the button pressing state
* @param button: the current state of the pressed button
* @param button_prev: the previous state of the pressed button
* @param raise: the raising edge of the event
* @param trail: the trailing edge of the event
* @param trigger: the boolean value to be returned
*/
void catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger);

bool calibrate();

#endif // GEOMAGIC_HPP_