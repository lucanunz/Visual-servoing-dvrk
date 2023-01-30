// Project Header files
#include "GeomagicProxy.hpp"
#include "Timer.hpp"

/**
* @brief Default contructor of GeomagicProxy class
*
*/
GeomagicProxy::GeomagicProxy() {


	/*this->setHapticDOF(GEOMAGIC_HAPTIC_DOF);
	this->setJointDOF(GEOMAGIC_HAPTIC_JOINTS);
	this->setButtonsNum(GEOMAGIC_BUTTONS_NUM);//*/

	// Geomagic stringstream
	leftHipBaseVelSS.str("");		//!< 6D Velocity of the left hip with respect to the base frame of the left geomagic
	rightHipBaseVelSS.str("");	//!< 6D Velocity of the right hip with respect to the base frame of the right geomagic
	leftHipPSMVelSS.str("");		//!< 6D Velocity of the left hip with respect to the base frame of the Left PSM
	rightHipPSMVelSS.str("");	//!< 6D Velocity of the right hip with respect to the base frame of the Left PSM
	leftHipRotSS.str("");			//!< Rotation matrix expressing the orientation of the left hip wrt the base frame of the left geomagic
	rightHipRotSS.str("");		//!< Rotation matrix expressing the orientation of the left hip wrt the base frame of the right geomagic
	leftHipPosSS.str("");			//!< Position of the left hip wrt the base frame of the left geomagic
	rightHipPosSS.str("");		//!< Position of the right hip wrt the base frame of the right geomagic

	// PSM stringstream
	qdotPSM1SS.str("");			//!< 6x1 Vector of PSM1 joint velocity
	qdotPSM2SS.str("");			//!< 6x1 Vector of PSM1 joint position
	qPSM1SS.str("");				//!< 6x1 Vector of PSM2 joint velocity
	qPSM2SS.str("");				//!< 6x1 Vector of PSM2 joint position
	Tb1gSS.str("");				//!< Homogeneous transformation matrix expressing the pose of the left gripper (PSM1) wrt the base frame of the PSM1
	Tb2gSS.str("");				//!< Homogeneous transformation matrix expressing the pose of the right gripper (PSM2) wrt the base frame of the PSM2
	vb1gSS.str("");				//!< Velocity vector of the left gripper (PSM1) expressed wrt base frame of the PSM1
	vb2gSS.str("");				//!< Velocity vector of the right gripper (PSM2) expressed wrt base frame of the PSM2

	// Signals
	clutchButtonSS.str("");
	holdhButtonSS.str("");
	leftRaisingClutchEdgeSS.str("");
	rightRaisingClutchEdgeSS.str("");
	leftTrailingClutchEdgeSS.str("");
	rightTrailingClutchEdgeSS.str("");
	leftTriggerClutchButtonSS.str("");
	rightTriggerClutchButtonSS.str("");


}

/**
* @brief Default destroyer of GeomagicProxy class
*
*/
GeomagicProxy::~GeomagicProxy() {}

/**
* @brief Default init function
*/
void GeomagicProxy::init() {

	// Debug print
	/*** Because of the inner working conditions of the haptic device and its OpenHaptics SDK,
	 *** this init() function here cannot initialize the device, because the main loop execution
	 *** occurs in an another thread. It seems that initialization, utilization and clear of the device
	 *** has to occur in the same thread. Therefore this function, that is called by Scheduler in the main thread,
	 *** loses its use, and it should just initialize local structures, but not the device.
	 *** Device initialization and clear and then left to the same function of the main loop (i.e. before and after
	 *** the while() loop).
	 ***/

	 // Initialize all the structures to zero
	this->geoStatus.stylusPosition.set(0.0, 0.0, 0.0);
	this->geoStatus.stylusGimbalAngles.set(0.0, 0.0, 0.0);
	this->geoStatus.stylusLinearVelocity.set(0.0, 0.0, 0.0);
	this->geoStatus.stylusAngularVelocity.set(0.0, 0.0, 0.0);
	this->geoStatus.force.set(0.0, 0.0, 0.0);

	// Initialize variables related to the buttons state
	this->geoStatus.stylusButtons = 0;
	this->geoStatus.action[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus.action[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus.evHoldButton[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus.evHoldButton[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus.evRaiseEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus.evRaiseEdge[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus.evTrailEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus.evTrailEdge[GEOMAGIC_HIGH_BUTTON] = false;
	std::memset(this->geoStatus.jointPosition, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);
	std::memset(this->geoStatus.jointVelocity, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);
	std::memset(this->geoStatus.jacobian, 0.0, sizeof(double) * (SPACE_DIM * 2) * GEOMAGIC_HAPTIC_JOINTS);


	// Utility data
	this->hdUtils.prvPos.set(0.0, 0.0, 0.0);
	this->hdUtils.lstPos.set(0.0, 0.0, 0.0);
	this->hdUtils.prvAng.set(0.0, 0.0, 0.0);
	this->hdUtils.lstAng.set(0.0, 0.0, 0.0);
	this->hdUtils.prvInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lvelocity.set(0.0, 0.0, 0.0);
	this->hdUtils.lvelocityTemp.set(0.0, 0.0, 0.0);
	this->hdUtils.avelocity.set(0.0, 0.0, 0.0);
	this->hdUtils.avelocityTemp.set(0.0, 0.0, 0.0);
	std::memset(this->hdUtils.jointPosPrev, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);

	this->availability(true);

}

/**
* @brief Calibrate function
* Calibrate the Geomagic device
* @return true if the device has been successfully calibrated
*/
bool GeomagicProxy::calibrate() {

	// vvv UNCOMMENT THIS
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
		int calibrationStyle;
		hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
		if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL) {
			std::cout << "Please prepare for starting the demo by\nplacing the device at its reset position.\n" << std::endl;
			while (hdCheckCalibration() != HD_CALIBRATION_OK) {}
			return false;
		}
		return true;
	}//*/
	return true;

}

/**
* @brief Event catch function
* Catch the event when the input button has been pressed, computed on the consecutive raising and trailing edges of the button pressing state
* @param button: the current state of the pressed button
* @param button_prev: the previous state of the pressed button
* @param raise: the raising edge of the event
* @param trail: the trailing edge of the event
* @param trigger: the boolean value to be returned
*/
void GeomagicProxy::catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger) {


	if (button && !button_prev) {
		raise = true;
	}
	if (!button && button_prev) {
		trail = true;
	}
	if (raise && trail) {

		trigger = !trigger;
		raise = false;
		trail = false;
	}

	button_prev = button;

}


/**
* @brief Default run function
*/
void GeomagicProxy::run() {

	/**** vvvvv maybe not needed here vvv  ****/
	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;

	// Get the clock rate
	Timer clock;
	clock.setRate(20.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;
	dt = 0.0;
	/**** ^^^^^ maybe not needed here ^^^^^ ****/

	// Required structures
	HDErrorInfo error;

	// Init the device
	this->dvcHandle = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to initialize haptic device. Code: 0x0" << std::endl;
		this->availability(false);
		return;
	}

	// Calibrate the device
	int ok = calibrate();

	// If calibration fails ...
	if (!ok) {
		std::cout << "Calibration failed!" << std::endl;
	}
	//hdDisable(HD_SOFTWARE_VELOCITY_LIMIT);

	// Set running on true
	this->setRunning(true);

	std::cout << "Master control loop started!\n";

	// Set the force feedback callback in the scheduler
	this->schHandle = hdScheduleAsynchronous(forceFeedbackCallback, this, HD_MAX_SCHEDULER_PRIORITY);

	// Enable force feedback
	hdEnable(HD_FORCE_OUTPUT);

	// Start the scheduler
	//hdSetSchedulerRate(800);
	hdStartScheduler();

	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to start scheduler" << std::endl;
	}

	// Run the main loop
	while (this->isRunning())
	{
		// Measure starting time
		tic = clock.getCurTime();



		//----------------------------------------------------------------//
		// Do stuff here... 


		while (!hdWaitForCompletion(schHandle, HD_WAIT_CHECK_STATUS));
		hdScheduleSynchronous(updateGeoStateCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);

		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			//clock.timeSleep(Ts - tictoc); //<----- DO NOT UNCOMMENT THIS! CONFLICT WITH THE INNER DEVICE SERVO LOOP! 
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);

		//debugPrint<double>("[GeomagicProxy] Running rate:", 1.0 / dt);
	}

	// Stop the scheduler
	hdStopScheduler();

	// Unschedule the tasks
	hdUnschedule(schHandle);

	// Disable the device
	hdDisableDevice(dvcHandle);

}

/**
* @brief Default clear function
*/
void GeomagicProxy::clear() {

	// Set running on false
	this->setRunning(false);

}

/**
* @brief Callback function
* Update the state of the Geomagic device
* @param data the data containing the updated status
*/
HDCallbackCode HDCALLBACK updateGeoStateCallback(void* data) {

	GeomagicProxy* device = (GeomagicProxy*)data;
	Eigen::Matrix<double, (SPACE_DIM * 2), GEOMAGIC_HAPTIC_JOINTS> J;
	Eigen::Matrix<double, (SPACE_DIM * 2), 1> vel;
	Eigen::Matrix<double, GEOMAGIC_HAPTIC_JOINTS, 1> q, qdot, qprev;
	bool curButton[GEOMAGIC_BUTTONS_NUM];
	double hdrate;

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else {

		// Get data from the device
		hdGetDoublev(HD_CURRENT_POSITION, device->geoStatus.stylusPosition);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, device->geoStatus.stylusGimbalAngles);
		hdGetDoublev(HD_CURRENT_JACOBIAN, device->geoStatus.jacobian);
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES, device->geoStatus.jointPosition);
		hdGetDoublev(HD_UPDATE_RATE, &hdrate);

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				J(i, j) = device->geoStatus.jacobian[i * 6 + j];
			}
		}//*/
		//std::cout << "J = \n" << J << std::endl;
		std::memcpy(q.data(), device->geoStatus.jointPosition, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);
		std::memcpy(qprev.data(), device->hdUtils.jointPosPrev, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);//*/


		// Convert from [mm] to [m]
		device->geoStatus.stylusPosition *= 1e-3;

		// Update the velocity vector
		device->updateVelocities();
		/*vel = J * (q - qprev)*(1.0/hdrate);
		if (vel.norm() > 1e3) vel.setZero();
		std::cout << "vel = " << vel.transpose() << std::endl;
		hduVector3Dd linvel(vel(0), vel(1), vel(2));
		hduVector3Dd angvel(vel(3), vel(4), vel(5));
		device->geoStatus.stylusLinearVelocity = linvel;
		device->geoStatus.stylusAngularVelocity = angvel;
		std::memcpy(device->hdUtils.jointPosPrev,device->geoStatus.jointPosition,sizeof(double)*HAPTIC_JOINTS);//*/

		// Get the state of the button
		hdGetIntegerv(HD_CURRENT_BUTTONS, &(device->geoStatus.stylusButtons));

		for (int i = 0; i < GEOMAGIC_BUTTONS_NUM; i++) {
			curButton[i] = (device->geoStatus.stylusButtons == i + 1 || device->geoStatus.stylusButtons == PRESSED_BOTH) ? true : false;

			// Catch the single pressure event
			device->catchButtonPressEvent(curButton[i], device->geoStatus.evHoldButton[i],
				device->geoStatus.evRaiseEdge[i],
				device->geoStatus.evTrailEdge[i],
				device->geoStatus.action[i]);

		}
	}

	return HD_CALLBACK_DONE;


}


/**
* @brief Callback function
* Force feedback callback of the Geomagic device
* @param data the data containing the force feedback data
*/
HDCallbackCode HDCALLBACK forceFeedbackCallback(void* data) {

	GeomagicProxy* device = (GeomagicProxy*)data;

	device->dvcHandle = hdGetCurrentDevice();
	HDErrorInfo error;
	hdBeginFrame(device->dvcHandle);

	hdSetDoublev(HD_CURRENT_FORCE, device->geoStatus.force);
	hdEndFrame(device->dvcHandle);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed");
		device->availability(false);
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else { device->availability(true); }
	return HD_CALLBACK_CONTINUE;


}

/**
* @brief Set function
* Set the feedback haptic force on the Geomagic device
* @param f: the feedback force to be set
*/
/*void GeomagicProxy::setHapticForce(const Eigen::Vector3f& f) {

	this->geoStatus.force.set(f(0), f(1), f(2));


}//*/


/**
* @brief Update function
* Update the linear and angular velocities of the Geomagic stylus
* Set internally stylusLinearVelocity and stylusAngularVelocity
*/
void GeomagicProxy::updateVelocities() {

	// Compute linear velocities
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus.stylusPosition * 3 - 4 * hdUtils.prvPos + hdUtils.lstPos) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.lvelocity = (.2196 * (vel_buff + hdUtils.vrLstInputVel) + .6588 * (hdUtils.prvInputVel + hdUtils.lstInputVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutVel + 2.5282 * hdUtils.lstOutVel - 0.7776 * hdUtils.vrLstOutVel); //cutoff freq of 20 Hz
	hdUtils.lstPos = hdUtils.prvPos;
	hdUtils.prvPos = this->geoStatus.stylusPosition;
	hdUtils.vrLstInputVel = hdUtils.lstInputVel;
	hdUtils.lstInputVel = hdUtils.prvInputVel;
	hdUtils.prvInputVel = vel_buff;
	hdUtils.vrLstOutVel = hdUtils.lstOutVel;
	hdUtils.lstOutVel = hdUtils.prvOutVel;
	hdUtils.prvOutVel = hdUtils.lvelocity;
	hdUtils.lvelocityTemp = hdUtils.lvelocityTemp + ((hdUtils.lvelocity - hdUtils.lvelocityTemp) * (0.001 / (0.001 + 0.07)));

	// Set linear velocities
	this->geoStatus.stylusLinearVelocity = hdUtils.lvelocityTemp;


	// Compute angular velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus.stylusGimbalAngles * 3 - 4 * hdUtils.prvAng + hdUtils.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.avelocity = (.2196 * (vel_buff + hdUtils.vrLstInputAngVel) + .6588 * (hdUtils.prvInputAngVel + hdUtils.lstInputAngVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutAngVel + 2.5282 * hdUtils.lstOutAngVel - 0.7776 * hdUtils.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils.lstAng = hdUtils.prvAng;
	hdUtils.prvAng = this->geoStatus.stylusGimbalAngles;
	hdUtils.vrLstInputAngVel = hdUtils.lstInputAngVel;
	hdUtils.lstInputAngVel = hdUtils.prvInputAngVel;
	hdUtils.prvInputAngVel = vel_buff;
	hdUtils.vrLstOutAngVel = hdUtils.lstOutAngVel;
	hdUtils.lstOutAngVel = hdUtils.prvOutAngVel;
	hdUtils.prvOutAngVel = hdUtils.avelocity;
	hdUtils.avelocityTemp = hdUtils.avelocityTemp + ((hdUtils.avelocity - hdUtils.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	// Set angular velocities
	this->geoStatus.stylusAngularVelocity = hdUtils.avelocityTemp;

}


/**
* @brief Set function
* Set the feedback haptic force on the Geomagic device
* @param f: the feedback force to be set
*/
void GeomagicProxy::setHapticForce(const Eigen::VectorXf& f) {

	for (int i = 0; i < GEOMAGIC_HAPTIC_DOF; i++) {
		this->geoStatus.force[i] = f(i);
	}


}

/**
* @brief Set function
* Set the 3D vector of the hip position of the device
* @param p: the 3D vector of the hip position of the device
*/
void GeomagicProxy::setHIPPosition(const Eigen::Vector3f& p) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus.stylusGimbalAngles[i] = p(i);
	}

}


/**
* @brief Set function
* Set the 3D vector of the hip orientation of the device
* @param p: the 3D vector of the hip orientation of the device
*/
void GeomagicProxy::setHIPGimbalAngles(const Eigen::Vector3f& ga) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus.stylusGimbalAngles[i] = ga(i);
	}

}

/**
* @brief Set function
* Set the 6D vector of the hip velocity of the device
* @param v: the 6D vector of the hip velocity of the device
*/
void GeomagicProxy::setHIPVelocity(const Eigen::Vector6f& v) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus.stylusLinearVelocity[i] = v(i);
		this->geoStatus.stylusAngularVelocity[i] = v(i + SPACE_DIM);
	}

}


/**
* @brief Set function
* Set the dynamic array with the state (on/off) of the buttons placed on the device
* @param state: the dynamic array with the state (on/off) of the buttons placed on the device
*/
void GeomagicProxy::setButtonState(const bool* state) {

	this->geoStatus.action[GEOMAGIC_LOW_BUTTON] = state[GEOMAGIC_LOW_BUTTON];
	this->geoStatus.action[GEOMAGIC_HIGH_BUTTON] = state[GEOMAGIC_HIGH_BUTTON];

}

/**
* @brief Get function
* Get the feedback haptic force on the Geomagic device
* @return the feedback force
*/
Eigen::VectorXf GeomagicProxy::getHapticForce() {

	Eigen::VectorXf ret;
	ret.setZero(GEOMAGIC_HAPTIC_DOF);

	for (int i = 0; i < GEOMAGIC_HAPTIC_DOF; i++) {
		ret(i) = this->geoStatus.force[i];
	}

	return ret;

}

/**
* @brief Get function
* Get the 3D vector of the hip position of the device
* @return the 3D vector of the hip position of the device
*/
Eigen::Vector3f GeomagicProxy::getHIPPosition() {

	Eigen::Vector3f ret;

	for (int i = 0; i < SPACE_DIM; i++) {
		ret(i) = this->geoStatus.stylusPosition[i];
	}

	return ret;

}

/**
* @brief Get function
* Get the 3D vector of the hip orientation of the device
* @return the 3D vector of the hip orientation of the device
*/
Eigen::Vector3f GeomagicProxy::getHIPGimbalAngles() {

	Eigen::Vector3f ret;

	for (int i = 0; i < SPACE_DIM; i++) {
		ret(i) = this->geoStatus.stylusGimbalAngles[i];
	}

	return ret;

}

/**
* @brief Get function
* Get the 6D velocity of the HIP
* @return the 6D velocity of the HIP
*/
Eigen::Vector6f GeomagicProxy::getHIPVelocity() {

	Eigen::Vector6f ret;

	for (int i = 0; i < SPACE_DIM; i++) {
		ret(i) = this->geoStatus.stylusLinearVelocity[i];
		ret(i + SPACE_DIM) = this->geoStatus.stylusAngularVelocity[i];
	}

	return ret;
}


/**
* @brief Get function
* Get the dynamic array with the state (on/off) of the buttons placed on the device
* @return the dynamic array with the state (on/off) of the buttons placed on the device
*/
bool* GeomagicProxy::getButtonState() {

	return (this->geoStatus.action);

}


void GeomagicProxy::saveLog() {

	// Geomagic stringstream
	std::string leftHipBaseVelFile = logPath + "leftHipBaseVel.txt";
	std::string rightHipBaseVelFile = logPath + "rightHipBaseVel.txt";
	std::string leftHipPSMVelFile = logPath + "leftHipPSMVel.txt";
	std::string rightHipPSMVelFile = logPath + "righttHipPSMVel.txt";
	std::string leftHipRotFile = logPath + "leftHipRot.txt";
	std::string rightHipRotFile = logPath + "rightHipRot.txt";
	std::string leftHipPosFile = logPath + "leftHipPos.txt";
	std::string rightHipPosFile = logPath + "rightHipPos.txt";

	// PSM stringstream
	std::string qdotPSM1File = logPath + "qdotPSM1.txt";
	std::string qdotPSM2File = logPath + "qdotPSM2.txt";
	std::string qPSM1File = logPath + "qPSM1.txt";
	std::string qPSM2File = logPath + "qPSM2.txt";
	std::string Tb1gFile = logPath + "Tb1g.txT";
	std::string Tb2gFile = logPath + "Tb2g.TXT";
	std::string vb1gFile = logPath + "vb1g.txt";
	std::string vb2gFile = logPath + "vb2g.txt";

	// Signals
	std::string clutchButtonFile = logPath + "clutchButton.txt";
	std::string holdhButtonFile = logPath + "holdhButton.txt";
	std::string leftRaisingClutchEdgeFile = logPath + "leftRaisingClutchEdge.txt";
	std::string rightRaisingClutchEdgeFile = logPath + "rightRaisingClutchEdge.txt";
	std::string leftTrailingClutchEdgeFile = logPath + "leftTrailingClutchEdge.txt";
	std::string rightTrailingClutchEdgeFile = logPath + "rightTrailingClutchEdge.txt";
	std::string leftTriggerClutchButtonFile = logPath + "leftTriggerClutchButton.txt";
	std::string rightTriggerClutchButtonFile = logPath + "rightTriggerClutchButton.txt";

	saveToFile(leftHipBaseVelSS, leftHipBaseVelFile);
	saveToFile(rightHipBaseVelSS, rightHipBaseVelFile);
	saveToFile(leftHipPSMVelSS, leftHipPSMVelFile);
	saveToFile(rightHipPSMVelSS, rightHipPSMVelFile);
	saveToFile(leftHipPosSS, leftHipPosFile);
	saveToFile(rightHipPosSS, rightHipPosFile);

	saveToFile(qPSM1SS, qPSM1File);
	saveToFile(qPSM2SS, qPSM2File);
	saveToFile(qdotPSM1SS, qdotPSM1File);
	saveToFile(qdotPSM2SS, qdotPSM2File);
	saveToFile(vb1gSS, vb1gFile);
	saveToFile(vb2gSS, vb2gFile);



}