#include "Geomagic.hpp"

void hapticLoop() {

	HHD hapticHandlers[ARMS];
	simxInt qHandlers[ARMS][PSM_JOINTS_NUM];
	Matrix6f J_be[ARMS];
	Vector6f q6[ARMS], q6dot[ARMS], q6cmd[ARMS];
	Vector7f qcmd[ARMS], q[ARMS], qdot[ARMS]; // Vector of joint position taking into account the pair of gripper actuation (J6 = f(q6,q7))
	Vector6f hipVel[ARMS], gripperVel[ARMS], gripperVel2[ARMS], resVel[ARMS];
	Eigen::Vector3f gripperPos[ARMS];
	Eigen::Vector3d hipPos[ARMS];
	Eigen::Vector3d hipLinVel[ARMS];
	Eigen::Vector3d hipAngVel[ARMS];
	Eigen::Matrix3d Rbh[ARMS]; // Rotation matrix from base frame to hip frame
	Eigen::Vector3d f_att[ARMS], f_rep[ARMS]; // Attractive and repulsive force
	Eigen::Matrix3f Rwb[ARMS]; // Rotation matrix from the V-REP world frame to the base frame (RCM) of the PSM
	Eigen::Matrix3f Rwb2[ARMS]; // Rotation matrix from the V-REP world frame to the new rotated base frame (RCM) of the PSM
	Eigen::Vector3f nb[ARMS]; // Unit normal vector of the table plane in the PSM frames
	Eigen::Vector3f projVel[ARMS];

	// Rotation matrix expressing the haptic base frame in the PSM base frame
	Eigen::Matrix3f Rmirror;

	// Rotation matrix to rotate the base frame of the geomagic of pi/2 around y, to match the view seen from the Oculus Rift (approximation)
	Eigen::Matrix3f Rbp2;
	Eigen::Matrix3f Rbp2T;


	Matrix6f R6, R6g;
	LARGE_INTEGER rate, tic, toc, tac, start_t, cur_t;
	std::string qNames[ARMS][PSM_JOINTS_NUM] = { { "J1_PSM1", "J2_PSM1", "J3_PSM1", "J1_TOOL1", "J2_TOOL1", "J3_dx_TOOL1", "J3_sx_TOOL1" },
												 { "J1_PSM2", "J2_PSM2", "J3_PSM2", "J1_TOOL2", "J2_TOOL2", "J3_dx_TOOL2", "J3_sx_TOOL2" } };
	std::string velSigNames[ARMS][VEL_DIM] = { { "psm1_vx", "psm1_vy", "psm1_vz", "psm1_wx", "psm1_wy", "psm1_wz" },
											   { "psm2_vx", "psm2_vy", "psm2_vz", "psm2_wx", "psm2_wy", "psm2_wz" } };
	std::string qdotSigNames[ARMS][PSM_JOINTS_NUM - 1] = { { "qLdot_1", "qLdot_2", "qLdot_3", "qLdot_4", "qLdot_5", "qLdot_6" },
														 { "qRdot_1", "qRdot_2", "qRdot_3", "qRdot_4", "qRdot_5", "qRdot_6" } };

	std::string gripHoldSigNames[ARMS] = { "LGripperHold","RGripperHold" };
	std::string canHoldSigNames[ARMS] = { "canHoldLeft","canHoldRight" };
	std::string distTableValNames[ARMS] = { "distTableLVal","distTableRVal" };
	std::string RCMNames[ARMS] = { "RCM_PSM1","RCM_PSM2" };
	simxInt RCMHandlers[ARMS];
	simxInt canHoldVal[ARMS];
	simxFloat eulAng[ARMS][ANG_VEL_DIM];
	simxFloat toolLV[ARMS][3];
	simxFloat toolAV[ARMS][3];
	simxInt csClients[ARMS];
	float vmax = 2.5;
	float wmax = 2.5;
	float Kd = 20;
	float distTableVal[ARMS];
	float tableThresh;
	float K_s = 200.0; //stiffness gain
	float K_v = 2.5; //viscous gain
	float fx_rep = 0.0;
	double dt, et, t_;
	bool clutchButton[ARMS], clutchButtonPrev[ARMS];
	bool raisingClutchEdge[ARMS], trailingClutchEdge[ARMS];
	bool triggerClutchButton[ARMS];

	bool holdButton[ARMS], holdButtonPrev[ARMS];
	bool raisingHoldEdge[ARMS], trailingHoldEdge[ARMS];
	bool triggerHoldButton[ARMS];

	int connectedDevicesNum = 0;
	std::cout << "hapticCHAICallback ... " << std::endl;

	//----------------------------------
	// REMINDER: LEFT -> 0 , RIGHT -> 1
	//----------------------------------

	// Define the GeoTeleopState structure storing updated information about the device state
	GeoTeleopState* geoState = new GeoTeleopState[ARMS];
	geoState[ARM_SIDE::LEFT].cmdForce.setZero();
	geoState[ARM_SIDE::RIGHT].cmdForce.setZero();
	geoState[ARM_SIDE::LEFT].buttonState[0] = false;
	geoState[ARM_SIDE::RIGHT].buttonState[0] = false;
	geoState[ARM_SIDE::LEFT].buttonState[1] = false;
	geoState[ARM_SIDE::RIGHT].buttonState[1] = false;
	geoState[ARM_SIDE::RIGHT].cmdForce.setZero();
	geoState[ARM_SIDE::LEFT].handle = -1;
	geoState[ARM_SIDE::RIGHT].handle = -1;
	geoState[ARM_SIDE::LEFT].hipVelocity.setZero();
	geoState[ARM_SIDE::RIGHT].hipVelocity.setZero();

	// Assign the CoppeliaSim clients
	csClients[ARM_SIDE::LEFT] = psmLClient;
	csClients[ARM_SIDE::RIGHT] = psmRClient;

	// Get the clock rate
	//QueryPerformanceFrequency(&rate);

	//----------------------------------
	// CoppeliaSim INITIALIZATION
	//----------------------------------

	// Rotation matrix to mirror the movements with respect to the view of V-REP (we assume manipulation on the robot side)
	Rmirror << 0, 1, 0,
		-1, 0, 0,
		0, 0, 1;//*/

	Rbp2 << 1, 0, 0,
		0, 0, 1,
		0, -1, 0;//*/


	/*Rbp2 << 1,  0, 0,
		0,  1, 0,
		0,  0, 1;//*/

	Rbp2T = Rbp2.transpose();


	R6.setZero();
	R6g.setZero();

	// Use this rotation matrix if you want to keep Geomagic grippers movements expressed with respect to its original reference frame
	//R6.topLeftCorner(3, 3) = Rmirror;
	//R6.bottomRightCorner(3, 3) = Rmirror;

	// Use this rotation matrix if you want to match Geomagic grippers movements with Oculus view (approximation)
	R6.topLeftCorner(3, 3) = Rmirror;
	R6.bottomRightCorner(3, 3) = Rmirror;
	R6g.topLeftCorner(3, 3) = Rbp2T;
	R6g.bottomRightCorner(3, 3) = Rbp2T;

	// Init clutch button
	clutchButton[LEFT] = false;
	clutchButton[RIGHT] = false;
	clutchButtonPrev[LEFT] = false;
	clutchButtonPrev[RIGHT] = false;
	raisingClutchEdge[LEFT] = false;
	raisingClutchEdge[RIGHT] = false;
	trailingClutchEdge[LEFT] = false;
	trailingClutchEdge[RIGHT] = false;
	triggerClutchButton[LEFT] = false;
	triggerClutchButton[RIGHT] = false;

	// Init hold button
	holdButton[LEFT] = false;
	holdButton[RIGHT] = false;
	holdButtonPrev[LEFT] = false;
	holdButtonPrev[RIGHT] = false;
	raisingHoldEdge[LEFT] = false;
	raisingHoldEdge[RIGHT] = false;
	trailingHoldEdge[LEFT] = false;
	trailingHoldEdge[RIGHT] = false;
	triggerHoldButton[LEFT] = false;
	triggerHoldButton[RIGHT] = false;

	distTableVal[LEFT] = 1000.0;
	distTableVal[RIGHT] = 1000.0;
	tableThresh = 0.001;

	canHoldVal[LEFT] = 0;
	canHoldVal[RIGHT] = 0;

	// Initialize joint names
	for (int i = 0; i < ARMS; i++) {

#ifdef TEST
		// Get the handlers of the V-REP 
		simxGetObjectHandle(clientID, (blockNames[i]).c_str(), &blocksH[i], simx_opmode_blocking);
#endif

		// Get Handlers of the RCM objects
		simxGetObjectHandle(csClients[i], RCMNames[i].c_str(), &RCMHandlers[i], simx_opmode_blocking);

		// Enable streaming to get the orientation of the RCM of the PSM
		simxGetObjectOrientation(csClients[i], RCMHandlers[i], -1, eulAng[i], simx_opmode_streaming);

		// Enable streaming to get the distances of grippers from tables
		simxGetFloatSignal(csClients[i], distTableValNames[i].c_str(), &distTableVal[i], simx_opmode_streaming);

		// Enable streaming to read the signal stating if you can hold something
		simxGetIntegerSignal(csClients[i], canHoldSigNames[i].c_str(), &canHoldVal[i], simx_opmode_streaming);

		// Initialize chai3d data structures
		hipPos[i].setZero();
		hipLinVel[i].setZero();
		hipAngVel[i].setZero();
		Rbh[i].setIdentity();

		// Initialize Eigen data structures
		J_be[i].setZero();
		q[i].setZero();
		q6[i].setZero();
		qdot[i].setZero();
		q6dot[i].setZero();
		qcmd[i].setZero();
		q6cmd[i].setZero();
		hipVel[i].setZero();
		gripperVel[i].setZero();
		gripperVel2[i].setZero();
		resVel[i].setZero();
		Rwb[i].setIdentity();
		nb[i].setZero();
		projVel[i].setZero();

		for (int j = 0; j < PSM_JOINTS_NUM; j++) {

			//Retrieve PSM-i joint handlers
			simxGetObjectHandle(csClients[i], qNames[i][j].c_str(), &qHandlers[i][j], simx_opmode_blocking);

			// Enable streaming for data buffering of PSM1 joint positions
			simxGetJointPosition(csClients[i], qHandlers[i][j], &q[i](j), simx_opmode_streaming);

			// Get PSM-i joint positions in blocking-call fashion (to have guarantee that a first value is captured)
			simxGetJointPosition(csClients[i], qHandlers[i][j], &q[i](j), simx_opmode_blocking);
			qcmd[i](j) = q[i](j);


		}

		// Set the 6D vector with coupled gripper actuation
		q6[i].topRows(PSM_JOINTS_NUM - 2) = q[i].topRows(PSM_JOINTS_NUM - 2);
		q6[i](PSM_JOINTS_NUM - 2) = 0.5 * (q[i](PSM_JOINTS_NUM - 2) - q[i](PSM_JOINTS_NUM - 1));

		// Init forces to zero
		f_rep[i].setZero();
		f_att[i].setZero();

	}

	// Required structures
	HDErrorInfo error;

	// check if one or two devices are connected
	// Device 1 (for right PSM)
	HHD dvcHandle_1 = hdInitDevice("Phantom2");
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to initialize First haptic device. Code: 0x0" << std::endl;
		return;
	}
	else { 
		connectedDevicesNum++; 
		hapticHandlers[ARM_SIDE::RIGHT] = dvcHandle_1;
		geoState[ARM_SIDE::RIGHT].handle = dvcHandle_1;
		printf("1. Found device %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
		hdEnable(HD_FORCE_OUTPUT);
	}

	// Device 2 (for right PSM)
	HHD dvcHandle_2 = hdInitDevice("Default Device");
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to initialize Second haptic device. Code: 0x0" << std::endl;

		// *** Here we do not return to allow teleoperation with only one haptic device
		hdMakeCurrentDevice(dvcHandle_1);
	}
	else { 
		connectedDevicesNum++;
		hapticHandlers[ARM_SIDE::LEFT] = dvcHandle_2;
		geoState[ARM_SIDE::LEFT].handle = dvcHandle_2;
		printf("2. Found device %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
		hdEnable(HD_FORCE_OUTPUT);
	}


	std::cout << "Number of connected haptic devices = " << connectedDevicesNum << std::endl;

	// Calibrate the device
	int ok = calibrate();
	
	// If calibration fails ...
	if (!ok) {
		std::cout << "Calibration failed!" << std::endl;
	}//*/

	//hdDisable(HD_SOFTWARE_VELOCITY_LIMIT);
	std::cout << "Master control loop started!\n";
	
	// Start the scheduler
	//hdSetSchedulerRate(800);
	std::cout << "Starting the haptic scheduler ... ";
	hdStartScheduler();
	std::cout << "Done. " << std::endl;


	// Set the force feedback callback in the scheduler
	std::cout << "Defining the forceFeedbacckCallback ... ";
	HDSchedulerHandle schHandle = hdScheduleAsynchronous(forceFeedbackCallback, geoState, HD_MAX_SCHEDULER_PRIORITY);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to start scheduler" << std::endl;
	}

	// Run the main loop
	while (running) {

		while (!hdWaitForCompletion(schHandle, HD_WAIT_CHECK_STATUS));
		
		// Update the state of the i-th Geomagic haptic interface
		hdScheduleSynchronous(updateGeoStateCallback, geoState, HD_DEFAULT_SCHEDULER_PRIORITY);
		
		for (int i = 0; i < connectedDevicesNum; i++) {

			// Select the i-th device as current haptic device
			hdMakeCurrentDevice(hapticHandlers[i]);

			// Get the button state of the device
			bool clutchButton = geoState[i].buttonState[0];
			bool holdButton = geoState[i].buttonState[1];

			// Read the V-REP buffer to get the distances of grippers from tables 
			// -- obsolete, introduce delay and so instability in the force rendering
			// -- better with hard-coded value 
			float z_table = -0.136;
			//simxGetFloatSignal(csClients[i], distTableValNames[i].c_str(), &distTableVal[i], simx_opmode_buffer);
			
			// Enable streaming to read the signal stating if you can hold something
			simxGetIntegerSignal(csClients[i], canHoldSigNames[i].c_str(), &canHoldVal[i], simx_opmode_buffer);
			
			// Read the V-REP buffer to get the Euler angles expressing the orientation of the RCM wrt the world frame of V-REP
			simxGetObjectOrientation(csClients[i], RCMHandlers[i], -1, eulAng[i], simx_opmode_buffer);

			// Build the Rotation matrix from the Euler angles
			//Rwb[i] = eulAng2Rot(eulAng[i][0], eulAng[i][1], eulAng[i][2]);

			// Extract the unit normal vector
			//nb[i] = (Rwb[i]).row(2);

			// Send Gripper hold state on V-REP
			simxSetIntegerSignal(csClients[i], gripHoldSigNames[i].c_str(), holdButton, simx_opmode_oneshot);

			// Get position
			hipPos[i] = geoState[i].hipPosition.cast<double>();
			
			// Get linear velocity
			hipLinVel[i] = geoState[i].hipVelocity.topRows(SPACE_DIM).cast<double>();

			// Get angular velocity
			hipAngVel[i] = geoState[i].hipVelocity.bottomRows(SPACE_DIM).cast<double>();

			// Clamp vectors
			hipLinVel[i] = clampVec(hipLinVel[i], -vmax, vmax);

			// Build hip velocity vector
			hipVel[i].topRows(LIN_VEL_DIM) = hipLinVel[i].cast<float>();
			hipVel[i].bottomRows(ANG_VEL_DIM) = hipAngVel[i].cast<float>();

			// If some grasping object is close to the gripper, send a force feedback
			f_att[i].setZero();
			f_att[i](1) = (canHoldVal[i]) ? (-0.5) : (0.0);

			// Get the PSM-i joint position from the V-REP buffer
			for (int j = 0; j < PSM_JOINTS_NUM; j++) {

				// Get the PSM-i joint position from the V-REP buffer
				simxGetJointPosition(csClients[i], qHandlers[i][j], &q[i](j), simx_opmode_oneshot);

			}
			// Set the 6D vector with coupled gripper actuation
			q6[i].topRows(PSM_JOINTS_NUM - 2) = q[i].topRows(PSM_JOINTS_NUM - 2);
			q6[i](PSM_JOINTS_NUM - 2) = 0.5 * (q[i](PSM_JOINTS_NUM - 2) - q[i](PSM_JOINTS_NUM - 1));

			// Compute the gripper position
			gripperPos[i] = PSM1DirKin(q6[i]).block<3, 1>(0, 3);
			// Compute the current Jacobian matrix
			J_be[i] = PSM_Jacobian(q6[i]);

			// Revert the orientation to match the view seen from the Oculus (approximation)
			gripperVel[i] = R6g * hipVel[i];
			
			if (clutchButton) {
				// Compute the PSM-i joint velocities
				q6dot[i] = (J_be[i]).inverse() * gripperVel[i]; // J^-1 * v
			}
			else {
				q6dot[i].setZero();
			}

			if (clutchButton) {

				distTableVal[i] = gripperPos[i](2) - z_table; 
				fx_rep = -K_s * distTableVal[i];
				if (distTableVal[i] < 0) {
					f_rep[i] << 0.0, fx_rep, 0.0;
				}
				else {
					f_rep[i].setZero();
				}
			}

			// Compute the new PSM-i joint position vector
			//q6cmd[i] +=  q6dot[i] * Ts;
			std::cout << "Velocities arm " << i+1 << std::endl;
			// Apply joint velocities to PSM-i manipulator
			for (int j = 0; j < PSM_JOINTS_NUM - 1; j++) {

				// Apply joint velocities -> Done in V-REP
				simxSetJointTargetVelocity(csClients[i], qHandlers[i][j], 0.05*q6dot[i](j), simx_opmode_oneshot);
				std::cout << q6dot[i](j) << "\t";
				//simxSetJointPosition(clientID, qHandlers[i][j], qcmd[i](j), simx_opmode_oneshot);

				// Send joint velocity data to V-REP
				//simxSetFloatSignal(csClients[i], qdotSigNames[i][j].c_str(), q6dot[i](j), simx_opmode_oneshot);
			}
			std::cout << std::endl;

			// Send Cartesian velocity data to V-REP
			for (int j = 0; j < VEL_DIM; j++) {
				simxSetFloatSignal(csClients[i], velSigNames[i][j].c_str(), gripperVel2[i](j), simx_opmode_oneshot);
			}

			// Set the Cartesian force vector to be applied by the Geomagic Touch interface
			geoState[i].cmdForce = (f_rep[i] + f_att[i]).cast<float>();

		}

	}

	// Stop the scheduler
	hdStopScheduler();

	// Unschedule the tasks
	hdUnschedule(schHandle);

	// Close the device(s)
	for (int i = 0; i < connectedDevicesNum; i++) {
		if ((int)geoState[i].handle != -1) {
			hdDisableDevice(geoState[i].handle);
		}
	}

	// Delete dynamic pointers
	delete[] geoState;
}



/**
* @brief Callback function
* Update the state of the Geomagic device
* @param data the data containing the updated status
*/
HDCallbackCode HDCALLBACK updateGeoStateCallback(void* data) {

	
	GeoTeleopState* teleop = static_cast<GeoTeleopState*>(data);
	Eigen::Matrix<double, (SPACE_DIM * 2), 1> vel;
	bool curButton[GEOMAGIC_BUTTONS_NUM];
	double hdrate;
	int connectedDevs = 0;

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else {

		if (teleop[ARM_SIDE::LEFT].handle != -1) {
			connectedDevs++;
		}
		if (teleop[ARM_SIDE::RIGHT].handle != -1) {
			connectedDevs++;
		}

		for (int i = 0; i < connectedDevs; i++) {
			
			hdMakeCurrentDevice(teleop[i].handle);
			
			// Get data from the device
			hduVector3Dd pos, abg;
			std::cout << "prima get vel" << std::endl;
			hdGetDoublev(HD_CURRENT_POSITION, pos);
			std::cout << "tra get vel" << std::endl;
			hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, abg);
			std::cout << "dopo get vel" << std::endl;
			// Convert from [mm] to [m]
			pos *= 1e-3;
			
			// Fill internally the TeleopState structure
			teleop[i].hipPosition = hdu2Eigen3D(pos).cast<float>();
			teleop[i].hipGimbalAngles = hdu2Eigen3D(abg).cast<float>();
			updateGeomagicVelocities(teleop[i]); // Set internally the velocity vectors in TeleopState

			// Get the state of the button
			int stylusButtons;
			hdGetIntegerv(HD_CURRENT_BUTTONS, &stylusButtons);

			for (int j = 0; j < GEOMAGIC_BUTTONS_NUM; j++) {
				curButton[j] = (stylusButtons == j + 1 || stylusButtons == PRESSED_BUTTONS::PRESSED_BOTH) ? true : false;

				// Catch the single pressure event
				catchButtonPressEvent(curButton[j], teleop[i].evHoldButton[j],
					teleop[i].evRaiseEdge[j],
					teleop[i].evTrailEdge[j],
					teleop[i].action[j]);

				teleop[i].buttonState[j] = teleop[i].action[j];
			}

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

	GeoTeleopState* teleop = (static_cast<GeoTeleopState*>(data));
	hduVector3Dd hduForce;
	Eigen::Vector3f eigForce;
	HDErrorInfo error;
	HHD devHandle_1 = teleop[0].handle;
	HHD devHandle_2 = teleop[1].handle;
	int deviceNum = 0;
	hduForce.set(0.0, 0.0, 0.0);
	eigForce.setZero();

	if (devHandle_1 != -1) {
	
		deviceNum++;

		eigForce = teleop[0].cmdForce;

		// Convert the force vector in the appropriate data type of the Geomagic class
		for (int i = 0; i < GEOMAGIC_HAPTIC_DOFS; i++) {
			hduForce[i] = eigForce(i);
		}

		hdBeginFrame(devHandle_1);
		hdSetDoublev(HD_CURRENT_FORCE, hduForce);
		hdEndFrame(devHandle_1);
	}

	if (devHandle_2 != -1) {

		deviceNum++;

		eigForce = teleop[1].cmdForce;

		// Convert the force vector in the appropriate data type of the Geomagic class
		for (int i = 0; i < GEOMAGIC_HAPTIC_DOFS; i++) {
			hduForce[i] = eigForce(i);
		}

		hdBeginFrame(devHandle_2);
		hdSetDoublev(HD_CURRENT_FORCE, hduForce);
		hdEndFrame(devHandle_2);
	}



	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else { 
	
		//std::cout << "Error in forcefeedback callback. " << std::endl;

	}
	return HD_CALLBACK_CONTINUE;



}

/**
* @brief Calibrate function
* Calibrate the Geomagic device
* @return true if the device has been successfully calibrated
*/
bool calibrate() {

	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
		int calibrationStyle;
		hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
		if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL) {
			std::cout << "Please prepare for starting the demo by\nplacing the device at its reset position." << std::endl;
			while (hdCheckCalibration() != HD_CALIBRATION_OK) {}
			return false;
		}
		return true;
	}
	return true;

}

/**
* @brief Update function
* Update the linear and angular velocities of the Geomagic stylus
* Set internally stylusLinearVelocity and stylusAngularVelocity
*/
void updateGeomagicVelocities(GeoTeleopState& geoTeleopState) {


	hduVector3Dd stylusPosition = eigen2hdu3D(geoTeleopState.hipPosition.cast<double>());
	hduVector3Dd stylusGimbalAngles = eigen2hdu3D(geoTeleopState.hipGimbalAngles.cast<double>());
	hduVector3Dd stylusLinearVelocity, stylusAngularVelocity;

	HDUtilityData hdUtils = geoTeleopState.hdUtils;

	// Compute linear velocities
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);
	vel_buff = (stylusPosition * 3 - 4 * hdUtils.prvPos + hdUtils.lstPos) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.lvelocity = (.2196 * (vel_buff + hdUtils.vrLstInputVel) + .6588 * (hdUtils.prvInputVel + hdUtils.lstInputVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutVel + 2.5282 * hdUtils.lstOutVel - 0.7776 * hdUtils.vrLstOutVel); //cutoff freq of 20 Hz
	hdUtils.lstPos = hdUtils.prvPos;
	hdUtils.prvPos = stylusPosition;
	hdUtils.vrLstInputVel = hdUtils.lstInputVel;
	hdUtils.lstInputVel = hdUtils.prvInputVel;
	hdUtils.prvInputVel = vel_buff;
	hdUtils.vrLstOutVel = hdUtils.lstOutVel;
	hdUtils.lstOutVel = hdUtils.prvOutVel;
	hdUtils.prvOutVel = hdUtils.lvelocity;
	hdUtils.lvelocityTemp = hdUtils.lvelocityTemp + ((hdUtils.lvelocity - hdUtils.lvelocityTemp) * (0.001 / (0.001 + 0.07)));

	// Set linear velocities
	stylusLinearVelocity = hdUtils.lvelocityTemp;

	// Compute angular velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (stylusGimbalAngles * 3 - 4 * hdUtils.prvAng + hdUtils.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.avelocity = (.2196 * (vel_buff + hdUtils.vrLstInputAngVel) + .6588 * (hdUtils.prvInputAngVel + hdUtils.lstInputAngVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutAngVel + 2.5282 * hdUtils.lstOutAngVel - 0.7776 * hdUtils.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils.lstAng = hdUtils.prvAng;
	hdUtils.prvAng = stylusGimbalAngles;
	hdUtils.vrLstInputAngVel = hdUtils.lstInputAngVel;
	hdUtils.lstInputAngVel = hdUtils.prvInputAngVel;
	hdUtils.prvInputAngVel = vel_buff;
	hdUtils.vrLstOutAngVel = hdUtils.lstOutAngVel;
	hdUtils.lstOutAngVel = hdUtils.prvOutAngVel;
	hdUtils.prvOutAngVel = hdUtils.avelocity;
	hdUtils.avelocityTemp = hdUtils.avelocityTemp + ((hdUtils.avelocity - hdUtils.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	// Set angular velocities
	stylusAngularVelocity = hdUtils.avelocityTemp;

	// Update the Eigen variables in the member structure TeleopStatus
	geoTeleopState.hipVelocity.topRows(SPACE_DIM) = hdu2Eigen3D(stylusLinearVelocity).cast<float>();
	geoTeleopState.hipVelocity.bottomRows(SPACE_DIM) = hdu2Eigen3D(stylusAngularVelocity).cast<float>();
	geoTeleopState.hdUtils = hdUtils;

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
void catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger) {


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

hduVector3Dd eigen2hdu3D(const Eigen::Vector3d& eig_in) {

	hduVector3Dd out;
	for (int i = 0; i < 3; i++) {
		out[i] = eig_in(i);
	}

	return out;
}


Eigen::Vector3d hdu2Eigen3D(const hduVector3Dd& hdu_in) {

	Eigen::Vector3d out;
	for (int i = 0; i < 3; i++) {
		out(i) = hdu_in[i];
	}

	return out;

}