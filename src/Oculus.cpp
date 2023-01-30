#include "Oculus.hpp"
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>

const GLchar* VREP_VS =
"#version 450 core\n \
layout(location=0) in vec3 in_vertex;\n \
layout(location=1) in vec2 in_texCoord;\n \
uniform float hit; \n \
uniform uint isLeft; \n \
out vec2 b_coordTexture; \n \
void main()\n \
{\n \
	if (isLeft == 1U)\n \
	{\n \
		b_coordTexture = in_texCoord;\n \
		gl_Position = vec4(in_vertex.x - hit, in_vertex.y, in_vertex.z,1);\n \
	}\n \
	else \n \
	{\n \
		b_coordTexture = vec2(1.0 - in_texCoord.x, in_texCoord.y);\n \
		gl_Position = vec4(-in_vertex.x + hit, in_vertex.y, in_vertex.z,1);\n \
	}\n \
}";

const GLchar* VREP_FS =
"#version 450 core\n \
uniform sampler2D u_textureVREP; \n \
in vec2 b_coordTexture;\n \
out vec4 out_color; \n \
void main()\n \
{\n \
	out_color = vec4(texture(u_textureVREP, b_coordTexture).rgb,1); \n \
}";



void oculusLoop() {

	LARGE_INTEGER rate, tic, start_t;
	simxInt camera;
	simxInt eyes[2];
	double t_;

	// Get the clock rate
	QueryPerformanceFrequency(&rate);

	std::cout << "Starting Rift thread\n";

	int resolution[2] = { 320, 240 };
	int status = 1;

	Mat arr[2], arr_flip[2], img, left, right;
	simxUChar* eyeBuffers[2];
	eyeBuffers[0] = new simxUChar[resolution[0] * resolution[1] * sizeof(simxUChar)];
	eyeBuffers[1] = new simxUChar[resolution[0] * resolution[1] * sizeof(simxUChar)];

	simxGetVisionSensorImage(ecmClient, eyes[0], resolution, &eyeBuffers[0], 0, simx_opmode_streaming);
	simxGetVisionSensorImage(ecmClient, eyes[1], resolution, &eyeBuffers[1], 0, simx_opmode_streaming);

	SDL_Init(SDL_INIT_VIDEO);

	ovrResult result = ovr_Initialize(nullptr);
	if (OVR_FAILURE(result)) {
		std::cout << "ERROR: Failed to initialize libOVR" << std::endl;
	}

	ovrSession session;
	ovrGraphicsLuid luid;
	result = ovr_Create(&session, &luid);
	if (OVR_FAILURE(result)) {
		std::cout << "ERROR: Oculus Rift not detected" << std::endl;
		ovr_Shutdown();
		SDL_Quit();
	}

	int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;
	int winWidth = 1280;
	int winHeight = 720;

	Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
	SDL_Window* window;
	window = SDL_CreateWindow("Stereo Camera", x, y, winWidth, winHeight, flags);

	SDL_GLContext glContext = SDL_GL_CreateContext(window);
	glewInit();
	SDL_GL_SetSwapInterval(0);

	GLuint naoCameraTexture;
	glGenTextures(1, &naoCameraTexture);
	glBindTexture(GL_TEXTURE_2D, naoCameraTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, resolution[0], resolution[1], 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
	ovrSizei textureSize0 = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], 1.0f);
	ovrSizei textureSize1 = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], 1.0f);
	ovrSizei bufferSize;
	bufferSize.w = textureSize0.w + textureSize1.w;
	bufferSize.h = std::max(textureSize0.h, textureSize1.h);

	ovrTextureSwapChain textureChain = nullptr;
	ovrTextureSwapChainDesc descTextureSwap = ovrTextureSwapChainDesc();
	descTextureSwap.Type = ovrTexture_2D;
	descTextureSwap.ArraySize = 1;
	descTextureSwap.Width = bufferSize.w;
	descTextureSwap.Height = bufferSize.h;
	descTextureSwap.MipLevels = 1;
	descTextureSwap.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
	descTextureSwap.SampleCount = 1;
	descTextureSwap.StaticImage = ovrFalse;

	result = ovr_CreateTextureSwapChainGL(session, &descTextureSwap, &textureChain);

	int length = 0;
	ovr_GetTextureSwapChainLength(session, textureChain, &length);

	if (OVR_SUCCESS(result)) {
		for (int i = 0; i < length; ++i) {
			GLuint chainTexId;
			ovr_GetTextureSwapChainBufferGL(session, textureChain, i, &chainTexId);
			glBindTexture(GL_TEXTURE_2D, chainTexId);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		}
	}
	else {
		std::cout << "ERROR: failed creating swap texture" << std::endl;
		ovr_Destroy(session);
		ovr_Shutdown();
		SDL_GL_DeleteContext(glContext);
		SDL_DestroyWindow(window);
		SDL_Quit();
	}

	GLuint fboID;
	glGenFramebuffers(1, &fboID);

	GLuint depthBuffID;
	glGenTextures(1, &depthBuffID);
	glBindTexture(GL_TEXTURE_2D, depthBuffID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GLenum internalFormat = GL_DEPTH_COMPONENT24;
	GLenum type = GL_UNSIGNED_INT;
	glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, type, NULL);

	ovrMirrorTextureDesc descMirrorTexture;
	memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
	descMirrorTexture.Width = winWidth;
	descMirrorTexture.Height = winHeight;
	descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
	ovrMirrorTexture mirrorTexture = nullptr;
	result = ovr_CreateMirrorTextureGL(session, &descMirrorTexture, &mirrorTexture);
	if (!OVR_SUCCESS(result)) {
		std::cout << "ERROR: Failed to create mirror texture" << std::endl;
	}
	GLuint mirrorTextureId;
	ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &mirrorTextureId);

	GLuint mirrorFBOID;
	glGenFramebuffers(1, &mirrorFBOID);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
	glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTextureId, 0);
	glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	long long frameIndex = 0;
	ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);
	ovrPosef eyeRenderPose[2];
	ovrEyeRenderDesc eyeRenderDesc[2];
	eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
	eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);
	ovrVector3f hmdToEyeOffset[2];
	ovrPosef hmdToEyePose[2];
	double sensorSampleTime;

	Shader shader((GLchar*)VREP_VS, (GLchar*)VREP_FS);
	float vrepFovH = (float)(60.0 * M_PI) / 180;
	float ovrFovH = (atanf(hmdDesc.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc.DefaultEyeFov[0].RightTan));
	unsigned int usefulWidth = resolution[0] * ovrFovH / vrepFovH;
	unsigned int widthFinal = bufferSize.w / 2;
	float heightGL = 1.f;
	float widthGL = 1.f;

#ifndef FULLSCREEN
	if (usefulWidth > 0.f) {
		unsigned int heightFinal = resolution[1] * widthFinal / usefulWidth;
		// Convert this size to OpenGL viewport's frame's coordinates
		heightGL = (heightFinal) / (float)(bufferSize.h);
		widthGL = ((resolution[1] * (heightFinal / (float)resolution[1])) / (float)widthFinal);
	}
	else {
		std::cout << "wrong v-rep parameters" << std::endl;
	}
#endif

	float ovrFovV = (atanf(hmdDesc.DefaultEyeFov[0].UpTan) + atanf(hmdDesc.DefaultEyeFov[0].DownTan));
	float offsetLensCenterX = ((atanf(hmdDesc.DefaultEyeFov[0].LeftTan)) / ovrFovH) * 2.f - 1.f;
	float offsetLensCenterY = ((atanf(hmdDesc.DefaultEyeFov[0].UpTan)) / ovrFovV) * 2.f - 1.f;

	struct GLScreenCoordinates {
		float left, up, right, down;
	} screenCoord;
	screenCoord.up = heightGL + offsetLensCenterY;
	screenCoord.down = heightGL - offsetLensCenterY;
	screenCoord.right = widthGL + offsetLensCenterX;
	screenCoord.left = widthGL - offsetLensCenterX;

	float rectVertices[12] = { -screenCoord.left, -screenCoord.up, 0,
		screenCoord.right, -screenCoord.up, 0,
		screenCoord.right, screenCoord.down, 0,
		-screenCoord.left, screenCoord.down, 0 };
	GLuint rectVBO[3];
	glGenBuffers(1, &rectVBO[0]);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertices), rectVertices, GL_STATIC_DRAW);

	float rectTexCoord[8] = { 0, 1, 1, 1, 1, 0, 0, 0 };
	glGenBuffers(1, &rectVBO[1]);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectTexCoord), rectTexCoord, GL_STATIC_DRAW);

	unsigned int rectIndices[6] = { 0, 1, 2, 0, 2, 3 };
	glGenBuffers(1, &rectVBO[2]);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectIndices), rectIndices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	float hit = 0.02f;
	SDL_Event events;

	unsigned int riftc = 0, zedc = 1;
	unsigned int rifttime = 0;
	int time1 = 0, timePerFrame = 0;
	int frameRate = (int)(1000 / MAX_FPS);

	glUseProgram(shader.getProgramId());
	glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
	glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
	glEnableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
	glVertexAttribPointer(Shader::ATTRIB_TEXTURE2D_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);

	std::cout << "Starting Rift main loop\n";

	//float rift[3] = { 0, 0, 0 };
	//float cam_zero[3] = { 0, 0, 0 };

	//int n = 0;
	//float* mat = new float[16]();

	ovr_RecenterTrackingOrigin(session);

	//while (true) {
	//	while (simxGetObjectOrientation(ecmClient, camera, -1, cam_zero, simx_opmode_blocking));
	//	int ret = 1;
	//	for (int i = 0; i < 3; i++) if (isnan(cam_zero[i]) || fabsf(cam_zero[i]) > M_PI) ret = 0;
	//	if (ret) break;
	//}

	////---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//simxInt J1_ECM, J2_ECM, J3_ECM, J4_ECM;

	//simxGetObjectHandle(ecmClient, "J1_ECM", &J1_ECM, simx_opmode_blocking);
	//simxGetObjectHandle(ecmClient, "J2_ECM", &J2_ECM, simx_opmode_blocking);
	//simxGetObjectHandle(ecmClient, "J3_ECM", &J3_ECM, simx_opmode_blocking);
	//simxGetObjectHandle(ecmClient, "J4_ECM", &J4_ECM, simx_opmode_blocking);

	//std::cout << "J1_ECM handler: " << J1_ECM << std::endl;
	//std::cout << "J2_ECM handler: " << J2_ECM << std::endl;
	//std::cout << "J3_ECM handler: " << J3_ECM << std::endl;
	//std::cout << "J4_ECM handler: " << J4_ECM << std::endl;

	//Eigen::Matrix<float, 3, 4> jacobian;
	//Eigen::Matrix<float, 3, 3> rRt;
	//Eigen::Matrix<float, 3, 3> bRe;
	//Eigen::Matrix3f eRr, Rflip;
	//Eigen::Matrix<float, 4, 4> DirKin;
	//Eigen::Matrix<float, 3, 1> p;
	//Eigen::Matrix<float, 3, 1> p_vel, wrr, wee;
	//Eigen::Matrix<float, 3, 1> p_vel_prec;
	//Quat<float> quat;
	//Eigen::Vector4f qdot;
	//Eigen::Quaternionf quaternion;
	////std::vector<simxFloat> q(4, 1);
	//Eigen::Vector4f q, qcmd;
	//float scale = 5.0;

	//double p_prec[3] = { 0, 0, 0 };
	//double p_after[3];
	//double p_dot[3];
	//double z = 0;
	//p_vel_prec.setZero();
	////auto start = std::chrono::system_clock::now();

	//simxFloat data;

	//// Enable streaming
	//simxGetJointPosition(ecmClient, J1_ECM, &q(0), simx_opmode_streaming);
	//simxGetJointPosition(ecmClient, J2_ECM, &q(1), simx_opmode_streaming);
	//simxGetJointPosition(ecmClient, J3_ECM, &q(2), simx_opmode_streaming);
	//simxGetJointPosition(ecmClient, J4_ECM, &q(3), simx_opmode_streaming);//*/

	//// Init some data: here, we use a blocking call to be sure to read the proper values
	//simxGetJointPosition(ecmClient, J1_ECM, &q(0), simx_opmode_blocking);
	//simxGetJointPosition(ecmClient, J2_ECM, &q(1), simx_opmode_blocking);
	//simxGetJointPosition(ecmClient, J3_ECM, &q(2), simx_opmode_blocking);
	//simxGetJointPosition(ecmClient, J4_ECM, &q(3), simx_opmode_blocking);//*/
	//qcmd = q;

	// Initialize time
	QueryPerformanceCounter(&start_t);

	while (running) {

		QueryPerformanceCounter(&tic);

		///*simxGetJointPosition(clientID, J1_ECM, &q[0], simx_opmode_blocking);
		//simxGetJointPosition(clientID, J2_ECM, &q[1], simx_opmode_blocking);
		//simxGetJointPosition(clientID, J3_ECM, &q[2], simx_opmode_blocking);
		//simxGetJointPosition(clientID, J4_ECM, &q[3], simx_opmode_blocking);//*/

		//// Get data from the buffer
		//simxGetJointPosition(ecmClient, J1_ECM, &q(0), simx_opmode_buffer);
		//simxGetJointPosition(ecmClient, J2_ECM, &q(1), simx_opmode_buffer);
		//simxGetJointPosition(ecmClient, J3_ECM, &q(2), simx_opmode_buffer);
		//simxGetJointPosition(ecmClient, J4_ECM, &q(3), simx_opmode_buffer);//*/

		//Posef pose = ovr_GetTrackingState(session, 0, true).HeadPose.ThePose;
		//pose.Rotation.GetEulerAngles<Axis_X, Axis_Z, Axis_Y>(&rift[0], &rift[1], &rift[2]);
		//ovrVector3f angVel = ovr_GetTrackingState(session, 0, true).HeadPose.AngularVelocity;
		//ovrVector3f linVel = ovr_GetTrackingState(session, 0, true).HeadPose.LinearVelocity;


		//if (GetKeyState(VK_SPACE)) {

		//	quat = pose.Rotation;
		//	quaternion.x() = quat.x;
		//	quaternion.y() = quat.y;
		//	quaternion.z() = quat.z;
		//	quaternion.w() = quat.w;


		//	rRt = quaternion.toRotationMatrix();


		//	/*
		//	auto end = std::chrono::system_clock::now();
		//	std::chrono::duration<double> diff = end - start;
		//	start = end;
		//	auto sampling_time = diff.count();
		//	if (sampling_time == 0)		continue;
		//	*/

		//	if (!GetKeyState(VK_DOWN)) { //Orientation

		//		std::cout << "ORIENTATION CONTROL ENABLED." << std::endl;


		//		p(0, 0) = cam_zero[0] - rift[0];
		//		p(1, 0) = cam_zero[1] + rift[1];
		//		p(2, 0) = cam_zero[2] + rift[2];

		//		p_after[0] = p(0, 0);
		//		p_after[1] = p(1, 0);
		//		p_after[2] = p(2, 0);

		//		p_dot[0] = (p_after[0] - p_prec[0]) / Ts; // --> sampling_time
		//		p_dot[1] = (p_after[1] - p_prec[1]) / Ts;
		//		p_dot[2] = (p_after[2] - p_prec[2]) / Ts;

		//		data = p_dot[0];
		//		//simxSetFloatSignal(clientID, "before", data, simx_opmode_oneshot);

		//		p_vel[0] = lowPassFrequency(p_dot[0], p_vel_prec[0]);
		//		p_vel[1] = lowPassFrequency(p_dot[1], p_vel_prec[1]);
		//		p_vel[2] = lowPassFrequency(p_dot[2], p_vel_prec[2]);

		//		data = p_vel[0];
		//		//simxSetFloatSignal(clientID, "after", data, simx_opmode_oneshot);

		//		p_vel_prec = p_vel;

		//		p_prec[0] = p_after[0];
		//		p_prec[1] = p_after[1];
		//		p_prec[2] = p_after[2];//*/

		//		p_vel(0) = angVel.x;
		//		p_vel(1) = angVel.y;
		//		p_vel(2) = angVel.z;//*/

		//		wrr = rRt.transpose() * p_vel;

		//		// Match the rift orientation with camera frame
		//		eRr << 1, 0, 0,
		//			0, -1, 0,
		//			0, 0, -1;

		//		Rflip << -1, 0, 0,
		//			0, -1, 0,
		//			0, 0, 1;

		//		wee = scale * (Rflip * eRr * wrr);
		//		//wee = scale *  wrr;
		//		//wee(0) *= -1;
		//		//wee(1) *= -1;

		//		// Send data to V-REP
		//		simxSetFloatSignal(ecmClient, "ecm_wx", wee(0), simx_opmode_oneshot);
		//		simxSetFloatSignal(ecmClient, "ecm_wy", wee(1), simx_opmode_oneshot);
		//		simxSetFloatSignal(ecmClient, "ecm_wz", wee(2), simx_opmode_oneshot);

		//		std::cout << "angVel: " << wee.transpose() << std::endl;

		//		// Compute direct and differential kinematics of the ECM
		//		DirKin = forwardKinematicsECM(q);
		//		bRe = DirKin.topLeftCorner(3, 3);
		//		jacobian = bRe.transpose() * ECM_Jacobian(q);

		//		// Compute the Joint velocity commands
		//		qdot = jacobian.fullPivLu().solve(wee); // J^-1 * p
		//		qdot(2) = 0;

		//		// Send joint velocity command data to CoppeliaSim
		//		simxSetFloatSignal(ecmClient, "qdotECM_1", qdot(0), simx_opmode_oneshot);
		//		simxSetFloatSignal(ecmClient, "qdotECM_2", qdot(1), simx_opmode_oneshot);
		//		simxSetFloatSignal(ecmClient, "qdotECM_3", qdot(2), simx_opmode_oneshot);
		//		simxSetFloatSignal(ecmClient, "qdotECM_4", qdot(3), simx_opmode_oneshot);

		//		/*qcmd[0] += (qdot(0, 0) * Ts); // --> sampling_time
		//		qcmd[1] += (qdot(1, 0) * Ts);
		//		qcmd[3] += (qdot(3, 0) * Ts);//*/

		//	}
		//	else { //Zoom

		//		std::cout << "POSITION CONTROL ENABLED." << std::endl;
		//		//z = -pose.Translation.z;
		//		Eigen::Vector3f eigen_lin_vel;

		//		eigen_lin_vel[0] = linVel.x;
		//		eigen_lin_vel[1] = linVel.y;
		//		eigen_lin_vel[2] = linVel.z;

		//		eigen_lin_vel = rRt * eigen_lin_vel;

		//		qdot(0) = 0;
		//		qdot(1) = 0;
		//		qdot(2) = -eigen_lin_vel[2];
		//		qdot(3) = 0;


		//		//qcmd[2] += - (linVel.z * Ts); // --> sampling_time
		//		//simxSetFloatSignal(clientID, "z", qcmd[2], simx_opmode_oneshot);

		//	}


			// AVOID BLOCKING-CALLS!
			/*simxSetJointPosition(clientID, J1_ECM, qcmd[0], simx_opmode_blocking);
			simxSetJointPosition(clientID, J2_ECM, qcmd[1], simx_opmode_blocking);
			simxSetJointPosition(clientID, J3_ECM, z, simx_opmode_blocking);
			simxSetJointPosition(clientID, J4_ECM, qcmd[3], simx_opmode_blocking);//*/

			/*simxSetJointPosition(clientID, J1_ECM, qcmd[0], simx_opmode_oneshot);
			simxSetJointPosition(clientID, J2_ECM, qcmd[1], simx_opmode_oneshot);
			simxSetJointPosition(clientID, J3_ECM, qcmd[2], simx_opmode_oneshot);
			simxSetJointPosition(clientID, J4_ECM, qcmd[3], simx_opmode_oneshot);//*/

			//std::cout << "q: " << q[0] << std::endl;
			/*
			q[0] = temp(0, 0);
			q[1] = temp(1, 0);
			q[2] = temp(2, 0);
			q[3] = temp(3, 0);*/
		/*}
		else {

			qdot.setZero();

		}

		simxSetFloatSignal(ecmClient, "qdotECM_1", qdot(0), simx_opmode_oneshot);
		simxSetFloatSignal(ecmClient, "qdotECM_2", qdot(1), simx_opmode_oneshot);
		simxSetFloatSignal(ecmClient, "qdotECM_3", qdot(2), simx_opmode_oneshot);
		simxSetFloatSignal(ecmClient, "qdotECM_4", qdot(3), simx_opmode_oneshot);*/


		t_ = (tic.QuadPart - start_t.QuadPart) / float(rate.QuadPart);

		// Log data
		/*qdotECMSS << t_ << ", " << qdot(0) << ", " << qdot(1) << ", " << qdot(2) << ", " << qdot(3) << "; " << std::endl;
		qECMSS << t_ << ", " << q(0) << ", " << q(1) << ", " << q(2) << ", " << q(3) << "; " << std::endl;
		vtrSS << t_ << ", " << linVel.x << ", " << linVel.y << ", " << linVel.z << ", " << angVel.x << ", " << angVel.y << ", " << angVel.z << "; " << std::endl;*/
		//vrrSS << t_ << ", " << linVel.x << ", " << linVel.y << ", " << linVel.z << ", " << wrr(0) << ", " << wrr(1) << ", " << wrr(2) << "; " << std::endl;

		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


		timePerFrame = SDL_GetTicks() - time1;

		if (timePerFrame < frameRate) {
			SDL_Delay(frameRate - timePerFrame);
			timePerFrame = frameRate;
		}

		rifttime += timePerFrame;
		riftc++;
		if (rifttime > 200) {
			rifttime = 0;
			riftc = 0;
		}
		time1 = SDL_GetTicks();

		while (SDL_PollEvent(&events)) {
			if (events.type == SDL_KEYUP) {
				if (events.key.keysym.scancode == SDL_SCANCODE_R)
					hit = 0.0f;
			}
			if (events.type == SDL_MOUSEWHEEL) {
				float s;
				events.wheel.y > 0 ? s = 1.0f : s = -1.0f;
				hit += 0.005f * s;
			}
		}

		GLuint curTexId;
		int curIndex;
		ovr_GetTextureSwapChainCurrentIndex(session, textureChain, &curIndex);
		ovr_GetTextureSwapChainBufferGL(session, textureChain, curIndex, &curTexId);

		eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
		eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);
		hmdToEyePose[0] = eyeRenderDesc[0].HmdToEyePose;
		hmdToEyePose[1] = eyeRenderDesc[1].HmdToEyePose;
		//hmdToEyeOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
		//hmdToEyeOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;
		ovr_GetEyePoses(session, frameIndex, ovrTrue, hmdToEyePose, eyeRenderPose, &sensorSampleTime);

		glBindFramebuffer(GL_FRAMEBUFFER, fboID);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0, 0, 0, 1);

		for (int i = 0; i < 2; i++) {
			simxGetVisionSensorImage(ecmClient, eyes[i], resolution, &eyeBuffers[i], 0, simx_opmode_buffer);
			try {
				arr[i] = Mat(resolution[1], resolution[0], CV_8UC3, eyeBuffers[i]);
				flip(arr[i], arr_flip[i], 1);
			}
			catch (cv::Exception) {
				continue;
			}

			glViewport(i == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
			glBindTexture(GL_TEXTURE_2D, naoCameraTexture);

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, resolution[0], resolution[1], 0, GL_RGB, GL_UNSIGNED_BYTE, arr_flip[i].ptr());

			glUniform1f(glGetUniformLocation(shader.getProgramId(), "hit"), i == ovrEye_Left ? hit : -hit);
			glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"), i == ovrEye_Left ? 1U : 0U);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, fboID);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
		ovr_CommitTextureSwapChain(session, textureChain);

		//frameIndex++;

		ovrLayerEyeFov ld;
		ld.Header.Type = ovrLayerType_EyeFov;
		ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;
		for (int eyes = 0; eyes < 2; ++eyes) {
			ld.ColorTexture[eyes] = textureChain;
			ld.Viewport[eyes] = OVR::Recti(eyes == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
			ld.Fov[eyes] = hmdDesc.DefaultEyeFov[eyes];
			ld.RenderPose[eyes] = eyeRenderPose[eyes];
		}

		ovrLayerHeader* layers = &ld.Header;

		std::cout << "session = " << session << std::endl;

		result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);


		if (!OVR_SUCCESS(result)) {
			std::cout << "ERROR: failed to submit frame" << std::endl;
			ovr_DestroyTextureSwapChain(session, textureChain);
			ovr_DestroyMirrorTexture(session, mirrorTexture);
			ovr_Destroy(session);
			ovr_Shutdown();
			SDL_GL_DeleteContext(glContext);
			SDL_DestroyWindow(window);
			SDL_Quit();
		}

		glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		GLint w = winWidth;
		GLint h = winHeight;
		glBlitFramebuffer(0, h, w, 0,
			0, 0, w, h,
			GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		SDL_GL_SwapWindow(window);




	}

	/*simxSetFloatSignal(ecmClient, "qdotECM_1", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(ecmClient, "qdotECM_2", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(ecmClient, "qdotECM_3", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(ecmClient, "qdotECM_4", 0.0, simx_opmode_oneshot);*/

	std::cout << "Ending Rift main loop\n";

	glDisableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
	glDisableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glUseProgram(0);
	glBindVertexArray(0);
	glDeleteBuffers(3, rectVBO);
	ovr_DestroyTextureSwapChain(session, textureChain);
	ovr_DestroyMirrorTexture(session, mirrorTexture);
	ovr_Destroy(session);
	ovr_Shutdown();
	SDL_GL_DeleteContext(glContext);
	SDL_DestroyWindow(window);
	SDL_Quit();

	std::cout << "Ending Rift thread\n";
}
