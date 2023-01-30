#include "vs_header.hpp"
#include <iostream>
#include <fstream>
#include <direct.h>
void vs_loop() {
	std::string plot_path;
	char curDirAndFile[1024];
	_getcwd(curDirAndFile, sizeof(curDirAndFile));
	plot_path = std::string(curDirAndFile);
	plot_path += "/plot/";
	std::ofstream x_file, y_file, depth_file,j1, j2, j3, j4;
	std::string x_fileName = plot_path + "feature_x.txt";
	std::string y_fileName = plot_path + "feature_y.txt";
	std::string depth_fileName = plot_path + "min_depth.txt";
	std::string j1_file = plot_path + "j1.txt";
	std::string j2_file = plot_path + "j2.txt";
	std::string j3_file = plot_path + "j3.txt";
	std::string j4_file = plot_path + "j4.txt";
	x_file.open(x_fileName, std::ios::app);
	y_file.open(y_fileName, std::ios::app);
	depth_file.open(depth_fileName, std::ios::app);
	j1.open(j1_file, std::ios::app);
	j2.open(j2_file, std::ios::app);
	j3.open(j3_file, std::ios::app);
	j4.open(j4_file, std::ios::app);
	//camera (normalized) focal length, handles and relative position
	float lambda = 0.0f;
	int eye_L = 0, eye_R = 0;
	float cam_pos[3] = {};

	//variables to communicate with coppeliasim
	simxFloat* auxValues_L;
	simxInt* auxValuesCount_L;
	simxFloat* auxValues_R;
	simxInt* auxValuesCount_R;

	//ecm joint handles, position and velocity
	Vector4f q, dq;
	float dq_prism_L = 0.0f, dq_prism_R = 0.0f;
	Vector4i ecmJointHandles;

	//position of the 4 blobs in the 2 vision sensors. mean_L is the mean in the left image plane, same for mean_R
	Vector2f p1_L, p2_L, p1_R, p2_R;
	Vector2f mean_L, mean_R;

	//p is the feature point (will be defined later), p_ref the feature reference point
	Vector2f p;
	const Vector2f p_ref(0.5, 0.5);

	//depth_threshold is the distance between the cameras and the closest marker under which the ecm should not go
	//we also specify a tolerance to relax the bound
	const float depth_threshold = 0.02f;
	float tolerance_depth = 0.005f;
	float min_depth = 0.0f; //depth of the closest marker
	float depth = 0.08f, depth_1 = 0.0f, depth_2 = 0.0f; //respectively depth of the mean of the two markers in space, and of each of them.
	float md_L = 0.0f, md_R = 0.0f; //mean distance of the blobs in the respective image plane. defined later
	const float k_prism = 0.8f, k_rev = 0.8f; //control gains
	const char mode = 'v';

	/*safety_threshold is helpful when in one camera there is only one marker but with 2 bounding boxes around it: the distance
	* between these boxes would be very small, hence with a high velocity the prismatic joint of the ecm would get close to them.
	* To avoid this, along with an erroneous control of the revolute joints, we set this threshold.
	*/
	const float safety_threshold = 0.02f;
	//variable to handle the case of only 1 blob in a vision sensor. It still is a mean distance as md_L, md_R.
	float  md = 0.0f;

	//set the joint, camera handles and set the focal length (normalized)
	set_handles(ecmJointHandles, eye_L, eye_R);
	lambda = retrieve_cam_params(eye_L, eye_R, cam_pos);

	//start streaming
	for (int i = 0; i < 4; ++i)
		simxGetJointPosition(vsClient, ecmJointHandles(i), &q(i), simx_opmode_streaming);
	simxReadVisionSensor(vsClient, eye_L, nullptr, &auxValues_L, &auxValuesCount_L, simx_opmode_streaming);
	simxReadVisionSensor(vsClient, eye_R, nullptr, &auxValues_R, &auxValuesCount_R, simx_opmode_streaming);
	std::cout << "Starting vs loop" << std::endl;
	
	while (running) {
		simxReadVisionSensor(vsClient, eye_L, nullptr, &auxValues_L, &auxValuesCount_L, simx_opmode_buffer);
		simxReadVisionSensor(vsClient, eye_R, nullptr, &auxValues_R, &auxValuesCount_R, simx_opmode_buffer);

		for (int i = 0; i < 4; ++i)
			simxGetJointPosition(vsClient, ecmJointHandles(i), &q(i), simx_opmode_buffer);

		if (auxValues_L != nullptr && auxValuesCount_L != nullptr && auxValues_R != nullptr && auxValuesCount_R != nullptr) {
			//auxValuesCount[1] is the size of pck 1, the standard one, so 
			//auxValues[auxValuesCount[1]] is the first element of packet 2, that is
			//the number of blobs
			if (auxValues_L[auxValuesCount_L[1]] > 1 && auxValues_R[auxValuesCount_R[1]] > 1) { //if there are 2 blobs in both cameras
				get_blobs(auxValues_L, auxValuesCount_L, p1_L, p2_L);
				mean_L << 0.5f * (p1_L + p2_L);

				get_blobs(auxValues_R, auxValuesCount_R, p1_R, p2_R);
				mean_R << 0.5f * (p1_R + p2_R);

				//triangulation to get the depth of the mean point of the markers, and then of the markers
				depth = abs(cam_pos[0]) * lambda / (mean_L(0) - mean_R(0));

				depth_1 = abs(cam_pos[0]) * lambda / (p1_L(0) - p1_R(0));
				depth_2 = abs(cam_pos[0]) * lambda / (p2_L(0) - p2_R(0));

				//this is our feature point, the mean of the means in each image plane. This point will be kept at the center
				p << 0.5 * (mean_L + mean_R);
				x_file << p(0) << std::endl;
				y_file << p(1) << std::endl;
				/*md is the mean distance of the points from their mean. In case of 2 points
				* it is simplified to (mean_L - p1_L).norm() where p1_L can be substituted
				* with p2_L (same for the right side). It is used as an indicator of closeness between the markers.
				*/
				md_L = 0.5f * ((mean_L - p1_L).norm() + (mean_L - p2_L).norm());
				md_R = 0.5f * ((mean_R - p1_R).norm() + (mean_R - p2_R).norm());

				if (md_L <= safety_threshold || md_R <= safety_threshold) {
					dq.setZero();
					dq(2) = -0.05f;
				}
				else {
					dq = compute_vel3R(q, p_ref, p, depth, k_rev, lambda);
					//compute the desired prismatic joint velocity considering independently each vision sensor
					dq_prism_L = compute_vel1P(p1_L, p2_L, md_L, k_prism);
					dq_prism_R = compute_vel1P(p1_R, p2_R, md_R, k_prism);
					dq(2) = (abs(dq_prism_L) > abs(dq_prism_R)) ? dq_prism_L : dq_prism_R; //then take the argmax of their absolute value.
				}


				//min_depth = std::min(depth_1, depth_2);
				//depth_file << min_depth << std::endl;
				////if the closest marker to the ECM is closer than a depth_threshold (considering also a tolerance_depth)
				////the prismatic joint is commanded to retract
				//if (min_depth < (depth_threshold - tolerance_depth)) {
				//	dq(2) = 30*tolerance_depth * (min_depth - depth_threshold);
				//}
				////instead, if (depth_threshold - tolerance_depth) < min_depth < depth_threshold
				////and the desired prismatic velocity is positive, i.e. we would like to get even closer,
				////the commanded velocity is overwritten and set to 0
				//else if (dq(2)>0 && min_depth > (depth_threshold - tolerance_depth) && min_depth <= depth_threshold) {
				//	dq(2) = 0.0f;
				//}

				j1 << dq(0) << std::endl;
				j2 << dq(1) << std::endl;
				j3 << dq(2) << std::endl;
				j4 << dq(3) << std::endl;

				//send the velocities
				for (int i = 0; i < 4; ++i)
					if (mode == 'v')
						simxSetJointTargetVelocity(vsClient, ecmJointHandles(i), dq(i), simx_opmode_oneshot);
					else
						simxSetJointPosition(vsClient, ecmJointHandles(i), q(i) + Ts * dq(i), simx_opmode_oneshot);
			}
			else { //at least one of the two vision sensors has detected only one blob
				if (auxValues_L[auxValuesCount_L[1]] > 1) {
					get_blobs(auxValues_L, auxValuesCount_L, p1_L, p2_L);
					p << 0.5f * (p1_L + p2_L);
					md = 0.5f * ((mean_L - p1_L).norm() + (mean_L - p2_L).norm());
				}
				else if (auxValues_R[auxValuesCount_R[1]] > 1) {
					get_blobs(auxValues_R, auxValuesCount_R, p1_R, p2_R);
					p << 0.5f * (p1_R + p2_R);
					md = 0.5f * ((mean_R - p1_R).norm() + (mean_R - p2_R).norm());
				}
				else { //if both of them have detected less than 2 blobs, then the revolute joints stop and the prismatic one retracts
					std::cout << "Less than 2 blobs detected" << std::endl;
					simxSetJointTargetVelocity(vsClient, ecmJointHandles(0), 0, simx_opmode_oneshot);
					simxSetJointTargetVelocity(vsClient, ecmJointHandles(1), 0, simx_opmode_oneshot);
					simxSetJointTargetVelocity(vsClient, ecmJointHandles(2), -0.05f, simx_opmode_oneshot);
					simxSetJointTargetVelocity(vsClient, ecmJointHandles(3), 0, simx_opmode_oneshot);
					continue;
				}
				/*if one of the two vision sensor still sees two blobs, we still can do visual servoing.
				* We do the same check on the safety threshold and move the revolute joints as in the nominal case,
				* using as depth the one from the previous iteration. The prismatic joint is commanded to simply retract. (it should already be at 0, but still)
				*/
				std::cout << "Only one vision sensor available for visual servoing" << std::endl;
				if (md <= safety_threshold)
					dq.setZero();
				else
					dq = compute_vel3R(q, p_ref, p, depth, k_rev, lambda);
				dq(2) = -0.05f;

				for (int i = 0; i < 4; ++i)
					simxSetJointTargetVelocity(vsClient, ecmJointHandles(i), dq(i), simx_opmode_oneshot);
			}
		}
		else
			std::cout << "Waiting for data ..." << std::endl;
	}
	x_file.close();
	y_file.close();
	depth_file.close();
	j1.close();
	j2.close();
	j3.close();
	j4.close();
	
}
void set_handles(Vector4i& ecmJointHandles, int& eye_L, int& eye_R) {

	const simxChar* joint_names[] = { "/J1_ECM" ,"/J2_ECM" ,"/J3_ECM" ,"/J4_ECM" };
	for (int i = 0; i < 4; i++)
		simxGetObjectHandle(vsClient, joint_names[i], &ecmJointHandles(i), simx_opmode_blocking);

	simxGetObjectHandle(vsClient, "/Vision_sensor_left", &eye_L, simx_opmode_blocking);
	simxGetObjectHandle(vsClient, "/Vision_sensor_right", &eye_R, simx_opmode_blocking);
}
Vector4f compute_vel3R(const Vector4f q, const Vector2f p_ref, const Vector2f p, const float& Z, const float& k_rev, const float& lambda) {
	//Jp point feature interaction matrix
	const Eigen::Matrix<float, 2, 6> Jp = get_interaction_matrix(p, Z, lambda);

	//homogeneous and rotation matrix from the base frame to the e.e.
	const Matrix4f T = forwardKinematicsECM(q);
	const Matrix3f R = T.block(0, 0, 3, 3);

	//express jacobian in the camera frame (same as e.e.). The jacobian is 3x4 
	//because it is only the angular part of the geometric.
	
	const Eigen::Matrix<float, 3, 4> J = R.transpose() * ECM_Jacobian(q);

	//only the last 3 columns for the angular movement. 
	const Eigen::Matrix<float, 2, 3> Jp_reduced = Jp.block(0, 3, 2, 3);
	
	const Vector2f e = compute_features_error(p_ref, p);
	const Vector4f dq = (Jp_reduced * J).fullPivLu().solve(k_rev * e); //pinv(J)*k*e
	
	return dq;
}

float compute_vel1P(const Vector2f p1, const Vector2f p2, const float& md, const float& k_prismatic) {
	const float zoom_in_threshold = 0.15f, zoom_out_threshold = 0.35f;
	float dq = 0.0f;
	const Vector2f center(0.5, 0.5);
	if (md >= zoom_in_threshold) {
		float max_dist = std::max((p1 - center).norm(), (p2 - center).norm());
		if (max_dist >= zoom_out_threshold)
			dq = k_prismatic * (zoom_out_threshold - max_dist);
		else
			dq = 0.0f;
	}
	else {
		dq = k_prismatic * (zoom_in_threshold - md);
	}
	return dq;
}

Eigen::Matrix<float, 2, 6> get_interaction_matrix(const Vector2f p, const float& Z, const float& lambda) {
	const float uc = 0.5f;
	const float vc = 0.5f;
	const float u = p(0);
	const float v = p(1);
	Eigen::Matrix<float, 2, 6> Jp;
	Jp << lambda / Z, 0.0f, (u - uc) / Z, (u - uc)* (v - vc) / lambda, lambda + pow(u - uc, 2) / lambda, -(v - vc),
		0.0f, -lambda / Z, -(v - vc) / Z, lambda - pow(v - vc, 2) / lambda, -(u - uc) * (v - vc) / lambda, (u - uc);
	return Jp;
}
inline Vector2f compute_features_error(const Vector2f p_ref, const Vector2f p) {
	return p_ref - p;
}
void get_blobs(const simxFloat* auxValues, const simxInt* auxValuesCount, Vector2f& p1, Vector2f& p2) {
	const int idx_x = auxValuesCount[1] + 4;
	const int idx_y = idx_x + 1;
	const int blob_sz = auxValues[auxValuesCount[1] + 1];
	const int num_blobs = auxValues[auxValuesCount[1]];
	p1 << auxValues[idx_x], auxValues[idx_y];
	p2 << auxValues[idx_x + blob_sz], auxValues[idx_y + blob_sz];
	if (num_blobs == 2)
		return;

	//if there are more than 2 blobs, get the 2 furthest ones
	float max = (p1 - p2).norm();
	float m_t = 0.0f;
	Vector2f temp;
	for (int i = 2; i < num_blobs; i++) {
		temp << auxValues[idx_x + i * blob_sz], auxValues[idx_y + i * blob_sz];
		m_t = (p1 - temp).norm();
		if (m_t > max) {
			p2 << temp;
			max = m_t;
		}
	}
	max = 0.0f;
	for (int i = 0; i < num_blobs; i++) {
		temp << auxValues[idx_x + i * blob_sz], auxValues[idx_y + i * blob_sz];
		m_t = (p2 - temp).norm();
		if (m_t > max) {
			p1 << temp;
			max = m_t;
		}
	}
	//p1 should always be the leftmost blob, p2 the rightmost. If they have the same x (impossible, they are floats), same resoning on y
	if (p1(0) > p2(0)) {
		temp << p2;
		p2 << p1;
		p1 << temp;
	}
	else if (p1(0) == p2(0) && p1(1) > p2(1)) {
		temp << p2;
		p2 << p1;
		p1 << temp;
	}
	return;
}

float retrieve_cam_params(const int& eye_L, const int& eye_R, float* cam_pos) {
	float perspAngle = 0.0f, lambda = 0.0f;
	simxGetObjectFloatParameter(vsClient, eye_L, 1004, &perspAngle, simx_opmode_streaming);
	simxGetObjectPosition(vsClient, eye_R, eye_L, cam_pos, simx_opmode_streaming);
	do {
		simxGetObjectFloatParameter(vsClient, eye_L, 1004, &perspAngle, simx_opmode_buffer);
		simxGetObjectPosition(vsClient, eye_R, eye_L, cam_pos, simx_opmode_buffer);

		lambda = 0.5f / tan(perspAngle / 2.0f);
	} while (!(perspAngle > 0.0f));

	return lambda;
}