// Project Header files
#include "utils.hpp"

/**
* @brief Load function
* Load and extract data from the input log
* @param file: the log file
* @return a structure containing the loaded data
*/
std::vector < Eigen::VectorXf > loadCSVLog(const std::string& file) {

	std::vector < Eigen::VectorXf > out;
	std::vector < std::string > content;

	content = readCSVFile(file.c_str());
	for (int i = 0; i < content.size(); i++) {
		std::vector < double > vec = parseCSVLine(content[i]);
		Eigen::VectorXf q_j(vec.size());
		for (int j = 0; j < vec.size(); j++) {
			q_j(j) = (float)vec[j];
		}
		out.push_back(q_j);
	}

	return out;
}


/**
* @brief Read function
* Read the conent of the file, assuming it's writte in the CSV format
* @ return a vector of string containing the set of values of the read CSV structure
*/
std::vector < std::string > readCSVFile(const char* filename) {

	std::vector < std::string > content;
	std::ifstream inFile;
	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;


	// Open the file
	inFile.open(filename, std::ios_base::in);

#ifdef DEBUG
	//std::cout << "File path: " << this->file << std::endl;
#endif //DEBUG

	if (!inFile) { // if cannot open the file
		std::cout << "Error in accessing the file " << filename << ". Could not read the content." << std::endl;
		std::string error("ERROR");
		content.push_back(error);
	}
	else {

		while (!inFile.eof()) {

			// Extract the current line
			std::getline(inFile, line);

			if (line.length() > 0) { // if not empty line ...
				if (line.find("###") == std::string::npos) { // not comment line
					content.push_back(line);
				}
			}
		}
	}

	// Return content
	return content;
}


/**
* @brief Utility function
* parse the input string line into an array of doubled-precision floating values
* @param the input string line
* @return the corresponding vector of double-precision floating values
*/
std::vector < double > parseCSVLine(const std::string& line) {

	std::vector < double > vec;
	std::string comma(",");
	std::string semicolumn(";");

	std::string temp_line = line;
	bool endloop = false;
	while (!endloop && temp_line.length() > 0) {

		// Find the comma
		int delimiter = temp_line.find(comma);

		// If not found, look for the semicolumn
		if (delimiter == std::string::npos) {
			delimiter = temp_line.find(semicolumn);
			endloop = true;
		}

		// Assign the j-th value
		vec.push_back(std::stod(temp_line.substr(0, delimiter)));

		// Update the line
		temp_line = temp_line.substr(delimiter + 1, temp_line.length());
	}
	return vec;
}

Eigen::VectorXf filter_velocity(Eigen::VectorXf velocity, Eigen::Ref<Eigen::VectorXf> velocity_state) {
	Eigen::VectorXf y, X_d;
	y = velocity_state * 0.7741 + 0.5363 * velocity;
	velocity_state = 0.1653 * velocity_state + 0.5 * velocity;
	return y;

	//    note on Discrete Filter: [t=0.01]                 225                     0.6024 z + 0.2922
	//                                            Tf =   ---------    ->   Tfz =   ---------
	//                                                    (s+225)                 z - 0.1054

}

Eigen::VectorXf filter_accelleration(Eigen::VectorXf accelleration, Eigen::Ref<Eigen::VectorXf> accelleration_state) {
	Eigen::VectorXf y, X_d;
	y = accelleration_state * 0.7741 + 0.5363 * accelleration;
	accelleration_state = 0.1653 * accelleration_state + 0.5 * accelleration;
	return y;
}

//    note on Discrete Filter: [t=0.01]                225                     0.6024 z + 0.2922
//                                           Tf =   ---------    ->   Tfz =   ---------
//                                                   (s+225)                  z - 0.1054


Eigen::VectorXf filter_Torques(Eigen::VectorXf torques, Eigen::Ref<Eigen::VectorXf> torques_state) {
	Eigen::VectorXf y, X_d;
	y = torques_state * 0.28;
	torques_state = 0.07 * torques_state + 0.25 * torques;
	return y;
}

//    note on Discrete Filter: [t=0.01]                200                     0.8647
//                                           Tf =   ---------    ->   Tfz =   ---------
//                                                   (s+200)                 z - 0.1353



/**
* @brief Rotation matrix 2 Roll Pitch Yaw angles
* Convert the input rotation matrix in the corresponding triple of roll pitch yaw angles
*/
Eigen::Vector3f rot2rpy(const Eigen::Matrix3f& R) {

	Eigen::Vector3f rpy_p, rpy_m;
	double pitch_p = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
	double pitch_m = atan2(-R(2, 0), -sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));

	if (abs(abs(pitch_p) - M_PI / 2) < 0.001) {
		//std::cout <<  "ERROR: inverse kinematic singularity" << '\n';
	}

	double roll_p = atan2(R(2, 1) / cos(pitch_p), R(2, 2) / cos(pitch_p));
	double roll_m = atan2(R(2, 1) / cos(pitch_m), R(2, 2) / cos(pitch_m));

	double yaw_p = atan2(R(1, 0) / cos(pitch_p), R(0, 0) / cos(pitch_p));
	double yaw_m = atan2(R(1, 0) / cos(pitch_m), R(0, 0) / cos(pitch_m));

	rpy_p << roll_p, pitch_p, yaw_p;
	rpy_m << roll_m, pitch_m, yaw_m;//*/

	return rpy_p;

}




float lowPassFrequency(float v_new, float v) {
	float output;
	double RC = 1.0 / (CUTOFF * 2 * 3.14);
	double dt = 1.0 / SAMPLE_RATE;
	double alpha = dt / (RC + dt);
	output = v + (alpha * (v_new - v));
	return output;
}

void saveToFile(std::stringstream& buff, std::string name)
{
	//---------------------------------------------
	//Input/Output State Bits
	//---------------------------------------------
	//| Goodbit | false/true | ERROS/NO ERRORS
	//| Eofbit  | 1          | End of File bit SET
	//| Failbit | 2          | Fail bit SET
	//| Badbit  | 4          | Bad bit SET
	//---------------------------------------------
	//| E+F     | 3
	//| E+B     | 5
	//| F+B     | 6
	//| E+F+B   | 7
	//---------------------------------------------
	std::ofstream data;
	data.open(name, std::ofstream::app);
	int i = 0;
	int j = 0;
	if (data.is_open())
	{
		std::cout << "Writing to " << name;
		do
		{
			if (i > 3) break;
			else
			{
				i++;
				std::cout << ".";
				data.write(buff.str().c_str(), buff.str().length());
			}
		} while (!data.good());
		if (data.good())
		{
			std::cout << "OK";
			std::cout << "\nClosing file";
		}
		else
		{
			std::cout << "ERRORS ON STREAM(" << data.rdstate() << ")";
			std::cout << "\nClosing file";
			data.clear();
		}
		do
		{
			std::cout << ".";
			data.close();
		} while (data.fail());
		std::cout << "OK\n";
	}
	else
	{
		std::cout << "Error while opening file " << name << "!";
	}

}


Eigen::Matrix3f eulAng2Rot(const float& alpha, const float& beta, const float& gamma) {

	Eigen::Matrix3f R, Rx, Ry, Rz;
	R.setIdentity();

	Rx << 1, 0, 0,
		0, cos(alpha), -sin(alpha),
		0, sin(alpha), cos(alpha);

	Ry << cos(beta), 0, sin(beta),
		0, 1, 0,
		-sin(beta), 0, cos(beta);

	Rz << cos(gamma), -sin(gamma), 0,
		sin(gamma), cos(gamma), 0,
		0, 0, 1;


	R = Rx * Ry * Rz;

	return R;
}

float clamp(const float& in, const float& min, const float& max) {

	return ((in < min) ? min : ((in > max) ? max : in));

}

Eigen::Vector3d clampVec(const Eigen::Vector3d& in, const float& min, const float& max) {

	Eigen::Vector3d out;

	for (int i = 0; i < 3; i++) {
		out(i) = clamp(in(i), min, max);
	}

	return out;

}

Eigen::Vector3f projVec(const Eigen::Vector3f& v, const Eigen::Vector3f& p) {

	Eigen::Vector3f ret;
	float scalar;

	scalar = v.dot(p);
	scalar /= p.norm();

	ret = scalar * p;

	return ret;

}

void usleep(DWORD waitTime) {
	LARGE_INTEGER perfCnt, start, now;

	QueryPerformanceFrequency(&perfCnt);
	QueryPerformanceCounter(&start);

	do {
		QueryPerformanceCounter((LARGE_INTEGER*)&now);
	} while ((now.QuadPart - start.QuadPart) / float(perfCnt.QuadPart) * 1000 * 1000 < waitTime);
}
