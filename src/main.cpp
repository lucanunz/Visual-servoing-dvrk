// Project includes
#include "utils.hpp"
#include "Oculus.hpp"
#include "vs_header.hpp"
#include "Geomagic.hpp"

// Eigen includes
#include <Eigen/Dense>

void escapeCallback();
void initLog();
void logData();

//////// Global variables
// // Oculus stringstream
std::stringstream vtrSS;				//!< 6D Velocity of the rift display expressed in the tracker reference frame
std::stringstream vrrSS;				//!< 6D Velocity of the rift display expressed in the rift reference frame
std::stringstream Ttr;					//!< Homogeneous transformation matrix expressing the pose of the Oculus rift wrt the tracker reference frame

// ECM stringstream
std::stringstream qdotECMSS;			//!< 4x1 Vector of ECM joint velocities
std::stringstream qECMSS;				//!< 4x1 Vector of ECM joint position
std::stringstream TbcSS;				//!< Homogeneous transformation matrix expressing the pose of the left camera wrt the base frame of the ECM
std::stringstream vcamLSS;				//!< Velocity vector of the left camera of the ECM
std::stringstream vcamRSS;				//!< Velocity vector of the right camera of the ECM


simxInt mainClient;
simxInt psmLClient;
simxInt psmRClient;
simxInt ecmClient;
simxInt vsClient;
simxFloat Ts;
bool running;

// Log variables
char curDirAndFile[1024];
std::string path;
int fileSeq;


int main(int argc, char** argv) {

	bool saveLog = false;

	// Initialize the connected devices
	std::cout << "Initializing the Geomagic device(s) ..." << std::endl;

	// Initialize the multiple connections with CoppeliaSim
	std::cout << "Initiliazing the multiple connections with CoppeliaSim ... " << std::endl;
	mainClient = simxStart("127.0.0.1", 19997, true, true, 5000, 5);

	// Start the simulation
	std::cout << "Starting the simulation ... " << std::endl;
	simxStartSimulation(mainClient, simx_opmode_blocking);

	psmLClient = simxStart("127.0.0.1", 19996, true, true, 5000, 5);
	psmRClient = simxStart("127.0.0.1", 19995, true, true, 5000, 5);
	ecmClient = simxStart("127.0.0.1", 19994, true, true, 5000, 5);
	vsClient = simxStart("127.0.0.1", 19993, true, true, 5000, 5);

	if (mainClient == -1 || psmLClient == -1 || psmRClient == -1 || ecmClient == -1 || vsClient == -1) {
		std::cout << "One of the channels with CoppeliaSim has not been opened. Exiting..." << std::endl;
		return -1;
	}

	simxGetFloatSignal(mainClient, "Ts", &Ts, simx_opmode_blocking);
	std::cout << "Simulation time step: " << Ts << std::endl;

	if (saveLog) {
		initLog();
	}

	// Set running flag on true to enable operations in next threads
	running = true;

	// Init and launch threads
	boost::thread_group group;
	//group.create_thread(boost::bind(hapticLoop));
	//group.create_thread(boost::bind(oculusLoop));
	group.create_thread(boost::bind(vs_loop));

	while (_getch() != 27);
	running = false;

	// Join threads
	group.join_all();

	simxStopSimulation(mainClient, simx_opmode_blocking);
	simxFinish(psmLClient);
	simxFinish(psmRClient);
	simxFinish(ecmClient);
	simxFinish(vsClient);

	if (saveLog) {
		// Save to file the logged data
		logData();
	}


	// Close files for logging
	std::ofstream seqFileOut;
	seqFileOut.open("sequeceFile.txt", std::ios::out);
	seqFileOut << fileSeq;
	seqFileOut.close();



	return 0;

}


void escapeCallback() {
	while (_getch() != 27);
	running = false;
	
}


void initLog() {


	_getcwd(curDirAndFile, sizeof(curDirAndFile));
	path = std::string(curDirAndFile);
	path += "/plot";

	std::ifstream seqFileIn;
	seqFileIn.open("sequeceFile.txt", std::ios::in);

	if (seqFileIn.is_open())
	{
		seqFileIn >> fileSeq;
		fileSeq++;
	}
	else {
		fileSeq = 1;
	}

	path = path + std::to_string(fileSeq);

	path += "/";

	CreateDirectory(path.c_str(), NULL);

	// Oculus stringstream
	vtrSS.str("");				//!< 6D Velocity of the rift display expressed in the tracker reference frame
	vrrSS.str("");				//!< 6D Velocity of the rift display expressed in the rift reference frame
	Ttr.str("");					//!< Homogeneous transformation matrix expressing the pose of the Oculus rift wrt the tracker reference frame

	// ECM stringstream
	qdotECMSS.str("");			//!< 4x1 Vector of ECM joint velocities
	qECMSS.str("");				//!< 4x1 Vector of ECM joint position
	TbcSS.str("");				//!< Homogeneous transformation matrix expressing the pose of the left camera wrt the base frame of the ECM
	vcamLSS.str("");				//!< Velocity vector of the left camera of the ECM
	vcamRSS.str("");				//!< Velocity vector of the right camera of the ECM



}

void logData() {

	// Oculus stringstream
	std::string vtrFile = path + "vtr.txt";
	std::string vrrFile = path + "vrr.txt";
	std::string TtrFile = path + "Ttr.txt";

	// ECM stringstream
	std::string qdotECMFile = path + "qdotECM.txt";
	std::string qECMFile = path + "qECM.txt";
	std::string TbcFile = path + "Tbc.txt";
	std::string vcamLFile = path + "vcamL.txt";
	std::string vcamRSSFile = path + "vcamRSS.txt";

	saveToFile(qdotECMSS, qdotECMFile);
	saveToFile(qECMSS, qECMFile);
	saveToFile(vtrSS, vtrFile);
}



