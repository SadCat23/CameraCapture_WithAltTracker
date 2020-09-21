#include <opencv2/highgui/highgui.hpp>
#include <iostream>;
#include <Antilatency.InterfaceContract.LibraryLoader.h>
#include <Antilatency.DeviceNetwork.h>
#include <Antilatency.StorageClient.h>
#include <Antilatency.Alt.Tracking.h>
#include <fstream>


Antilatency::DeviceNetwork::INetwork deviceNetwork;
Antilatency::Alt::Tracking::ILibrary altTrackingLibrary;
Antilatency::Alt::Tracking::ITrackingCotask trackingCotask;
Antilatency::Alt::Tracking::IEnvironment environment;
Antilatency::Math::floatP3Q placement;

bool exitRequested = false;


BOOL WINAPI consoleHandler(DWORD signal)
{
	if (signal == CTRL_C_EVENT)
	{
		exitRequested = true;
		return FALSE;
	}
	else if (signal == CTRL_CLOSE_EVENT || signal == CTRL_BREAK_EVENT)
	{
		exitRequested = true;
		return FALSE;
	}

	return FALSE;
}



void StopTrackingTask()
{
	if (trackingCotask != nullptr)
	{
		trackingCotask = {};
	}
}

Antilatency::DeviceNetwork::NodeHandle GetTrackingNode()
{
	auto result = Antilatency::DeviceNetwork::NodeHandle::Null;

	auto cotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();

	auto nodes = cotaskConstructor.findSupportedNodes(deviceNetwork);
	if (!nodes.empty())
	{
		for (auto node : nodes)
		{
			if (deviceNetwork.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle)
			{
				result = node;
				break;
			}
		}
	}

	return result;
}

void RunTrackingTask(Antilatency::DeviceNetwork::NodeHandle node)
{


	while (trackingCotask != nullptr && !trackingCotask.isTaskFinished())
	{
		if (exitRequested)
		{
			break;
		}

		//Get raw tracker state
		auto state = trackingCotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
		std::cout << "Raw position x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;

		//Get extrapolated tracker state with placement correction
		auto extrapolatedState = trackingCotask.getExtrapolatedState(placement, 0.06f);
		std::cout << "Extrapolated position x: " << extrapolatedState.pose.position.x << ", y: " << extrapolatedState.pose.position.y << ", z: " << extrapolatedState.pose.position.z << std::endl;

		std::cout << "Current tracking stage: " << (int32_t)extrapolatedState.stability.stage << std::endl;

		//5 FPS pose printing
		Sleep(200);
	}

	trackingCotask = {};
}

bool altInit(uint32_t &updateId, Antilatency::DeviceNetwork::NodeHandle &trackingNode)
{
	if (!SetConsoleCtrlHandler(consoleHandler, TRUE))
	{
		printf("\nERROR: Could not set control handler");
		return 0;
	}

#ifdef _WIN64
	SetDllDirectory(L"../../bin/win/x64");
#else
	SetDllDirectory(L"../../bin/win/x86");
#endif

	//Load libraries
	auto adnLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>("AntilatencyDeviceNetwork");
	auto antilatencyStorageClient = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::StorageClient::ILibrary>("AntilatencyStorageClient");
	altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>("AntilatencyAltTracking");

	if (adnLibrary == nullptr)
	{
		std::cout << "Failed to load AntilatencyDeviceNetwork library" << std::endl;
		return 0; 
	}

	if (antilatencyStorageClient == nullptr)
	{
		std::cout << "Failed to load AntilatencyStorageClient library" << std::endl;
		return 0;
	}

	if (altTrackingLibrary == nullptr)
	{
		std::cout << "Failed to load AntilatencyAltTracking library" << std::endl;
		return 0;
	}

	//For Android devices you have to call these functions FIRST before using any library functions passing JavaVM and your application context as jobject
#ifdef _ANDROID_
	auto vm = nullptr; //Pointer to JavaVM
	auto context = nullptr; //Your application context

	auto adnJni = adnLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
	adnJni.initJni(vm, context);

	auto altSystemClientJni = antilatencyStorageClient.queryInterface<AndroidJniWrapper::IAndroidJni>();
	altSystemClientJni.initJni(vm, context);
#endif

	//Set log verbosity level for Antilatency Device Network library.
	adnLibrary.setLogLevel(Antilatency::DeviceNetwork::LogLevel::Info);

	std::cout << "ADN version: " << adnLibrary.getVersion() << std::endl;

	//Alt socket USB device ID
	Antilatency::DeviceNetwork::UsbDeviceType antilatencyUsbDeviceType;
	antilatencyUsbDeviceType.pid = 0x0000;
	antilatencyUsbDeviceType.vid = Antilatency::DeviceNetwork::UsbVendorId::Antilatency;

	//Alt socket USB device ID (deprecated)
	Antilatency::DeviceNetwork::UsbDeviceType antilatencyUsbDeviceTypeLegacy;
	antilatencyUsbDeviceTypeLegacy.pid = 0x0000;
	antilatencyUsbDeviceTypeLegacy.vid = Antilatency::DeviceNetwork::UsbVendorId::AntilatencyLegacy;

	deviceNetwork = adnLibrary.createNetwork(std::vector<Antilatency::DeviceNetwork::UsbDeviceType>{antilatencyUsbDeviceType, antilatencyUsbDeviceTypeLegacy});

	auto environmentCode = antilatencyStorageClient.getLocalStorage().read("environment", "default");
	auto placementCode = antilatencyStorageClient.getLocalStorage().read("placement", "default");

	environment = altTrackingLibrary.createEnvironment(environmentCode);
	placement = altTrackingLibrary.createPlacement(placementCode);

	auto markers = environment.getMarkers();
	std::cout << "Environment markers count: " << markers.size() << std::endl;
	for (auto i = 0; i < markers.size(); ++i)
	{
		std::cout << "Marker " << i << ": {" << markers[i].x << ", " << markers[i].y << ", " << markers[i].z << "}" << std::endl;
	}


}


int main()
{
	std::cout << "Alt tracking on Y/N\n";
	std::string altOnStr;
	std::cin >> altOnStr;
	bool altOn = false;
	if (altOnStr == "Y")
	{
		altOn = true;
	}
	std::ofstream foutALT;
	std::ofstream foutTime;
	std::ofstream foutOnlyALTPos;
	if (altOn)
	{
		foutALT.open("timeStepsANDAltPos.txt");
		foutOnlyALTPos.open("altPos.txt");
	}
	foutTime.open("timeStepsOnly.txt");
	uint32_t updateId = 0;
	Antilatency::DeviceNetwork::NodeHandle trackingNode;
	
	altInit(updateId, trackingNode);
	

	int camID;
	std::cout << "enter cameraID:";
	std::cin >> camID;
	cv::VideoCapture cap(camID);
	cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
	cv::Mat im;
	bool haveActiveTracker = false;
	int frameCount = 0;
	while (true)
	{
		if (altOn)
		{
			auto newUpdateId = deviceNetwork.getUpdateId();

			if (haveActiveTracker == false)
			{
				if (updateId != newUpdateId)
				{
					updateId = newUpdateId;

					std::cout << "Factory update id has been incremented, searching for available tracking node..." << std::endl;

					trackingNode = GetTrackingNode();
					if (trackingNode != Antilatency::DeviceNetwork::NodeHandle::Null)
					{
						//Found tracking node
						auto nodeSerialNo = deviceNetwork.nodeGetStringProperty(deviceNetwork.nodeGetParent(trackingNode), Antilatency::DeviceNetwork::Interop::Constants::HardwareSerialNumberKey);
						std::cout << "Tracking node found, serial number: " << nodeSerialNo << std::endl;
						haveActiveTracker = true;
						auto cotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();
						trackingCotask = cotaskConstructor.startTask(deviceNetwork, trackingNode, environment);
						std::cout << "Run tracking" << std::endl;
					}
					else
					{
						haveActiveTracker = false;
						std::cout << "Tracking node not found." << std::endl;
					}
				}
			}
			if (haveActiveTracker == true)
			{
				//Get raw tracker state
				auto state = trackingCotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
				std::cout << "Raw position x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;

				//Get extrapolated tracker state with placement correction
				auto extrapolatedState = trackingCotask.getExtrapolatedState(placement, 0.06f);
				std::cout << "Extrapolated position x: " << extrapolatedState.pose.position.x << ", y: " << extrapolatedState.pose.position.y << ", z: " << extrapolatedState.pose.position.z << std::endl;

				std::cout << "Current tracking stage: " << (int32_t)extrapolatedState.stability.stage << std::endl;

			}
		}
		//5 FPS pose printing
		
		//Sleep(200);
		
		__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		
		cap.read(im);
		cv::imshow("Image", im);
		frameCount++;
	
		cv::imwrite("../images/" + std::to_string(now) + ".jpeg", im);
		if (frameCount % 10 == 0)
		{
			cv::imwrite("../images_10fps/" + std::to_string(now) + ".jpeg", im);
			std::cout << "Write keyFrame\n";
		}
		char sep = ';';
		if (altOn)
		{
			auto extrapolatedState = trackingCotask.getExtrapolatedState(placement, 0.06f);
			foutALT << now << sep << extrapolatedState.pose.position.x << sep << extrapolatedState.pose.position.y << sep << extrapolatedState.pose.position.z << sep << extrapolatedState.pose.rotation.x << sep << extrapolatedState.pose.rotation.y << sep << extrapolatedState.pose.rotation.z << "\n";
			foutOnlyALTPos << extrapolatedState.pose.position.x << sep << extrapolatedState.pose.position.y << sep << extrapolatedState.pose.position.z;
		}
		foutTime << now<<"\n";
		if (cv::waitKey(1) >= 0)
			break;
	}
	
	if (altOn)
	{
		foutALT.close();
	}
	foutTime.close();
	cap.release();



	return 0;
}

