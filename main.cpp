#include "Upsurge.h"


int main(){
	std::map<std::string,double> values;

/*	values["lowH"] = 55;	//Prelimenary Green Values
	values["lowS"] = 125;
	values["lowV"] = 110;
	values["highH"] = 75;
	values["highS"] = 255;
	values["highV"] = 255;
	values["minAreaFilter"] = 20;
	values["maxAreaFilter"] = 200;
	values["blur"]=1;
	values["inputWidth"] = 424;
	values["inputHeight"] = 240;
	values["procWidth"] = 424/2;
	values["procHeight"] = 240/2;
	values["FPS"] = 60;
	values["colorExposure"] = 5;
	values["depthExposure"] = 300;
	values["tapeSpread"] = 11.5;
	values["leftTargetXOffset"] = -12;
	values["leftTargetYOffset"] = -3;
	values["rightTargetXOffset"] = 12;
	values["rightTargetYOffset"] = -3;
	values["leftWheelXCameraOffset"] = -12;
	values["leftWheelYCameraOffset"] = -0.5;
	values["rightWheelXCameraOffset"] = 12;
	values["rightWheelYCameraOffset"] = -0.5;
	values["wheelDiameter"] = 6;*/

	values["lowH"] = 40;	//Test Values
	values["lowS"] = 100;
	values["lowV"] = 75;
	values["highH"] = 75;
	values["highS"] = 190;
	values["highV"] = 200;
	values["minAreaFilter"] = 100;
	values["maxAreaFilter"] = 20000;
	values["blur"]=5;
	values["inputWidth"] = 424;
	values["inputHeight"] = 240;
	values["procWidth"] = 424;
	values["procHeight"] = 240;
	values["FPS"] = 60;
	values["colorExposure"] = 150;
	values["depthExposure"] = 300;
	values["angleTolerance"] = 1;
	values["tapeSpread"] = 11.5;
	values["leftTargetXOffset"] = 12;
	values["leftTargetYOffset"] = 3;
	values["rightTargetXOffset"] = 12;
	values["rightTargetYOffset"] = 3;
	values["leftWheelXCameraOffset"] = -15;
	values["leftWheelYCameraOffset"] = -6;
	values["rightWheelXCameraOffset"] = 15;
	values["rightWheelYCameraOffset"] = -6;
	values["wheelDiameter"] = 6;
	
	Upsurge vision(values);
	vision.start(values);
	
	
	
	boost::asio::io_service ioService;
	boost::asio::ip::tcp::socket dataSocket(ioService);	
	
	std::cout<<vision.getRioIP()<<std::endl;	

	//Connect front drivetrain stream
	vision.connectTCPStream(dataSocket, vision.getRioIP(), 5431);
	
		
	
	while(true){
		auto start = std::chrono::high_resolution_clock::now();
		std::array<float, 4> results = vision.procFrames(values);	
		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = finish - start;
		std::cout << "FPS: " << 1/(elapsed.count()) <<std::endl;
		vision.showFrames();
		vision.showProcFrames();
	}
	
}
