#include <iostream>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <ctime>
#include <ratio>
#include <unistd.h>
#include <sstream>
#include <boost/asio.hpp>
#include <algorithm>
#include <string>
#include "base64.h"

class Upsurge{
private:
	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::frameset frames;
	cv::Mat depth;
	cv::Mat color;
	cv::Mat HSV;
	cv::Mat threshold;
	cv::Mat contourImage;
	float DPP;
	float pairAngle;
	bool pairFound;
	double Rd;
	double Ld;
	double pi;
	double circ;
	float rightNeededTravel;
	float leftNeededTravel;
	std::vector<int> compressionParams;

public:
	Upsurge(std::map<std::string,double>& values);
	std::array<float,4> procFrames(std::map< std::string, double>& values);
	void start(std::map<std::string,double>& values);
	void showFrames();
	void showProcFrames();
	float normalizeDiagnolAngle(cv::RotatedRect &a);
	static bool sortBoxesLTOR(const cv::RotatedRect &a, const cv::RotatedRect &b);
	static bool sortBoxesRTOL(const cv::RotatedRect &a, const cv::RotatedRect &b);
	void pairFinder();
	std::string getRioIP();
	void connectTCPStream(boost::asio::ip::tcp::socket &sock ,std::string rawIP,unsigned short int portNum); 
	void sendData(boost::asio::ip::tcp::socket &sock);
	void sendFrame(boost::asio::ip::tcp::socket &sock);
	float euclidianDistance(float x1, float y1, float x2, float y2);
};
