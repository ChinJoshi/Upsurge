#include "Upsurge.h"

	Upsurge::Upsurge(std::map< std::string, double>& values){
		cfg.enable_stream(RS2_STREAM_DEPTH, values["inputWidth"] , values["inputHeight"] , RS2_FORMAT_Z16, values["FPS"] );
		cfg.enable_stream(RS2_STREAM_COLOR, values["inputWidth"], values["inputHeight"], RS2_FORMAT_BGR8 , values["FPS"] );
		std::cout<<"Set Width: " << values["inputWidth"] << std::endl << "Set Height: " << values["inputHeight"] << std::endl;
		DPP = 69.4/values["procWidth"];
		pi = atan(1) * 4;
		circ = pi*values["wheelDiameter"];
		pairFound=false;
		compressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
		compressionParams.push_back(values["jpegCompression"]);
	}

	std::string Upsurge::getRioIP(){
		std::string cmd="getent hosts roboRIO-5431-FRC.local";
		std::string data;
		FILE * stream;
		const int max_buffer = 256;
		char buffer[max_buffer];
		cmd.append(" 2>&1");
		stream = popen(cmd.c_str(),"r");
		if (stream) {
		while (!feof(stream))
		if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
		pclose(stream);
		}
		std::istringstream f(data);
		std::string ip;
		std::getline(f, ip, ' ');
		return ip;
	}

	void Upsurge::connectTCPStream(boost::asio::ip::tcp::socket &sock,std::string rawIP,unsigned short int portNum){
		boost::system::error_code ec;
		boost::asio::ip::address ipAddress = boost::asio::ip::address::from_string(rawIP);
		boost::asio::ip::tcp::endpoint endpoint(ipAddress, portNum);
		sock.connect(endpoint, ec);
		if (ec) {
			std::cout << "AN ERROR HAS OCCURED IN CONNECTING!!!" << std::endl;
			std::cout << ec.message() << std::endl;;
		}
		else{	
			std::cout<<"Succesfully Connected"<<std::endl;
		}
	}

	void Upsurge::sendData(boost::asio::ip::tcp::socket &sock){
		try{
			if(pairFound=true){
				std::string data = std::to_string(leftNeededTravel) + ","  + std::to_string(rightNeededTravel)+"\n";
				boost::asio::write(sock, boost::asio::buffer(data));
				std::cout<<"Sent Pair Data"<<std::endl;
			}
			else{
				std::cout<<"Send Null"<<std::endl;
				boost::asio::write(sock, boost::asio::buffer("null,null\n"));
			}
		}
		catch(const boost::system::system_error &ex){
			std::cout << "Memed"<<std::endl;
		}
	}

	void Upsurge::sendFrame(boost::asio::ip::tcp::socket &sock){
		if(color.empty()){
			std::cout<<"Cannot Send Because No Color Image Yet"<<std::endl;
		}
		else{
			try{
				cv::Mat tiny;
				cv::resize(color,tiny,cv::Size(),0.5,0.5,CV_INTER_AREA);
				std::vector<uchar> buf;
				cv::imencode(".jpg",tiny,buf,compressionParams);
				uchar *encImg = reinterpret_cast<uchar*>(buf.data());
				std::string encoded = base64_encode(encImg, buf.size());
				boost::asio::write(sock, boost::asio::buffer(encoded+"\n"));
			}
			catch(const boost::system::system_error &ex){
				std::cout<<"Frame Memed"<<std::endl;
			}
		}
	}


	void Upsurge::start(std::map< std::string, double>& values){
		pipe.start(cfg);
		rs2::device dev = pipe.get_active_profile().get_device();
		dev.query_sensors()[1].set_option(rs2_option::RS2_OPTION_EXPOSURE,values["colorExposure"] );
		dev.query_sensors()[0].set_option(rs2_option::RS2_OPTION_EXPOSURE,values["depthExposure"] );
		std::cout << dev.query_sensors()[1].get_info(RS2_CAMERA_INFO_NAME) << " expsure is: " << values["colorExposure"] <<std::endl;
		std::cout << dev.query_sensors()[0].get_info(RS2_CAMERA_INFO_NAME) << " expsure is: " << dev.query_sensors()[0].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
		std::cout<<"Pipe Started"<<std::endl;
	}

	std::array<float,4> Upsurge::procFrames(std::map<std::string,double>& values){
		pairFound=false;
		frames = pipe.wait_for_frames();
		rs2::align align(RS2_STREAM_COLOR);
		frames = align.process(frames);
		rs2::video_frame color_frame = frames.get_color_frame();
		rs2::depth_frame depth_frame = frames.get_depth_frame();
		depth = cv::Mat(cv::Size(values["inputWidth"], values["inputHeight"]), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
		color = cv::Mat(cv::Size(values["inputWidth"], values["inputHeight"]), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);


		cv::GaussianBlur(color , color, cv::Size(values["blur"],values["blur"]),0,0);

		cv::cvtColor(color, HSV, cv::COLOR_BGR2HSV);
		cv::inRange(HSV, cv::Scalar(values["lowH"], values["lowS"], values["lowV"]), cv::Scalar(values["highH"], values["highS"], values["highV"]), threshold);
		std::vector<std::vector<cv::Point> > contours;
		
		cv::findContours( threshold, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
		std::vector< std::vector<cv::Point>> countours_poly(contours.size() );
		
		std::vector<cv::RotatedRect> boundingBoxes;
		
		contourImage = cv::Mat(color.size(), CV_8UC3, cv::Scalar(0,0,0));
		std::vector<float> distance;		

		for(int i = 0; i<contours.size(); i++){
			if(cv::contourArea(contours.at(i)) > values["minAreaFilter"] && cv::contourArea(contours.at(i)) < values["maxAreaFilter"]){
				cv::RotatedRect bounder = cv::minAreaRect(contours.at(i));
				if( ( bounder.size.height*1.5 > bounder.size.width && normalizeDiagnolAngle(bounder)>90 ) || ( bounder.size.width*1.5 > bounder.size.height && normalizeDiagnolAngle(bounder)<90 )   ){
					boundingBoxes.push_back( bounder );
				}
			}
		}
		
		std::sort(boundingBoxes.begin(), boundingBoxes.end(), sortBoxesLTOR);
		
		for(int i = 0; i<boundingBoxes.size(); i++){
			float currentDiagnolAngle = 666;
			float prevDiagnolAngle;


			currentDiagnolAngle = normalizeDiagnolAngle( boundingBoxes.at(i) );
			int centerX = boundingBoxes.at(i).center.x;
			int centerY = boundingBoxes.at(i).center.y;

			float horizontalAngle = (centerX*DPP) - 34.7 ;

			if((i>0) && (0<prevDiagnolAngle && prevDiagnolAngle<60) && (145.5<currentDiagnolAngle && currentDiagnolAngle<178.5)){
				pairFound=true;
				pairAngle = (( (centerX + boundingBoxes.at(i-1).center.x)/2 )*DPP) - 34.7;


				cv::Point2f rect_points[4];
				boundingBoxes.at(i).points(rect_points);
				for(int j = 0; j<4; j++){
					cv::line(contourImage, rect_points[j], rect_points[(j+1)%4], cv::Scalar(110,255,65), 3);
				}

				boundingBoxes.at(i-1).points(rect_points);
				for(int j = 0; j<4; j++){
					cv::line(contourImage, rect_points[j], rect_points[(j+1)%4], cv::Scalar(110,255,65), 3);
				}

				if(pairAngle<1 && pairAngle>-1){
					std::cout<<"Centered on target pair"<<std::endl;
					float leftTapeY = depth_frame.get_distance(boundingBoxes.at(i-1).center.x, boundingBoxes.at(i-1).center.y) * 39.37;
					float rightTapeY = depth_frame.get_distance(centerX, centerY) * 39.37;
					float leftTapeX = -(sqrt( pow(values["tapeSpread"],2)-(abs(rightTapeY-leftTapeY))  )) /2;
					float rightTapeX = (sqrt( pow(values["tapeSpread"],2)-(abs(rightTapeY-leftTapeY))  )) /2;
					float slope = (rightTapeY-leftTapeY)/(rightTapeX-leftTapeX);
					float recipSlope = -1/slope;

					float leftTargetXOffsetX;
					float leftTargetXOffsetY; 
					float rightTargetXOffsetX;
					float rightTargetXOffsetY;

					float leftTargetYOffsetX;
					float leftTargetYOffsetY;
					float rightTargetYOffsetX; 
					float rightTargetYOffsetY; 
					if(leftTapeY>rightTapeY){
						leftTargetXOffsetX = leftTapeX - ((values["leftTargetXOffset"])/(sqrt(1+(slope*slope))));
						
						leftTargetXOffsetY = leftTapeY + ( (values["leftTargetXOffset"]) / ( sqrt(( 1 + ( (1)/(slope*slope) ) )) ) );
						
						rightTargetXOffsetX = rightTapeX + ( (values["rightTargetXOffset"])/(sqrt(1+(slope*slope)))  );
						
						rightTargetXOffsetY = rightTapeY - ( (values["rightTargetXOffset"]) / (sqrt((1 + ( (1)/(slope*slope) ) ) ))  );


						leftTargetYOffsetX = leftTargetXOffsetX + ((values["leftTargetYOffset"])/(sqrt((1)+(recipSlope*recipSlope))));

						leftTargetYOffsetY = leftTargetXOffsetY - ((values["leftTargetYOffset"])/(sqrt(((1)+(1/(recipSlope*recipSlope))))));

						rightTargetYOffsetX = rightTargetXOffsetX + ((values["rightTargetYOffset"])/(sqrt(((1)+(recipSlope*recipSlope)))));

						rightTargetYOffsetY = rightTargetXOffsetY - ((values["righttargetYOffset"])/(sqrt(((1)+((1)/(recipSlope*recipSlope))))));
					}
					else{
						leftTargetXOffsetX = leftTapeX - ((values["leftTargetXOffset"])/(sqrt(1+(slope*slope))));
						
						leftTargetXOffsetY = leftTapeY - ( (values["leftTargetXOffset"]) / ( sqrt(( 1 + ( (1)/(slope*slope) ) )) ) );
						
						rightTargetXOffsetX = rightTapeX + ( (values["rightTargetXOffset"])/(sqrt(1+(slope*slope)))  );
						
						rightTargetXOffsetY = rightTapeY + ( (values["rightTargetXOffset"]) / (sqrt((1 + ( (1)/(slope*slope) ) ) ))  );


						leftTargetYOffsetX = leftTargetXOffsetX + ((values["leftTargetYOffset"])/(sqrt((1)+(recipSlope*recipSlope))));

						leftTargetYOffsetY = leftTargetXOffsetY - ((values["leftTargetYOffset"])/(sqrt(((1)+(1/(recipSlope*recipSlope))))));

						rightTargetYOffsetX = rightTargetXOffsetX + ((values["rightTargetYOffset"])/(sqrt(((1)+(recipSlope*recipSlope)))));

						rightTargetYOffsetY = rightTargetXOffsetY - ((values["righttargetYOffset"])/(sqrt(((1)+((1)/(recipSlope*recipSlope))))));
					}


					
					
					
					float rightTargetRelativeXOffset = rightTargetYOffsetX;
					float rightTargetRelativeYOffset = rightTargetYOffsetY;
					float leftTargetRelativeXOffset = leftTargetYOffsetX;
					float leftTargetRelativeYOffset = leftTargetYOffsetY;


					leftNeededTravel = euclidianDistance(leftTargetRelativeXOffset,leftTargetRelativeYOffset, values["leftWheelXCameraOffset"], values["leftWheelYCameraOffset"]);
					rightNeededTravel =  euclidianDistance(rightTargetRelativeXOffset, rightTargetRelativeYOffset, values["rightWheelXCameraOffset"], values["rightWheelYCameraOffset"] );

					std::cout<<"Left Depth Distance: "<< leftTapeY <<std::endl;
					std::cout<<"Right Depth Distance: "<< rightTapeY  <<std::endl;
					std::cout<<"Left Needed Travel: "<< leftNeededTravel <<std::endl;
					std::cout<<"Right Needed Travel: "<< rightNeededTravel <<std::endl;
					float leftRotations = leftNeededTravel/circ;
					float rightRotations = rightNeededTravel/circ;
					break;
				}
				else{
					std::cout<<"Tape Pair Horizontal Angle :"<<pairAngle<<std::endl;
					break;
				}
			}
			prevDiagnolAngle=currentDiagnolAngle;
		
		}
		
			
	}

	float Upsurge::euclidianDistance(float x1, float y1, float x2, float y2){
		return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0); 
	}
	
	void Upsurge::showFrames(){
		cv::imshow("Color Image", color);
		cv::imshow("Depth Image", depth);
		cv::waitKey(1);
	}
	
	void Upsurge::showProcFrames(){
		cv::imshow("HSV",HSV);
		cv::imshow("threshold",threshold);
		cv::imshow("Contours",contourImage);
		cv::waitKey(1);
	}

	bool Upsurge::sortBoxesLTOR(const cv::RotatedRect &a, const cv::RotatedRect &b){
		return a.center.x < b.center.x;
	}
	
	bool Upsurge::sortBoxesRTOL(const cv::RotatedRect &a, const cv::RotatedRect &b){
		return a.center.x > b.center.x;
	}

	float Upsurge::normalizeDiagnolAngle(cv::RotatedRect &a){
		float diagnolAngle;
		if(a.size.width < a.size.height){
			diagnolAngle = a.angle+180;
		}
		else{
			diagnolAngle = a.angle+90;
		}
		return diagnolAngle;
	}
