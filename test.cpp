#include <iostream>
#include <cmath>

int main(){

	float tapeSpread = 11.5;
	float leftDistance = depth_frame.get_distance(boundingBoxes.at(i-1).center.x, boundingBoxes.at(i-1).center.y) * 39.37;
	float rightDistance = depth_frame.get_distance(centerX, centerY) * 39.37;
	float cameraX=values["cameraX"];
	float cameraY=values["cameraY"];
	float LwheelXOffset = values["leftWheelXCameraOffset"];	//Always Posotive
	float LwheelYOffset = values["leftWheelYCameraOffset"];
	float RwheelXOffset = values["rightWheelXCameraOffset"];	//Always posotive
	float RwheelYOffset = values["rightWheelYCameraOffset"];
	float LXOffset = values["leftTargetXOffset"];	//Next 4 Always Posotive
	float LYOffset = values["leftTargetYOffset"];
	float RXOffset = values["rightTargetXOffset"];
	float RYOffset = values["rightTargetYOffset"];
	float LTapeY = leftDistance;
	float LTapeX = -sqrt((tapeSpread*tapeSpread)-std::abs(rightDistance-leftDistance)) / 2;
	std::cout<<"Left Tape X: " << LTapeX <<std::endl;
	float RTapeY = rightDistance;
	float RTapeX = sqrt((tapeSpread*tapeSpread)-std::abs(rightDistance-leftDistance)) / 2;
	/*float tapeSpread = 11.5;
	float leftDistance = 35.02;
	float rightDistance = 38.1091;
	float cameraX=0;
	float cameraY=0;
	float LwheelXOffset = 11.5;
	float LwheelYOffset = 0.5;
	float RwheelXOffset = 11.5;	
	float RwheelYOffset = 0.5;
	float LXOffset = 6;
	float LYOffset = 5;
	float RXOffset = 6;
	float RYOffset = 5;
	float LTapeY = leftDistance;
	float LTapeX = -sqrt((tapeSpread*tapeSpread)-std::abs(rightDistance-leftDistance)) / 2;
	std::cout<<"Left Tape X: " << LTapeX <<std::endl;
	float RTapeY = rightDistance;
	float RTapeX = sqrt((tapeSpread*tapeSpread)-std::abs(rightDistance-leftDistance)) / 2;
	std::cout<<"Right Tape X: " << RTapeX <<std::endl;*/
	std::cout<<"Absolute Value: "<< std::abs(rightDistance-leftDistance) << std::endl;
	float m = (RTapeY-LTapeY)/(RTapeX-LTapeX);
	std::cout<<"Slope: "<<m<<std::endl;
	float mRecip = -1/m;
	if(leftDistance>rightDistance){
		float LRelativeXOffsetX = LTapeX-(LXOffset/sqrt(1+(m*m)));
		float LRelativeXOffsetY = LTapeY+(LXOffset/sqrt(1+(1/(m*m))));
		float RRelativeXOffsetX = RTapeX+(RXOffset/sqrt(1+(m*m)));
		float RRelativeXOffsetY = RTapeY-(RXOffset/sqrt(1+(1/(m*m))));
		
		float LRelativeYOffsetX = LRelativeXOffsetX-(LYOffset/sqrt(1+(mRecip*mRecip)));
		float LRelativeYOffsetY = LRelativeXOffsetY-(LYOffset/sqrt(1+(1/(mRecip*mRecip))));
		float RRelativeYOffsetX = RRelativeXOffsetX-(RYOffset/sqrt(1+(mRecip*mRecip)));
		float RRelativeYOffsetY = RRelativeXOffsetY-(RYOffset/sqrt(1+(1/(mRecip*mRecip))));
		
		float leftTravel = sqrt(pow(LRelativeYOffsetX-(cameraX-LwheelXOffset),2) + pow(LRelativeYOffsetY-(cameraY-LwheelYOffset),2));
		float rightTravel = sqrt(pow(RRelativeYOffsetX-(cameraX+RwheelXOffset),2) + pow(RRelativeYOffsetY-(cameraY-LwheelYOffset),2));
		std::cout<<"Left Travel: " << leftTravel<<std::endl;
		std::cout<<"Right Travel: " << rightTravel<<std::endl;
	
	}
	else{
		float LRelativeXOffsetX = LTapeX-(LXOffset/sqrt(1+(m*m)));
		std::cout<< "LRelativeXOffsetX: " <<  LRelativeXOffsetX <<std::endl;
		float LRelativeXOffsetY = LTapeY-(LXOffset/sqrt(1+(1/(m*m))));
		std::cout<< "LRelativeXOffsetY: " <<  LRelativeXOffsetY <<std::endl;
		float RRelativeXOffsetX = RTapeX+(RXOffset/sqrt(1+(m*m)));
		std::cout<< "RRelativeXOffsetX: " <<  RRelativeXOffsetX <<std::endl;
		float RRelativeXOffsetY = RTapeY+(RXOffset/sqrt(1+(1/(m*m))));
		std::cout<< "RRelativeXOffsetY: " << RRelativeXOffsetY <<std::endl;
		
		float LRelativeYOffsetX = LRelativeXOffsetX+(LYOffset/sqrt(1+(mRecip*mRecip)));
		float LRelativeYOffsetY = LRelativeXOffsetY-(LYOffset/sqrt(1+(1/(mRecip*mRecip))));
		float RRelativeYOffsetX = RRelativeXOffsetX+(RYOffset/sqrt(1+(mRecip*mRecip)));
		float RRelativeYOffsetY = RRelativeXOffsetY-(RYOffset/sqrt(1+(1/(mRecip*mRecip))));
		
		float leftTravel = sqrt(pow(LRelativeYOffsetX-(cameraX-LwheelXOffset),2) + pow(LRelativeYOffsetY-(cameraY-LwheelYOffset),2));
		float rightTravel = sqrt(pow(RRelativeYOffsetX-(cameraX+RwheelXOffset),2) + pow(RRelativeYOffsetY-(cameraY-LwheelYOffset),2));
		std::cout<<"Left Travel: " << leftTravel<<std::endl;
		std::cout<<"Right Travel: " << rightTravel<<std::endl;
	}
	
}
