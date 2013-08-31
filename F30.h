#ifndef INCLUDED_F30_H
#define INCLUDED_F30_H

#include "OpenCV.h"
#include <iostream>

class F30{

public:
	
	F30(void);
	~F30(void);
	void Update(void);
	void CreateThermalImage();
	cv::Mat getImg(void);
	void showImage(void);
	void saveImage(char* str);

private:
	static const int            Width; 
	static const int            Height; 
	cv::Mat raw;
	cv::Mat rgb;

	cv::VideoCapture VideoCapture;
	void getRGB(float ratio,unsigned char* r,unsigned char* g,unsigned char* b);
	void setRGB(cv::Vec3b* ref,unsigned char r,unsigned char g,unsigned char b);

}


#endif