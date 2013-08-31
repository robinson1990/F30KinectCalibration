#include "F30.h"

const int F30::Width=640; 
const int F30::Height=480; 

F30::F30():
VideoCapture(1){
  if(!VideoCapture.isOpened()){
		std::cout<<"f30 is not opened."<<std::endl;
  }
} 

F30::~F30(void){
;	
}

void F30::Update(void){
		VideoCapture>>raw; 
	return;
}


cv::Mat F30::getImg(void){
	return rgb;
}

void F30::CreateThermalImage(){

	cv::Rect roi(0,0,720,480);
	cv::Mat rgb0(raw,roi);
	cv::resize(rgb0,rgb0,cv::Size(Width,Height),0,0,cv::INTER_NEAREST);
	rgb0.copyTo(rgb);
	cv::Mat greyrgb;
	cvtColor(rgb,greyrgb,CV_RGB2GRAY);

	  //inverse colour for calibration//

	  // 	for(int i=0;i<rgb0.size().height;++i){
	  //	for(int j=0;j<rgb0.size().width;++j){
	  //		cv::Vec3b p=rgb0.at<cv::Vec3b>(i,j);
	  //		p[0]=255-p[0];
	  //		p[1]=255-p[1];
	  //		p[2]=255-p[2];
	  //		rgb0.at<cv::Vec3b>(i,j)=p;
	  //	}
	  //}
	  //rgb0.copyTo(rgb);

	  //inverse colour for calibration//


	  //turn into fake colour//

	for(int y=0; y<Height; y++){
		for(int x=0; x<Width; x++){
			unsigned char r=0,g=0,b=0;
			float ratio=(float)greyrgb.at<cv::Vec3b>(y,x).val[0]/255.0f;
			if(ratio>0.0){
				getRGB(ratio,&r,&g,&b);			
				setRGB(&rgb.at<cv::Vec3b>(cv::Point2d(x,y)),r,g,b);
			}
		}
	}

	//turn into fake colour//

	return;
}

void F30::showImage(void){
	char key;

	bool loopFlag=true;
	while(loopFlag){

		F30::Update();
		F30::CreateThermalImage();

	//show
	cv::imshow("Capture",rgb);
	key=cv::waitKey(1);

	//save
	if(key=='s'){
		cv::imwrite("colour_image.bmp",rgb);
		std::cout<<"save colour_image"<<std::endl;
			loopFlag=false;
	}
		
	}
	return;
}

void F30::getRGB(float ratio,unsigned char* r,unsigned char* g,unsigned char* b){
	ratio = ratio>=0.99? 0.99:ratio;
	if(ratio<0.33){
		*r = ratio/0.33*255;
	}else if(ratio<0.66){
		*r = (0.66-ratio)/0.33*255;
		*g = (ratio-=0.33)/0.33*255;
	}else{
		*g = (0.99-ratio)/0.33*255;
		*b = (ratio-=0.66)/0.33*255;
	}
}

void F30::setRGB(cv::Vec3b* ref,unsigned char r,unsigned char g,unsigned char b){
	ref->val[0]=r;
	ref->val[1]=g;
	ref->val[2]=b;
}
