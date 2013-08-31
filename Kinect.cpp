#include "Kinect.h"

const int Kinect::Width = 640;
const int Kinect::Height = 480;

Kinect::Kinect():
	Image_Generator(0),
	Depth_Generator(0),
	ImageMD(0),
	Colour_Image(Kinect::Height, Kinect::Width),
	DepthMD(0),
	Depth_Image(Kinect::Height, Kinect::Width),
	Kinect_ID(0),
	Intrinsic_Matrix(3, 3){}
//�A�h���X��0�������ăA�N�Z�X�ᔽ��h��
Kinect::~Kinect(){
	delete   Image_Generator;
	delete   Depth_Generator;
	delete     	     ImageMD;
	delete  	     DepthMD;
	Image_Generator = 0;
	Depth_Generator = 0;
	ImageMD = 0;
	DepthMD = 0;
}

void Kinect::SetImageGenerator(xn::ImageGenerator* image_generator){
	Image_Generator = image_generator;
}
void Kinect::SetDepthGenerator(xn::DepthGenerator* depth_generator){
	Depth_Generator = depth_generator;
}
void Kinect::SetKinectID(int kinect_id){
	Kinect_ID = kinect_id;
}
xn::DepthMetaData*  Kinect::GetDepthMD()const{
	return DepthMD;
}

cv::Mat_<cv::Vec3b> Kinect::GetColourImage()const{
	return Colour_Image;
}
cv::Mat_<cv::Vec3b> Kinect::GetDepthImage()const{
	return Depth_Image;
}
cv::Mat_<double> Kinect::GetIntrinsicMatrix()const{
	return Intrinsic_Matrix;
}
int Kinect::GetKinectID()const{
	return Kinect_ID;
}
void Kinect::ProjectToReal(XnPoint3D& proj, XnPoint3D& real){
	Depth_Generator->ConvertProjectiveToRealWorld(1, &proj, &real);
}
void Kinect::RealToProject(XnPoint3D& real, XnPoint3D& proj){
	Depth_Generator->ConvertRealWorldToProjective(1, &real, &proj);
}
void Kinect::InitAllData(){

	// �f�v�X�̍��W���C���[�W�ɍ��킹��
	xn::AlternativeViewPointCapability viewPoint =
		Depth_Generator->GetAlternativeViewPointCap();
	//�r���[�|�C���g�@�\���擾����
	//�r���[�|�C���g���w�肵���m�[�h�ɍ��킹��
	viewPoint.SetViewPoint(*Image_Generator);
	//depthMD�쐬
	DepthMD = new xn::DepthMetaData();
	//imageMD�쐬
	ImageMD = new xn::ImageMetaData();
	//RGB�摜�̍쐬
	XnMapOutputMode outputmode;
	Image_Generator->GetMapOutputMode(outputmode);
	//Color�摜�̍쐬
	//Color_Image = cv::Mat_<cv::Vec3b>(Kinect::Height, Kinect::Width);
	//Depth�摜�̍쐬
	//Depth_Image = cv::Mat_<cv::Vec3b>(Kinect::Height, Kinect::Width);
	//IntrinsicMatrix
	//Intrinsic_Matrix = cv::Mat_<double>(3, 3);
	XnDouble pixel_size;
	unsigned long long F;
	Depth_Generator->GetIntProperty("ZPD", F);
	Depth_Generator->GetRealProperty("ZPPS", pixel_size);
	Intrinsic_Matrix = (cv::Mat_<double>(3, 3) << (double)F/(double)(2.0*pixel_size), 0.0, Kinect::Width/2.0,
													0.0, (double)F/(double)(2.0*pixel_size), Kinect::Height/2.0,
														0.0, 0.0, 1.0);
}

void Kinect::UpdateAllData(){
	//imageMetadata�쐬
	Image_Generator->GetMetaData(*ImageMD);
	//depthMetadata�쐬
	Depth_Generator->GetMetaData(*DepthMD);
	//color_image�쐬
	memcpy(Colour_Image.data, ImageMD->Data(), Colour_Image.step * Colour_Image.rows); 
	cv::cvtColor(Colour_Image, Colour_Image, CV_RGB2BGR);
}

void Kinect::CreateDepthImage(){
	
	//�f�v�X�̌X�����v�Z����(�A���S���Y����NiSimpleViewer.cpp�𗘗p)
	const int MAX_DEPTH = Depth_Generator->GetDeviceMaxDepth();
	//�f�v�X�̍ő�l�擾
	std::vector<float> depth_hist(MAX_DEPTH);

	unsigned int points = 0;
	//points �f�v�X���Ƃꂽ�S�_�̐�
	const XnDepthPixel* pDepth = DepthMD->Data();
	for (XnUInt y = 0; y < DepthMD->YRes(); ++y) {
		for (XnUInt x = 0; x < DepthMD->XRes(); ++x, ++pDepth) {
			if (*pDepth != 0) {
				//depth_hist[*pDepth]++;
				//std::cout << depth_data[y*Image_Size->width+x]<< std::endl;
				depth_hist[*pDepth]++;
				points++;
			}
			//�����f�v�X���Ƃꂽ��pDepth�̈ʒu�̃f�v�X�̒l�Ԗڂ̃q�X�g�O�������J�E���g
			//�f�v�X���Ƃꂽ�_�̐����J�E���g
		}
	}

	for (int i = 1; i < MAX_DEPTH; ++i) {
		depth_hist[i] += depth_hist[i-1];
		//�S���̃f�v�X�l�̃q�X�g�O�����̒l�ɂ��Ă�����l�����������̂̒l�����Z
		//����ɂ��f�v�X�̑傫�Ȓl�̃q�X�g�O�����͑傫�Ȓl�ɂȂ�
	}

	if ( points != 0) {
		for (int i = 1; i < MAX_DEPTH; ++i) {
			depth_hist[i] =
				(unsigned int)(256 * (1.0f - (depth_hist[i] / points)));
			//�f�v�X���傫�����̂قǏ������q�X�g�O���������蓖�Ă�
			//std::cout << depth_hist[i] << std::endl;
		}
	}

	for(int y=0; y < DepthMD->YRes(); y++){
		for(int x=0; x < DepthMD->XRes(); x++){
			Depth_Image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, depth_hist[(*DepthMD)(x, y)], depth_hist[(*DepthMD)(x, y)]); 
		}
	}
}

void Kinect::SavePointcloud(double range_near, double range_far){
	////�擾�����_�Q
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	////�e�_�̏��
	//pcl::PointXYZRGB points;
	XnPoint3D proj, real;
	for(int y=0; y<DepthMD->YRes(); y++){
		for(int x=0; x<DepthMD->XRes(); x++){
			double depth_tmp = (*DepthMD)(x, y);
			if(depth_tmp > range_near*1000 && depth_tmp < range_far*1000){ 
				//�摜�Ƌ����̑Ή����擾
				proj.X = x;
				proj.Y = y;
				proj.Z = depth_tmp;
				//���E���W�֕ϊ�
				Depth_Generator->ConvertProjectiveToRealWorld(1, &proj, &real);
				//points.x = (double)(real.X / 1000.0);
				//points.y = (double)(real.Y / 1000.0);
				//points.z = (double)(real.Z / 1000.0);
				//points.b = (unsigned char)Color_Image.at<cv::Vec3b>(y, x).val[0];
				//points.g = (unsigned char)Color_Image.at<cv::Vec3b>(y, x).val[1];
				//points.r = (unsigned char)Color_Image.at<cv::Vec3b>(y, x).val[2];
				////pointcloud�Ɋi�[
				//cloud.push_back(points);
			}
		}
	}
	//pcl::io::savePCDFile("input_points.pcd", cloud);
}

SingleKinect::SingleKinect(){
	//Context��Generator�̏�����
	Registration();
	//member�̏�����
	InitAllData();
}

SingleKinect::~SingleKinect(){
	delete Kinect_Context;
	Kinect_Context = 0;
}
void SingleKinect::UpdateContextAndData(){
	Kinect_Context->WaitAndUpdateAll();
	UpdateAllData();
}
void SingleKinect::ShowImage(){

	bool ShouldRun = true;
	char key;
	while( ShouldRun ){
		//�f�[�^�X�V��҂�
		Kinect_Context->WaitAndUpdateAll();
		UpdateAllData();
		//depthimage�쐬
		CreateDepthImage();
		//�J���[�摜�\��
		cv::imshow("colour_image",Colour_Image);
		key=cv::waitKey(1);
		cv::imshow("depth_image",Depth_Image);
		key=cv::waitKey(1);  
		if(key=='s' ){
			cv::imwrite("colour_image.jpg",Colour_Image);
			std::cout<<"save color_image"<<std::endl;
			//pointcloud�ۑ�
			SavePointcloud(0.0,10.0);
			std::cout<<"save pointcloud"<<std::endl;
			ShouldRun = false;
		}
	}
}

void SingleKinect::Registration(){
	// �R���e�L�X�g�̏�����
	Kinect_Context = new xn::Context();
	XnStatus rc = Kinect_Context->InitFromXmlFile("SamplesConfig.xml");
	if (rc != XN_STATUS_OK) {
		throw std::runtime_error(xnGetStatusString(rc));
	}

	// �C���[�W�W�F�l���[�^�̍쐬
	Image_Generator = new xn::ImageGenerator();
	//�w�肵���m�[�h�̃C���X�^���X���쐬����
	rc = Kinect_Context->FindExistingNode(XN_NODE_TYPE_IMAGE, *Image_Generator);
	if (rc != XN_STATUS_OK) {
		throw std::runtime_error(xnGetStatusString(rc));
	}

	// �f�v�X�W�F�l���[�^�̍쐬
	Depth_Generator = new xn::DepthGenerator();
	//�w�肵���m�[�h�̃C���X�^���X���쐬����
	rc = Kinect_Context->FindExistingNode(XN_NODE_TYPE_DEPTH, *Depth_Generator);
	if (rc != XN_STATUS_OK) {
		throw std::runtime_error(xnGetStatusString(rc));
	}

}
