#include <opencv2/opencv.hpp>
#include "Kinect.h"
#include "F30.h"

static void F30KinectCalibration(){ 

	F30 f30;
	SingleKinect kinect;
	
	cv::Mat_<double> depth_matrix=cv::Mat::zeros(Kinect::Height,Kinect::Width,CV_64F); 
	bool loopFlag=false;
	char key;
	int count=0;
	
	while(loopFlag){

		kinect.UpdateContextAndData();
		f30.Update();

		kinect.CreateDepthImage();
		f30.CreateThermalImage();

		cv::imshow("ThermalImage",f30.getImg());
		cv::imshow("KinectImage",kinect.GetColourImage());
		cv::imshow("depth_image",kinect.GetDepthImage());
		key = cv::waitKey(1);
		if(key=='s'){
			//ThermalImageの保存
			std::ostringstream PGRChessName;
			PGRChessName <<"./Calibration/PointGrey_0/chess"<< count << ".bmp";
			cv::imwrite(PGRChessName.str(), pgr.GetImage());
			std::cout << PGRChessName.str() << std::endl;
			//KinectImageの保存
			std::ostringstream KinectChessName;
			KinectChessName <<"./Calibration/Kinect_0/chess"<< count << ".bmp";
			cv::imwrite(KinectChessName.str(), kinect.GetColourImage());
			std::cout << KinectChessName.str() << std::endl;
			//depthの保存
			for(int y=0; y<Kinect::Height; y++){
				for(int x=0; x<Kinect::Width; x++){
					depth_matrix.at<double>(y, x) = (*kinect.GetDepthMD())(x, y);
				}
			}
			std::ostringstream filename;
			filename <<"./Calibration/Depth_0/depth"<< count << ".xml";
			cv::FileStorage cvfs(filename.str(), CV_STORAGE_WRITE);
			cv::write(cvfs, "depth_matrix",depth_matrix); 
			std::cout << filename.str() <<std::endl;
			count++;
		}
		if(key == 'q')
			loopFlag=false;
	}
	//pgr.Calibration();
		const int CHESS_ROWS = 10;
		const int CHESS_COLS = 7;
		const int CHESSBOARD_NUM = 20;
		const int NUM_OF_CAMERAS = 2;
		const int CAMERA_ID = 0;
		const bool DISPLAY_CHESS = true;
		int DetectedChessCount = 0;

		const cv::Size patternSize(CHESS_COLS, CHESS_ROWS);
		std::vector<cv::Point3f> Kinect_3Dpoints;			//3Dpoints from Kinect
		std::vector<cv::Point2f> PGR_2Dpoints;		//2Dpoints from PointGrey
		cv::Mat PGR_corners = cv::Mat();			//for calculating fundamental Mat
		cv::Mat Kinect_corners = cv::Mat();			//for calculating fundamental Mat

		std::cout << "detecting... "<<std::endl;
		for(int j=0; j<CHESSBOARD_NUM; j++){	
			//PGRCameraのコーナー検出
			std::vector<cv::Point2f>	PGRCorners;
			bool canFindPGRCorners;
			//Loadする名前を設定
			std::ostringstream PGRChessName;
			PGRChessName <<"./Calibration/PointGrey_" << CAMERA_ID <<"/chess"<< j << ".bmp";
			cv::Mat PGRChessImage_dist = cv::imread(PGRChessName.str(), 0);
			//undistort
			cv::Mat PGRChessImage = cv::Mat(PGRCamera::Height, PGRCamera::Width, CV_8UC1);
			cv::undistort(PGRChessImage_dist, PGRChessImage, pgr.GetIntrinsicMatrix(), pgr.GetDistortion());
			//Chessboardのコーナー検出
			canFindPGRCorners = cv::findChessboardCorners(
				PGRChessImage, 
				patternSize, 
				PGRCorners,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
			//Kinectのコーナー検出
			std::vector<cv::Point2f>	KinectCorners;
			bool canFindKinectCorners;
			//Loadする名前を設定
			std::ostringstream KinectChessName;
			KinectChessName <<"./Calibration/Kinect_" << CAMERA_ID << "/chess"<< j << ".bmp";
			cv::Mat KinectChessImage = cv::imread(KinectChessName.str(), 0);
			//Chessboardのコーナー検出
			canFindKinectCorners = cv::findChessboardCorners(
				KinectChessImage, 
				patternSize, 
				KinectCorners,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
			//KinectとPointGreyがどちらも検出できた場合
			if(canFindPGRCorners && canFindKinectCorners){
				//PointGreyCameraから検出した点をサブピクセル単位に変換
				cv::cornerSubPix(
					PGRChessImage,
					PGRCorners,
					cv::Size(11, 11),
					cv::Size(-1, -1),
					cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01) );

				//Kinectから検出した点をサブピクセル単位に変換
				cv::cornerSubPix(
					KinectChessImage,
					KinectCorners,
					cv::Size(11, 11),
					cv::Size(-1, -1),
					cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01) );
				//補正された点をpush_back
				//PGRの点
				cv::Point2f pgr_point;
				std::vector<cv::Point2f>::iterator PGRCorners_itr = PGRCorners.begin();
				//Kienctで取得した3次元点
				XnPoint3D proj, real;
				cv::Point3f point_3d;
				std::vector<cv::Point2f>::iterator KinectCorners_itr = KinectCorners.begin();
				//ファイルからdepthのデータを読み込む
				cv::Mat_<double> depth_matrix = cv::Mat::zeros(Kinect::Height, Kinect::Width, CV_64F);
				std::ostringstream depthfile;
				depthfile << "./Calibration/Depth_" << CAMERA_ID<< "/depth"<< j << ".xml";
				cv::FileStorage cvfs(depthfile.str(), CV_STORAGE_READ);
				cv::FileNode node(cvfs.fs, NULL);
				cv::read(node["depth_matrix"], depth_matrix);
				
				while( PGRCorners_itr != PGRCorners.end() && KinectCorners_itr != KinectCorners.end()){

					//Kinect画像と距離の対応を取得
					proj.X = (XnFloat)KinectCorners_itr->x;
					proj.Y = (XnFloat)KinectCorners_itr->y;
					proj.Z = (XnFloat)depth_matrix.at<double>((int)proj.Y, (int)proj.X);
					//ノイズ除去
					if(proj.Z > 100){
						//世界座標に変換
						kinect.ProjectToReal(proj, real);
						point_3d.x = real.X;
						point_3d.y = -real.Y;//opencv coordinate
						point_3d.z = real.Z;
						//Kinectの3Dpointを格納
						Kinect_3Dpoints.push_back(point_3d);
						//Kinectの2Dpointを格納
						cv::Mat kinect_2d = (cv::Mat_<double>(1, 2) << (double)KinectCorners_itr->x*2.0, (double)KinectCorners_itr->y*2.0);
						Kinect_corners.push_back(kinect_2d);
						//PGRの2Dpointを格納
						pgr_point.x = PGRCorners_itr->x;
						pgr_point.y = PGRCorners_itr->y;
						PGR_2Dpoints.push_back(pgr_point);
						cv::Mat pgr_2d = (cv::Mat_<double>(1, 2) << (double)PGRCorners_itr->x, (double)PGRCorners_itr->y);
						PGR_corners.push_back(pgr_2d);
								}
					KinectCorners_itr++;
					PGRCorners_itr++;
				}
				//どちらも検出できたペアを数える
				DetectedChessCount++;
			}
		}

		std::cout << "Kinect_3Dpoints_size: "<< Kinect_3Dpoints.size() << std::endl;
		std::cout << "PGR_2Dpoints_size: "<< PGR_2Dpoints.size() << std::endl;
		std::cout << "calibrating..." << std::endl;

		cv::Mat rotation_matrix;		//solvePnPで得られるRotation
		cv::Mat translation_matrix;		//solvePnPで得られるTranslation
		cv::Mat distortion_coeffs =
			(cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 0.0);
		cv::Mat param1_(Kinect_3Dpoints, false);
		cv::Mat param2_(PGR_2Dpoints, false);
		//3D-2D対応から物体の姿勢を求める
		cv::solvePnPRansac(param1_, param2_,  pgr.GetIntrinsicMatrix(),
			distortion_coeffs, rotation_matrix, translation_matrix,
			false, Kinect_3Dpoints.size() * 3, 2.0,
			(int)(Kinect_3Dpoints.size() * 0.9), cv::noArray(), CV_ITERATIVE);
		//得られたMatrixで再投影
		std::vector<cv::Point2f> outp;
		cv::projectPoints(param1_, rotation_matrix, translation_matrix,
			 pgr.GetIntrinsicMatrix(), distortion_coeffs, outp);
		//再投影
		float sum = 0.0f;
		for (int i = 0; i < outp.size(); ++i) {
			cv::Point2f vec = outp[i] - PGR_2Dpoints[i];
			sum += sqrtf(vec.x * vec.x + vec.y * vec.y);
		}
		if (outp.size() > 0)
			std::cout << "Reprojection error : " << sum / outp.size()
			<< std::endl;

		//回転行列
		cv::Mat rodrigues;
		//回転ベクトルから回転行列に変換
		cv::Rodrigues(rotation_matrix, rodrigues);
		//calculate fundamental Mat
		cv::Mat fundamental_matrix = cv::findFundamentalMat(Kinect_corners, PGR_corners);
		//ファイル出力
		std::ostringstream filename;
		filename << "./Calibration/PGRKinect_" << CAMERA_ID << ".xml";
		cv::FileStorage	cvfs(filename.str(), CV_STORAGE_WRITE);
		
		cv::write(cvfs, "rotation", rodrigues);
		cv::write(cvfs, "translation", translation_matrix);
		cv::write(cvfs, "f_matrix", fundamental_matrix);
		std::cout << filename.str() << std::endl;
}
