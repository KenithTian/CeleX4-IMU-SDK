/************************************************************

FileName: CoordinateTransfer.h

Author: yueyin.zhou@celepixel.com	Date:2018/10/15

coordinate transformation functions   

Version: 0.2

Function List:
1. configureIntrinsic			configure intrinsic parameters of the camera and IMU
								eg:	coordinate_transfer::CoordinateTransfer ct;
									ct.configureIntrinsic(coordinate_transfer::Point2D(MAT_COLS / 2, MAT_ROWS / 2), kFocal_length);

2. updateWarpMatrix				update warp matrix by relative tilt/pan/roll angles
								eg:	coordinate_transfer::CoordinateTransfer ct;
									imu_proc::IMUProc imu;	//see more details in IMUProc to deal with imu data
									ct.updateWarpMatrix(imu.getCurTilt() - imu.getInitTilt(), imu.getCurPan() - imu.getInitPan(), imu.getCurRoll() - imu.getInitRoll());

3. transferSinglePixel			transfer each original pixel by pos angles(relative values) and output stabled pixel position

4. transferMatPixel				transfer a whole cv::Mat to stabled image by pos angles.

5. cameraConfig					load the config (calibraiton parameters) of camera.
								eg: std::string adr = "CameraCalibParams.ini";
								    coordinate_transfer::CoordinateTransfer cameraCeleX;
									cameraCeleX.cameraConfig(adr);

6. transferCameraPixelImage2Vehicle
								transfer a 2D point from image to X-Y vehicle coordinate
								cameraConfig() must be done
								eg:	coordinate_transfer::Point2D q_test;
									q_test.x = 289.;
									q_test.y = 369.;
									coordinate_transfer::Point2D p_test = cameraCeleX.transferCameraPixelImage2Vehicle(q_test);
									std::cout << "distance x = " << p_test.x << " y = " << p_test.y << std::endl;

7. transferCameraPixelVehicle2Image
								transfer a 2D point from X-Y vehicle coordinate back to image
								cameraConfig() must be done
								eg: coordinate_transfer::Point2D p_test;
									p_test.x = 50;
									p_test.y = 0;
									q_test = cameraCeleX.transferCameraPixelVehicle2Image(p_test);
									std::cout << "pixel u = " << q_test.x << " v = " << q_test.y << std::endl;

8. transferCameraMatImage2Vehicle
								transfer points from image to airscope image
								airscope image can be set by ratio_x_meter2pixel and ratio_y_meter2pixel,
								where x-direction is coloum direction and y represents row direction.
								An example, ratio_x_meter2pixel = 5.0 and ratio_y_meter2pixel = 50.0.
								eg: cv::Mat src_img = cv::imread("img.png");
									cv::Mat dst_img = cameraCeleX.transferCameraMatImage2Vehicle(src_img, 5, 50);
									cv::imshow("img",dst_img);
									cv::waitKey();
9. RadarConfig					read calibration params of Radar. To use it, refer to cameraConfig().

10.	transferRadarPositionRadar2Vehicle							
								transfer a position (Radar raw 2D point of an object) to X-Y vehicle coordinate
								To use it, refer to transferCameraPixelImage2Vehicle().

History:
ZHOU Yueyin    2018/10/15     0.1	build this module
He Qisheng	   2018/11/15     0.2   realize coordinate transform for camera and Radar
***********************************************************/

#ifndef COORDINATE_TRANSFER_H
#define COORDINATE_TRANSFER_H

#define CELEX_API_EXPORTS

#ifdef _WIN32
#ifdef CELEX_API_EXPORTS
#define CELEX_EXPORTS __declspec(dllexport)
#else
#define CELEX_EXPORTS __declspec(dllimport)
#endif
#else
#if defined(COORDINATETRANSFER_LIBRARY)
#define CELEX_EXPORTS
#else
#define CELEX_EXPORTS
#endif
#endif

#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/LU>
#include <fstream>
#include <string.h>
#include <sstream>

#include "INIParser.h"

namespace coordinate_transfer {

	const uint16_t kMatRows = 640;
	const uint16_t kMatCols = 768;

	struct Point3D 
	{
		Point3D()
			: x(0.0), y(0.0), z(0.0) {}
		Point3D(double x_ini, double y_ini, double z_ini)
			: x(x_ini), y(y_ini), z(z_ini) {}

		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};

	struct Point2D
	{
		Point2D()
			: x(0.0), y(0.0) {}
		Point2D(double x_ini, double y_ini)
			: x(x_ini), y(y_ini) {}

		double x = 0.0;
		double y = 0.0;
	};

	double stringToDouble(std::string num);

	class CELEX_EXPORTS CoordinateTransfer
	{
	public:
		struct CameraParam
		{
			// Intrinsic 
			coordinate_transfer::Point2D		center_point; // Base on camera coordinate system
			double		focal_length;
			double		focal_length_x;
			double		focal_length_y;
			double		Principal_x;
			double		Principal_y;
			double		dist_coeff_0;
			double		dist_coeff_1;
			double		dist_coeff_2;
			double		dist_coeff_3;
			double		dist_coeff_4;

			// Extrinsic params base on ISO coordinate system
			double		camera_translation_x;
			double		camera_translation_y;
			double		camera_translation_z;
			//double		camera_rotation_x;
			//double		camera_rotation_y;
			//double		camera_rotation_z;
			// double		camera_height;

			// Transfor matrix
			Eigen::Matrix3d ProjectionMatrixImage2Vehicle;// Base on ISO coordinate system
			Eigen::Matrix3d ProjectionMatrixVehicle2Image;
		};

		CameraParam CameraParamCeleX;

		CoordinateTransfer();
		~CoordinateTransfer();

		// coordinate transfer//////////////////////////////////////////////////////////////////////////
		
		// read calibration params of camera
		void cameraConfig(std::string camera_config_address); 

		// transfer a 2D point from image to X-Y vehicle coordinate
		coordinate_transfer::Point2D transferCameraPixelImage2Vehicle(coordinate_transfer::Point2D src_point);

		// transfer a 2D point from X-Y vehicle coordinate back to image
		coordinate_transfer::Point2D transferCameraPixelVehicle2Image(coordinate_transfer::Point2D src_point);

		// transfer points from image to airscope image
		cv::Mat transferCameraMatImage2Vehicle(cv::Mat src_mat, double ratio_x_meter2pixel, double ratio_y_meter2pixel); 
		//cv::Mat transferCameraMatVehicle2Image(cv::Mat src_mat);


		//IMU compensation//////////////////////////////////////////////////////////////////////////
		//first step: configure once
		void configureIntrinsic(coordinate_transfer::Point2D center_point, double focal_length);

		//second step: update for each frame when new pose generated
		bool updateWarpMatrix(const double tilt_x, const double pan_y, const double roll_z);

		//third step: use updated warp matrix to transfer each pixel
		coordinate_transfer::Point2D transferSinglePixel(int u_col, int v_row);
		//or third step: use updated warp matrix to transfer a whole image
		cv::Mat transferMatPixel(cv::Mat src_mat);

		//set&get//////////////////////////////////////////////////////////////////////////

	private:
		Eigen::MatrixXd getRotationWarpMatrix2D(double roll_z);
		Eigen::MatrixXd getTranslationWarpMatrix2D(double tilt_x, double pan_y);

		Eigen::MatrixXd warpMatrix_ = Eigen::MatrixXd::Identity(2, 3);
		double focal_length_;
		coordinate_transfer::Point2D center_point_;
	};	//class

	class CELEX_EXPORTS CoordinateTransferRadar
	{
	public:
		struct RadarParam
		{
			double		Radar_translation_x;// Base on ISO coordinate system
			double		Radar_translation_y;
			//double		Radar_translation_z;
			//double		Radar_rotation_x;
			//double		Radar_rotation_y;
			double		Radar_rotation_z;
			Eigen::Matrix2d ProjectionMatrixRadar2Vehicle;// Base on ISO coordinate system
		};

		RadarParam RadarParamConti;

		// read calibration params of Radar
		void RadarConfig(std::string Radar_config_address);

		// transfer a position (Radar raw 2D point of an object) to X-Y vehicle coordinate
		coordinate_transfer::Point2D transferRadarPositionRadar2Vehicle(coordinate_transfer::Point2D);

		// transfer a velocity (Radar raw data) to X-Y vehicle coordinate
		//coordinate_transfer::Point2D transferRadarVelocityRadar2Vehicle(coordinate_transfer::Point2D);

	private:

	};

}		//namespace
#endif	//COORDINATE_TRANSFER_H