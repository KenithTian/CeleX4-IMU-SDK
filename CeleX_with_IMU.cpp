// CeleX_with_IMU.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"


/*
* Copyright (c) 2017-2018 CelePixel Technology Co. Ltd. All Rights Reserved
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>

#include <chrono>
#include "IMUProc.h"
#include "CoordinateTransfer.h"
#include "INIParser.h"

#include "celex4/celex4.h"
#include "celex4/celex4datamanager.h"
#include "celex4/celex4processeddata.h"

#ifdef _WIN32
#include <windows.h>
#else
#include<unistd.h>
#endif


using namespace std;

#define MAT_ROWS 640
#define MAT_COLS 768
#define PIXELS_NUMBER 768 * 640
#define FPN_PATH    "D:/3.Data/FPN.txt"		// define your own FPN file
#define INI_FILE	"conf.ini"

static std::vector<EventData> last_frame;
static std::vector<EventData> current_frame;


/*camera calibration*/
double kFocal_length = 937.66;
/*double kFocal_length = 1277;*/
// const double kPrincipal_x = 378.35;
// const double kPrincipal_y = 340.86;
const double kPrincipal_x = MAT_COLS/2;
const double kPrincipal_y = MAT_ROWS/2;

/*frame time*/
const double kPeriod_last = 60.0;	//60ms
const int kTick_number = 750000;	//75w帧一个周期
const double kTime_per_tick = kPeriod_last / kTick_number; //每个计数时长

double timestamp_event = 0.0;
double timestamp_imu = 0.0;
imu_proc::IMUProc imu;
coordinate_transfer::CoordinateTransfer ct;
util::INIParser iniParser;

class SensorDataObserver : public CeleX4DataManager
{
public:
	SensorDataObserver(CX4SensorDataServer* pServer)
	{
		m_pServer = pServer;
		m_pServer->registerData(this, CeleX4DataManager::CeleX_Frame_Data);
	}
	~SensorDataObserver()
	{
		m_pServer->registerData(this, CeleX4DataManager::CeleX_Frame_Data);
	}
	virtual void onFrameDataUpdated(CeleX4ProcessedData* pSensorData);//overrides Observer operation

	CX4SensorDataServer* m_pServer;
};

void SensorDataObserver::onFrameDataUpdated(CeleX4ProcessedData* pSensorData)
{
	if (NULL == pSensorData)
		return;
	emSensorMode sensorMode = pSensorData->getSensorMode(); //FullPic_Event_Mode or FullPictureMode

	if (EventMode == sensorMode)
	{
		//event
		cv::Mat mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
		cv::Mat mat_stable = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
		cv::Mat mat_test = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
		uint64_t frameNo = 0;
		std::vector<EventData> v = pSensorData->getEventDataVector();	//vector to store the event data
		std::vector<IMUData> data = pSensorData->getIMUDataVector();

		for (int i = 0; i < v.size() - 1; ++i)
		{
			mat.at<uchar>(MAT_ROWS - v[i].row - 1, v[i].col) = v[i].brightness;
		}
		timestamp_event = frameNo * kPeriod_last + v.back().t * kTime_per_tick;

		double closest_timestamp = timestamp_event;
		double delta_time = 999999.9;

		for (auto it_data = data.begin(); it_data != data.end(); it_data++) {
			timestamp_imu = it_data->frameNo * kPeriod_last + it_data->t_GYROS * kTime_per_tick;

			imu.updateTiltPanRoll6Axis(
				uint64_t(timestamp_imu),
				it_data->x_ACC,
				it_data->y_ACC,
				it_data->z_ACC,
				it_data->x_GYROS,
				it_data->y_GYROS,
				it_data->z_GYROS);
// 			imu.updateTiltPanRoll9Axis(
// 				uint64_t(timestamp_imu),
// 				it_data->x_ACC,
// 				it_data->y_ACC,
// 				it_data->z_ACC,
// 				it_data->x_GYROS,
// 				it_data->y_GYROS,
// 				it_data->z_GYROS,
// 				it_data->x_MAG,
// 				it_data->y_MAG,
// 				it_data->z_MAG);


			if (fabs(timestamp_event - timestamp_imu) < delta_time) {
				delta_time = fabs(timestamp_event - timestamp_imu);
				closest_timestamp = timestamp_imu;

				imu_proc::IMUData imu_data;
				imu.getIMUData(imu_data);
				std::cout << "current_tilt = " << imu_data.cur_tilt << ",current_pan = " << imu_data.cur_pan << ",current_roll = " << imu_data.cur_roll << std::endl;

				//for hand holding
				ct.updateWarpMatrix(imu_data.cur_tilt - imu_data.init_tilt, imu_data.cur_pan - imu_data.init_pan, imu_data.cur_roll - imu_data.init_roll);

				// for real car
				/*ct.updateWarpMatrix(imu_data.cur_tilt - imu_data.dynamic_tilt_base, 0.0, imu_data.cur_roll - imu_data.dynamic_roll_base);*/
			}
		}

		//std::cout << "time between event and imu is: " << closest_timestamp - timestamp_event << " ms" << std::endl;
		mat_stable = ct.transferMatPixel(mat);
	
		cv::imshow("show", mat);
		cv::imshow("show_stabled", mat_stable);
		cv::waitKey(10);
	}
}


int main()
{
	CeleX4 *pCelex = new CeleX4;
	if (NULL == pCelex)
		return 0;

	iniParser.ReadINI(INI_FILE);
	std::string str_imu_calib_file = iniParser.GetValue("calibration", "imu_calib_file");
	std::string str_camera_calib_file = iniParser.GetValue("calibration", "camera_calib_file");
	util::INIParser iniCameraParser;
	iniCameraParser.ReadINI(str_camera_calib_file);
	// configure imu calibration parameters
	imu.configIMUCalib(str_imu_calib_file);

	// configure application mode: 0->real car mode; 1->hand hold mode.
	imu.setAppMode(std::stoi(iniParser.GetValue("general", "app_mode")));

	// read focal length calibration parameter or configure by default.
	kFocal_length = std::stod(iniCameraParser.GetValue("Intrinsic parameters", "kFocal_length"));


	pCelex->openSensor("");
	pCelex->setFpnFile(FPN_PATH);
	pCelex->setSensorMode(EventMode); //EventMode, FullPic_Event_Mode and FullPictureMode
	pCelex->setThreshold(std::stoi(iniParser.GetValue("general", "threshold")));


	//imu.setAutoUpdateStatus(true);
	ct.configureIntrinsic(coordinate_transfer::Point2D(MAT_COLS / 2, MAT_ROWS / 2), kFocal_length);

	SensorDataObserver* pSensorData = new SensorDataObserver(pCelex->getSensorDataServer());
	while (true)
	{
		pCelex->pipeOutFPGAData();

#ifdef _WIN32
		Sleep(10);
#else
		usleep(1000 * 10);
#endif

	}
	return 1;
}