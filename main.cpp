/** @mainpage Eye position tracker documentation

 @author Yuta Itoh <itoh@in.tum.de>, \n<a href="http://wwwnavab.in.tum.de/Main/YutaItoh">Homepage</a>.


TO DO:
Double-check zeroing works correctly (it should)
Fixed eyeCamRotation() output, check if correct
Check Roll Angle at each state to make sure they're all either between 0 and 90 or 0 and -90
**/


#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>

#include "ubitrack_util.h" // claibration file handlers
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread.hpp>

#include <opencv2/opencv.hpp> /////
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/ml.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/photo/photo.hpp>

#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include "pupilFitter.h" // 2D pupil detector

#include "timer.h"

#include "eye_model_updater.h" // 3D model builder
#include "eye_cameras.h" // Camera interfaces

#include <thread>
#include <mutex>

#define M_PI 3.14159  /* pi */
double deg2rad = M_PI / 180;
double rad2deg = 180 / M_PI;

// Bool values for debugging
bool tempRS = true;		//Temp coordinates for RealSense (Center)
bool orderCrop = true;	//Crop eyecam or use full image
bool streamcam = true;	//Using IP stream as eyecam (Not working, keep as true) Not implemented

// Minimum Bounding Box Side Pixel Size
int minBoxSize = 30;


// Eyecam Resolution and Crop Settings
int eyeCamResolution = 960;
int cropXstart = eyeCamResolution / 4, cropYstart = 1.5*eyeCamResolution / 5;
int cropWidth = eyeCamResolution - (eyeCamResolution / 3.5), cropHeight = eyeCamResolution - (eyeCamResolution / 2.5);
int croppedAspectRatio = cropHeight / cropWidth;


// Depth cam resolution settings
int rsResolutionVert = 480, rsResolutionHor = 640, rsFOVvert = 42, rsFOVhor = 69;
double focalHor = (0.5 * rsResolutionHor) / tan(0.5 * rsFOVhor * deg2rad);
double focalVert = (0.5 * rsResolutionVert) / tan(0.5 * rsFOVvert * deg2rad);
int centerVert = rsResolutionVert / 2, venterHor = rsResolutionHor / 2;


//Variable for pulling uncalibrated gaze vector
Eigen::Matrix<double, 3, 1> directionCalc;


//--------RealSense Initializers Start--------
Mat RSimage;
Mat imgHSV;
Mat OutputImage;


int iLowH = 0;
int iHighH = 12;
int iLowS = 60;
int iHighS = 255;
int iLowV = 0;
int iHighV = 245;

int acc = 1;
int rows = 10;
int para1 = 5;
int para2 = 10;
int minRad = 5;
int maxRad = 70;

int x;
int y;

int rsResX = 640;
int rsResY = 480;

int input;
//--------RealSense Initializers End--------

// Object Recognition values
int threshold_value = 127;
int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_binary_value = 255;


// Extra initializers
int calibState = 0;		// 0 for startup, 1 until you have a satisfactory calibration snapshot, 2 and onwards for running object detection
bool nextstate;
int focusPixelX, focusPixelY;
double angleHor, angleVert;
int rsCenterX = rsResX / 2;
int rsCenterY = rsResY / 2;
double angleOffsetHor = 0;
double angleOffsetVert = 0;
double angleOffsetRoll = 0;

double RollTemp1 = 0;
double RollTemp2 = 0;
double RollTemp3 = 0;
double RollTemp4 = 0;

vector<double> scaleAngles(8, 0.0);
bool is_reliable_global;

double scale_angleHorR, scale_angleHorL, scale_angleVertU, scale_angleVertD;
double xFocus, yFocus;

//Testing variables
int testx = 1;
int testy = 1;
bool nexttest, goodfit;
int testcount = 1;

// Calculate Roll Angle
double calcRoll(double Hor, double Vert)/*vector<double> origin, vector<double> direction*///Output in Radians
{

	vector<double> dir(3), direction(2);
	direction[0] = (-Hor) * deg2rad + M_PI / 2;
	direction[1] = (-Vert) * deg2rad + M_PI / 2;

	// Convert angles to cartesian coordinate
	dir[0] = sin(direction[1]) * cos(direction[0]);
	dir[1] = sin(direction[1]) * sin(direction[0]);
	dir[2] = cos(direction[1]);
	///*cout << endl << endl << "INPUT: " << endl <<
	//	"Horizontal Angle: " << direction[0] * rad2deg << endl <<
	//	"Vertical Angle: " << direction[1] * rad2deg << endl <<
	//	"Roll x: " << dir[0] << endl <<
	//	"Roll y: " << dir[1] << endl <<
	//	"Roll z: " << dir[2] << endl;*/

	double outp = atan2(dir[2], dir[0]) * rad2deg;

	return outp; //Output in Degrees
}

// Math for rotating gaze vector to match depth cam direction
vector<double> eyeCamRotation(vector<double> direction)/*vector<double> origin, vector<double> direction*///Output in Radians
{

	vector<double> vec(3), dir(3), outp(2);

	// Convert angles to cartesian coordinate
	dir[0] = sin(direction[1]) * cos(direction[0]);
	dir[1] = sin(direction[1]) * sin(direction[0]);
	dir[2] = cos(direction[1]);

	// y-axis rotation
	vec[0] = dir[0] * cos(angleOffsetRoll * deg2rad) + dir[2] * sin(angleOffsetRoll * deg2rad);
	vec[1] = dir[1];
	vec[2] = dir[2] * cos(angleOffsetRoll * deg2rad) - dir[0] * sin(angleOffsetRoll * deg2rad);



	//Vertical Angle
	outp[1] = acos(vec[2] / sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2)));


	//Horizontal angle
	outp[0] = 0;
	if (vec[0] > 0)
	{
		outp[0] = atan(vec[1] / vec[0]);
	}
	else if (vec[0] < 0 && vec[1] >= 0)
	{
		outp[0] = atan(vec[1] / vec[0]) + M_PI;
	}
	else if (vec[0] < 0 && vec[1] < 0)
	{
		outp[0] = atan(vec[1] / vec[0]) - M_PI;
	}
	else if (vec[0] == 0 && vec[1] > 0)
	{
		outp[0] = M_PI / 2;
	}
	else if (vec[0] == 0 && vec[1] < 0)
	{
		outp[0] = -M_PI / 2;
	}

	if (outp[0] > M_PI)
	{
		outp[0] = -(2 * M_PI) + outp[0];
	}

	//Show coordinates when not calibrating z-rotation (makes it possible to see state changes in the cout)
	//if (nextstate == false)
	//{
		//cout << endl << endl << "EYECAM ROTATION: " << endl <<
			//	"Horizontal Angle: " << direction[0] * rad2deg << endl <<
			//	"Vertical Angle: " << direction[1] * rad2deg << endl <<
			//	//"Horizontal Angle Zero: " << direction[0] * rad2deg + angleOffsetHor << endl <<
			//	//"Vertical Angle Zero: " << direction[1] * rad2deg + angleOffsetVert << endl <<
			//"Pre x: " << dir[0] << endl <<
			//"Pre y: " << dir[1] << endl <<
			//"Pre z: " << dir[2] << endl <<
			//"Post x: " << vec[0] << endl <<
			//"Post y: " << vec[1] << endl <<
			//"Post z: " << vec[2] << endl << endl;
		//"Post Rot Hor: " << outp[0] * rad2deg << endl <<
		//"Post Rot Vert: " << outp[1] * rad2deg << endl
	//}

	return outp; //Output in Radians
}

// Converting a 3D point to spherical coordinates
// Straight ahead is along the y-axis: (0.0, 1.0, 0.0)
// This is done so the spherical coordinates don't revolve around the "Straight Ahead"-axis when doing the calculations
vector<double> pointToAngles(Eigen::Matrix<double, 3, 1> direction)
{
	vector<double> dir(3), angles(2);
	dir[0] = -direction(0, 0); dir[1] = -direction(2, 0); dir[2] = -direction(1, 0);
	if (dir[1] < 0)
	{
		dir[1] = -dir[1];
	}
	double calcangleVert = 0;
	double calcangleHor = 0;
	// cout input values
	//if (calibState < 4 || calibState == 11)
	//{
	//	cout << endl << endl << "INPUT: " << endl <<
	//		"x: " << dir[0] << endl <<
	//		"y: " << dir[1] << endl <<
	//		"z: " << dir[2] << endl
	//		/* << "Magnitude: " << sqrt(pow(dir[0], 2) + pow(dir[1], 2) + pow(dir[2], 2))*/;
	//}

	//Vertical Angle
	calcangleVert = acos(dir[2]/sqrt(pow(dir[0], 2) + pow(dir[1], 2) + pow(dir[2], 2)));


	//Horizontal angle
	calcangleHor = 0;
	if (dir[0] > 0)
	{
		calcangleHor = atan(dir[1] / dir[0]);
	}
	else if (dir[0] < 0 && dir[1] >= 0)
	{
		calcangleHor = atan(dir[1] / dir[0]) + M_PI;
	}
	else if (dir[0] < 0 && dir[1] < 0)
	{
		calcangleHor = atan(dir[1] / dir[0]) - M_PI;
	}
	else if (dir[0] == 0 && dir[1] > 0)
	{
		calcangleHor = M_PI / 2;
	}
	else if (dir[0] == 0 && dir[1] < 0)
	{
		calcangleHor = -M_PI / 2;
	}

	// Offset by measured values
	angles[0] = calcangleHor + angleOffsetHor * deg2rad;
	angles[1] = calcangleVert + angleOffsetVert * deg2rad;

	// Real z-axis rotation (Rotating around y-axis because of flips done earlier)
	angles = eyeCamRotation(angles);


	//Output Offset To Zero
	angles[0] = -((angles[0] - M_PI / 2) * rad2deg);
	angles[1] = -((angles[1] - M_PI / 2) * rad2deg);

	if (calibState == 11)
	{
		if (angles[0] > 0 && angles[1] > 0)
		{
			angles[0] = angles[0] * (rsFOVhor / 2) / scaleAngles[0];
			angles[1] = angles[1] * (rsFOVvert / 2) / scaleAngles[1];
		}
		else if (angles[0] < 0 && angles[1] > 0)
		{
			angles[0] = angles[0] * (-rsFOVhor / 2) / scaleAngles[2];
			angles[1] = angles[1] * (rsFOVvert / 2) / scaleAngles[3];
		}
		else if (angles[0] > 0 && angles[1] < 0)
		{
			angles[0] = angles[0] * (rsFOVhor / 2) / scaleAngles[4];
			angles[1] = angles[1] * (-rsFOVvert / 2) / scaleAngles[5];
		}
		else if (angles[0] < 0 && angles[1] < 0)
		{
			angles[0] = angles[0] * (-rsFOVhor / 2) / scaleAngles[6];
			angles[1] = angles[1] * (-rsFOVvert / 2) / scaleAngles[7];
		}
	}



	//if (calibState < 4 || calibState == 11)
	//{
	//	cout << endl << "OUTPUT" << endl <<
	//		"Pre Rot Hor: " << calcangleHor * rad2deg << endl <<
	//		//"Offset Hor: " << angleOffsetHor << endl <<
	//		"Final Hor: " << angles[0] << endl <<
	//		"Pre Rot Vert: " << calcangleVert * rad2deg << endl <<
	//		//"Offset Vert: " << angleOffsetVert << endl <<
	//		"Final Vert: " << angles[1] << endl << 
	//		endl;
	//}
	//if (calibState == 11)
	//{
	//	cout << endl << "ROT OFFSETS:" << endl <<
	//		"1: " << RollTemp1 << endl <<
	//		"2: " << RollTemp2 << endl <<
	//		"3: " << RollTemp3 << endl <<
	//		"4: " << RollTemp4 << endl << endl;
	//}

	return angles; //Output in Degrees
}


// Converting pixel positions to a horizontal and vertical angle, given known FOV and resolution
vector<double> pixel2angles(double x, double y)
{
	double hor, vert;
	vector<double> outp(2);

	hor = -(atan2(focalHor, (x - rsResX / 2)) * rad2deg - 90);
	vert = atan2(focalVert, (y - rsResY / 2)) * rad2deg - 90;

	outp[0] = hor;
	outp[1] = vert;
	return outp;
}


// Converting horizontal and vertical angles to a pixel position, given known FOV and resolution
vector<int> angle2pixel(double calcangleHor, double calcangleVert)
{
	int hor, vert;
	vector<int> outp(2);

	hor = (rsResX / 2) + (tan(calcangleHor * deg2rad) * focalHor);
	if (hor >= rsResX - 5)
	{
		hor = rsResX - 5;
	}
	else if (hor < 5)
	{
		hor = 5;
	}

	//cout << "angleHor: " << angleHor << endl;
	//cout << "Hor calc: " << tan(angleHor * deg2rad) * focalHor << endl;

	vert = (rsResY / 2) - (tan(calcangleVert * deg2rad) * focalVert); // Minus because a higher the y-coordinate means a lower pixel position ((0,0) in upper left)
	if (vert >= rsResY - 5)
	{
		vert = rsResY - 5;
	}
	else if (vert < 5)
	{
		vert = 5;
	}

	outp[0] = hor;
	outp[1] = vert;
	return outp;
}



static void HSVthreshold(int, int, int, int, int, int, void*)
{
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), OutputImage);
}

 
namespace {

enum InputMode { CAMERA, CAMERA_MONO, VIDEO, IMAGE };

}


int main(int argc, char* argv[]) {
	// RealSense Setup
	 // Contructing piplines and other stuff to receive data from the realsense camera.

	//Contruct a pipeline which abstracts the device
	rs2::pipeline pipe;     //for color

	//Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;



	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_COLOR, rsResX, rsResY, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, rsResX, rsResY, RS2_FORMAT_Z16, 30);

	//Instruct pipeline to start streaming with the requested configuration
	rs2::pipeline_profile selection = pipe.start(cfg);    //for color

	 // Camera warmup - dropping several first frames to let auto-exposure stabilize
	rs2::frameset frames;

	for (int i = 0; i < 30; i++)
	{
		//Wait for all configured streams to produce a frame
		frames = pipe.wait_for_frames();
	}


	//End of RealSense Setup


	// Variables for FPS
	eye_tracker::FrameRateCounter frame_rate_counter;

	bool kVisualization = false;
	kVisualization = true;

	InputMode input_mode =
		// InputMode::VIDEO;  // Set a video as a video source
		// InputMode::CAMERA; // Set two cameras as video sources
		 InputMode::CAMERA_MONO; // Set a camera as video sources
		// InputMode::IMAGE;// Set an image as a video source


	////// Command line opitions /////////////
	std::string kDir = "C:/Users/Lucas/Pictures";
	std::string media_file;
	std::string media_file_stem;
	//std::string kOutputDataDirectory(kDir + "out/");	// Data output directroy
	if (argc > 2) {
		boost::filesystem::path file_name = "Eye3.mp4";//std::string(argv[2]);
		//kDir = std::string(argv[1]);
		media_file_stem = file_name.stem().string();
		media_file = kDir + file_name.string();
		//kOutputDataDirectory = kDir + "./";
		std::cout << "Load " << media_file << std::endl;
		std::string media_file_ext = file_name.extension().string();

		if (media_file_ext == ".avi" ||
			media_file_ext == ".mp4" ||
			media_file_ext == ".wmv") {
			input_mode = InputMode::VIDEO;
		}
		else {
			input_mode = InputMode::IMAGE;
		}
	}
	else {
		if (input_mode == InputMode::IMAGE || input_mode == InputMode::VIDEO) {
			switch (input_mode)
			{
			case InputMode::IMAGE:
				media_file = kDir + "data3/test.png";
				media_file_stem = "test";
				break;
			case InputMode::VIDEO:
				media_file = kDir + "/Eye3.mp4";// "out/test.avi";
				media_file_stem = "test";
				break;
			default:
				break;
			}
		}
	}
	///////////////


	//// Camera intrinsic parameters
	std::string calib_path = "../../docs/cameraintrinsics_eye.txt";
	eye_tracker::UbitrackTextReader<eye_tracker::Caib> ubitrack_calib_text_reader;
	if (ubitrack_calib_text_reader.read(calib_path) == false) {
		std::cout << "Calibration file onpen error: " << calib_path << std::endl;
		return -1;
	}
	cv::Mat K; // Camera intrinsic matrix in OpenCV format
	cv::Vec<double, 8> distCoeffs; // (k1 k2 p1 p2 [k3 [k4 k5 k6]]) // k: radial, p: tangential
	ubitrack_calib_text_reader.data_.get_parameters_opencv_default(K, distCoeffs);

	// Focal distance used in the 3D eye model fitter
	double focal_length = (K.at<double>(0, 0) + K.at<double>(1, 1)) * 0.5; //  Required for the 3D model fitting


	// Set mode parameters
	size_t kCameraNums;
	switch (input_mode)
	{
	case InputMode::IMAGE:
	case InputMode::VIDEO:
	case InputMode::CAMERA_MONO:
		kCameraNums = 1;
		break;
	case InputMode::CAMERA:
		kCameraNums = 2;
		break;
	default:
		break;
	}


	// Setup of classes that handle monocular/stereo camera setups
	// We can encapslate them into a wrapper class in future update
	std::vector<std::unique_ptr<eye_tracker::EyeCameraParent>> eyecams(kCameraNums);                 // Image sources
	std::vector<std::unique_ptr<eye_tracker::CameraUndistorter>> camera_undistorters(kCameraNums); // Camera undistorters
	std::vector<std::string> window_names(kCameraNums);                                            // Window names
	std::vector<cv::Mat> images(kCameraNums);                                                      // buffer images
	std::vector<std::string> file_stems(kCameraNums);                                              // Output file stem names
	std::vector<int> camera_indices(kCameraNums);                                                  // Camera indices for Opencv capture
	std::vector<std::unique_ptr<eye_tracker::EyeModelUpdater>> eye_model_updaters(kCameraNums);    // 3D eye models

	// Instantiate and initialize the class vectors
	try {
		switch (input_mode)
		{
		case InputMode::IMAGE:
			eyecams[0] = std::make_unique<eye_tracker::EyeCamera>(media_file, false);
			eye_model_updaters[0] = std::make_unique<eye_tracker::EyeModelUpdater>(focal_length, 5, 0.5);
			camera_undistorters[0] = std::make_unique<eye_tracker::CameraUndistorter>(K, distCoeffs);
			window_names = { "Video/Image" };
			file_stems = { media_file_stem };
			break;
		case InputMode::VIDEO:
			eyecams[0] = std::make_unique<eye_tracker::EyeCamera>(media_file, false);
			eye_model_updaters[0] = std::make_unique<eye_tracker::EyeModelUpdater>(focal_length, 5, 0.5);
			camera_undistorters[0] = std::make_unique<eye_tracker::CameraUndistorter>(K, distCoeffs);
			window_names = { "Video/Image" };
			file_stems = { media_file_stem };
			break;
		case InputMode::CAMERA:
			camera_indices[0] = 0;
			camera_indices[1] = 2;
#if 0
			// OpenCV HighGUI frame grabber
			eyecams[0] = std::make_unique<eye_tracker::EyeCamera>(camera_indices[0], false);
			eyecams[1] = std::make_unique<eye_tracker::EyeCamera>(camera_indices[1], false);
#else
			// DirectShow frame grabber
			eyecams[0] = std::make_unique<eye_tracker::EyeCameraDS>("Pupil Cam1 ID0");
			eyecams[1] = std::make_unique<eye_tracker::EyeCameraDS>("Pupil Cam1 ID2");
#endif
			eye_model_updaters[0] = std::make_unique<eye_tracker::EyeModelUpdater>(focal_length, 5, 0.5);
			eye_model_updaters[1] = std::make_unique<eye_tracker::EyeModelUpdater>(focal_length, 5, 0.5);
			camera_undistorters[0] = std::make_unique<eye_tracker::CameraUndistorter>(K, distCoeffs);
			camera_undistorters[1] = std::make_unique<eye_tracker::CameraUndistorter>(K, distCoeffs);
			window_names = { "Cam0", "Cam1" };
			file_stems = { "cam0", "cam1" };
			break;
		case InputMode::CAMERA_MONO:
			eyecams[0] = std::make_unique<eye_tracker::EyeCameraDS>("USB_Camera"); //
			eye_model_updaters[0] = std::make_unique<eye_tracker::EyeModelUpdater>(focal_length, 5, 0.5);
			camera_undistorters[0] = std::make_unique<eye_tracker::CameraUndistorter>(K, distCoeffs);
			window_names = { "Cam0" };
			file_stems = { "cam0" };
			break;
		default:
			break;
		}
	}
	catch (char* c) {
		std::cout << "Exception: ";
		std::cout << c << std::endl;
		return 0;
	}


	////////////////////////
	// 2D pupil detector
	PupilFitter pupilFitter;
	pupilFitter.setDebug(false);
	/////////////////////////

	// Main loop
	if (streamcam)
		cout << "Finding Stream..." << endl;

	VideoCapture noir1("http://192.168.123.23:8081");
	//noir1.set(CV_CAP_PROP_BUFFERSIZE, 1);
	Mat noir1frame, noir1preframe;
	Rect myROI(cropXstart, cropYstart, cropWidth, cropHeight);
	bool findEye = false, buildEye = false, isStream = true;
	
	const char kTerminate = 27;//Escape 0x1b
	bool is_run = true;

	if (!noir1.isOpened() && streamcam)
		return -1;

	while (is_run) {
		while (noir1.isOpened())
		{
			if (streamcam)
			{
				noir1 >> noir1frame;
				if (noir1frame.empty()) break;
			}

			// Fetch key input
			char kKEY = 0;
			if (kVisualization) {
				kKEY = cv::waitKey(1);
			}
			switch (kKEY) {
			case kTerminate:
				is_run = false;
				break;
			}

			if (!streamcam)
			{
				// Fetch images
				for (size_t cam = 0; cam < kCameraNums; cam++) {
					eyecams[cam]->fetchFrame(images[cam]);
				}
			}
			// Process each camera images
			for (size_t cam = 0; cam < kCameraNums; cam++) {

				// Must be images[cam] to run on non-IP stream inputs
				cv::Mat& preimg = noir1frame;
				//cv::Mat& preimg = images[cam];
				if (!streamcam)
				{
					preimg = images[cam];
				}

				if (preimg.empty()) {
					//is_run = false;
					break;
				}

				// Undistort a captured image
				camera_undistorters[cam]->undistort(preimg, preimg);
				Mat previmg(preimg, myROI);
				cv::Mat& img = preimg(myROI);
				if (!orderCrop)
				{
					img = preimg;
				}

				cv::Mat img_rgb_debug = img.clone();
				cv::Mat img_grey;

				switch (kKEY) {
				case 'r':
					cout << endl << "RESET" << endl << endl;
					eye_model_updaters[cam]->reset();
					angleOffsetHor = 0;
					angleOffsetVert = 0;
					angleOffsetRoll = 0;
					calibState = 0;
					break;
				case 't':
					cout << endl << "UPDATE" << endl << endl;
					eye_model_updaters[cam]->add_fitter_max_count(10);
					break;
				case 'f':
					cout << endl << "RE-CALIBRATING" << endl << endl;
					angleOffsetHor = 0;
					angleOffsetVert = 0;
					angleOffsetRoll = 0;
					calibState = 1;

				default:
					break;
				}

				if (!findEye && streamcam)
				{
					if (!buildEye)
					{
						cout << "Please Align Eye Camera" << endl << "Press 'g' when camera is aligned" << endl;
						buildEye = true;
					}
					cv::imshow(window_names[cam], img_rgb_debug);
					if ((char)waitKey(1) == 103)// Continue when 'g' is pressed
					{
						findEye = true;
						cout << endl << "Please look around" << endl << "Building Eye Model..." << endl << endl;
					}
				}
				else
				{
					if (!eye_model_updaters[cam]->is_model_built())
					{
						cv::imshow(window_names[cam], img_rgb_debug);
					}

					// 2D ellipse detection
					std::vector<cv::Point2f> inlier_pts;
					cv::cvtColor(img, img_grey, CV_RGB2GRAY);
					cv::RotatedRect rr_pf;
					bool is_pupil_found = pupilFitter.pupilAreaFitRR(img_grey, rr_pf, inlier_pts);

					singleeyefitter::Ellipse2D<double> el = singleeyefitter::toEllipse<double>(eye_tracker::toImgCoordInv(rr_pf, img, 1.0));

					// 3D eye pose estimation
					bool is_reliable = false;
					bool is_added = false;
					const bool force_add = false;
					const double kReliabilityThreshold = 0.0;// 0.96;
					double ellipse_realiability = 0.0; /// Reliability of a detected 2D ellipse based on 3D eye model
					if (is_pupil_found) {
						if (eye_model_updaters[cam]->is_model_built()) {
							ellipse_realiability = eye_model_updaters[cam]->compute_reliability(img, el, inlier_pts);
							is_reliable = (ellipse_realiability > kReliabilityThreshold);
							//					is_reliable = true;
						}
						else {
							is_added = eye_model_updaters[cam]->add_observation(img_grey, el, inlier_pts, force_add);
						}

					}

					// Visualize results
					if (cam == 0 && kVisualization) {

						//cout << "Visualize" << endl;
						// 2D pupil
						if (is_pupil_found) {
							cv::ellipse(img_rgb_debug, rr_pf, cv::Vec3b(255, 128, 0), 1);
						}
						// 3D eye ball
						if (eye_model_updaters[cam]->is_model_built()) {
							cv::putText(img, "Reliability: " + std::to_string(ellipse_realiability), cv::Point(30, 440), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 128, 255), 1);
							is_reliable_global = is_reliable;
							if (is_reliable) {
								eye_model_updaters[cam]->render(img_rgb_debug, el, inlier_pts);

								//originCalc = eye_model_updaters[cam]->angle(img_rgb_debug, el, inlier_pts).centre; // Eyeball Centre from eyecam. Irrelevant for this solution, but could be useful
								directionCalc = eye_model_updaters[cam]->angle(img_rgb_debug, el, inlier_pts).normal;

								//std::cout << std::endl << "Direction: " << directionCalc << std::endl << std::endl;

								//Get gaze angles
								vector<double> calcAngles = pointToAngles(directionCalc);
								angleHor = calcAngles[0];
								angleVert = calcAngles[1];

								/*angleHor = 0; //For Debugging Purposes
								angleVert = 0;*/

								xFocus = angle2pixel(angleHor, angleVert)[0];
								yFocus = angle2pixel(angleHor, angleVert)[1];
							}
							else {
								eye_model_updaters[cam]->render_status(img_rgb_debug);
								cv::putText(img, "Sample #: " + std::to_string(eye_model_updaters[cam]->fitter_count()) + "/" + std::to_string(eye_model_updaters[cam]->fitter_end_count()),
									cv::Point(30, 440), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 128, 255), 2);
							}
							cv::imshow(window_names[cam], img_rgb_debug);

						} // Visualization
					}
				} // For each cameras

				// Compute FPS
				frame_rate_counter.count();
				// Print current frame data
				static int ss = 0;
				if (ss++ > 100) {
					//std::cout << "Frame #" << frame_rate_counter.frame_count() << ", FPS=" << frame_rate_counter.fps() << std::endl;
					ss = 0;
				}

				//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
				//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
				//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

				char key_p = (char)waitKey(1);

				if (key_p != 'g')
				{
					nextstate = true;
				}
				// Calibration states
				if (eye_model_updaters[cam]->is_model_built())
				{
					// Note horizontal offset
					if (calibState == 1 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						angleOffsetHor = angleHor;
						calibState = 2;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// Note vertical offset
					if (calibState == 2 && nextstate == true && is_reliable_global)
					{
						angleOffsetVert = angleVert;
						calibState = 3;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// Note roll angle 1
					if (calibState == 3 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						RollTemp1 = calcRoll(angleHor, angleVert);
						cout << "RollTemp1: " << RollTemp1 << endl;
						if (RollTemp1 > 180)
						{
							RollTemp1 = 360 - RollTemp1;
						}
						cout << "RollTemp1: " << RollTemp1 << endl;
						calibState = 4;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// Note roll angle 2
					if (calibState == 4 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						RollTemp2 = calcRoll(angleHor, angleVert) - 180;
						cout << "RollTemp2: " << RollTemp2 << endl;
						if (RollTemp2 > 180)
						{
							RollTemp2 = 360 - RollTemp2;
						}
						cout << "RollTemp2: " << RollTemp2 << endl;
						calibState = 5;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// Note roll angle 3
					if (calibState == 5 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						RollTemp3 = calcRoll(angleHor, angleVert) - 90;
						cout << "RollTemp3: " << RollTemp3 << endl;
						if (RollTemp3 > 180)
						{
							RollTemp3 = 360 - RollTemp3;
						}
						cout << "RollTemp3: " << RollTemp3 << endl;
						calibState = 6;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// Note roll angle 4 and calculate average roll
					if (calibState == 6 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						RollTemp4 = calcRoll(angleHor, angleVert) + 90;
						cout << "RollTemp4: " << RollTemp4 << endl;
						if (RollTemp4 > 180)
						{
							RollTemp4 = 360 - RollTemp4;
						}
						cout << "RollTemp4: " << RollTemp4 << endl;
						angleOffsetRoll = (RollTemp1 + RollTemp2 + RollTemp3 + RollTemp4) / 4;
						calibState = 7;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// 1st quadrant scaling
					if (calibState == 7 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						scaleAngles[0] = angleHor;
						scaleAngles[1] = angleVert;
						cout << "scaleAngles[0]: " << scaleAngles[0] << endl <<
							"scaleAngles[1]: " << scaleAngles[1] << endl;
						calibState = 8;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// 2nd quadrant scaling
					if (calibState == 8 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						scaleAngles[2] = angleHor;
						scaleAngles[3] = angleVert;
						cout << "scaleAngles[2]: " << scaleAngles[2] << endl <<
							"scaleAngles[3]: " << scaleAngles[3] << endl;
						calibState = 9;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// 4th quadrant scaling
					if (calibState == 9 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						scaleAngles[4] = angleHor;
						scaleAngles[5] = angleVert;
						cout << "scaleAngles[4]: " << scaleAngles[4] << endl <<
							"scaleAngles[5]: " << scaleAngles[5] << endl;
						calibState = 10;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
					// 3rd quadrant scaling
					if (calibState == 10 && key_p == 'g' && nextstate == true && is_reliable_global)
					{
						scaleAngles[6] = angleHor;
						scaleAngles[7] = angleVert;
						cout << "scaleAngles[6]: " << scaleAngles[6] << endl <<
							"scaleAngles[7]: " << scaleAngles[7] << endl;
						calibState = 11;
						nextstate = false;
						cout << "Next state = " << calibState << endl;
					}
				}
				// For noting test iterations
				if (key_p != 'p')
				{
					nexttest = true;
				}

				if (calibState == 11 && key_p == 'p' && nexttest == true)
				{
					cout << "Test Point: " << testy << testx << endl;
					if (testx == 5)
					{
						testy++;
						testx = 1;
					}
					else
					{
						testx++;
					}
					nexttest = false;
				}

				if (key_p != 'x')
				{
					goodfit = true;
				}

				if (calibState == 11 && key_p == 'x' && goodfit == true)
				{
					cout << endl << "GOOD FIT" << endl;
					goodfit = false;
				}


				//Realsense Run When 'h' Is Pressed
				// Snapshot and calculation
				if ((key_p == 'h' && eye_model_updaters[cam]->is_model_built()) || (calibState == 0 && eye_model_updaters[cam]->is_model_built()))
				{
					// Align eyecam before calibrating
					if (calibState == 0)
					{
						cout << endl << "Please focus on a point, and press 'h' until the green square is on the focus point" <<
							endl << "When they overlap, press 'g'" << endl;
						calibState = 1;
					}
					if (calibState == 11)
					{
						//cout << endl << "'h' Pressed" << endl << endl;
					}

					frames = pipe.wait_for_frames();
					auto depth_frame = frames.get_depth_frame();
					auto color_frame = frames.get_color_frame();
					// Make sure that both depth and  color are present for calculations
					if (!depth_frame || !color_frame)
						continue;


					// Creating OpenCV Matrix from a color RSimage
					Mat color(Size(rsResX, rsResY), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

					// cheking if an RSimage was read
					if (color.empty())
					{
						cerr << "RSimage was not generated !" << endl;
						return 1;
					}

					/*namedWindow("Display Image", WINDOW_AUTOSIZE);
					imshow("Display Image", color);*/


					//convert RGB to HSV
					cvtColor(color, imgHSV, COLOR_BGR2HSV);


					//aplying color filter to HSV RSimage
					HSVthreshold(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, 0);

					auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
					auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

					auto depth_intrin = depth_profile.get_intrinsics();
					auto color_intrin = color_profile.get_intrinsics();
					auto depth2color_extrin = depth_profile.get_extrinsics_to(color_profile);
					auto color2depth_extrin = color_profile.get_extrinsics_to(depth_profile);

					// Setting green pixel positions based on state
					if (calibState == 1 || calibState == 2)
					{
						//Center
						xFocus = rsCenterX;
						yFocus = rsCenterY;
					}
					if (calibState == 3)
					{
						//Right
						xFocus = rsResX - 5;
						yFocus = rsResY / 2;
					}
					if (calibState == 4)
					{
						//Left
						xFocus = 5;
						yFocus = rsResY / 2;
					}
					if (calibState == 5)
					{
						//Up
						xFocus = rsResX / 2;
						yFocus = 5;
					}
					if (calibState == 6)
					{
						//Down
						xFocus = rsResX / 2;
						yFocus = rsResY - 5;
					}
					if (calibState == 7)
					{
						//Top Right
						xFocus = rsResX - 5;
						yFocus =  5;
					}
					if (calibState == 8)
					{
						//Top Left
						xFocus = 5;
						yFocus = 5;
					}
					if (calibState == 9)
					{
						//Bottom Right
						xFocus = rsResX - 5;
						yFocus = rsResY - 5;
					}
					if (calibState == 10)
					{
						//Bottom Left
						xFocus = 5;
						yFocus = rsResY - 5;
					}


					// Object Detection Here, if gaze is within FOV
						if (xFocus >= 5 && xFocus <= (rsResX - 5) && yFocus >= 5 && yFocus <= (rsResY - 5))
						{
							if (calibState == 11)
							{
								//----------------------------------------Object Recognition Start----------------------------------------

								Mat greyOut, threshOut, cannyImg;

								//Extract the contours so that
								vector<vector<Point> > contours; //Each contour point is saved in the vector
								vector<Vec4i> hierarchy;


								blur(color, color, Size(3, 3));
								cvtColor(color, greyOut, COLOR_BGR2GRAY);
								threshold(greyOut, threshOut, threshold_value, max_binary_value, threshold_type);
								//imshow("Greyscale window", greyOut);
								//imshow("Threshold window", threshOut);

								cv::Canny(color, cannyImg, 127, 2 * 127);
								//imshow("Canny edge detection window", cannyImg);
								findContours(cannyImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
								
								vector<vector<Point> > contours_poly(contours.size());
								vector<Rect> boundRect(contours.size());
								//contours.resize(contours.size());
								//contours_poly.resize(contours_poly.size());
								//boundRect.resize(contours.size());
								vector<Point> centroidBox; // Vector with bounding box coordinates

								int closestDist = 800;
								int closestX = 0;
								int closestY = 0;
								int closestObj = 0, distCalc = 0;
								if (!contours.empty())
								{
									for (size_t i = 0; i < contours.size(); i++)
									{
										approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
										boundRect[i] = boundingRect(Mat(contours_poly[i]));

										Scalar c = Scalar(255, 0, 0);
										if (boundRect[i].width > minBoxSize && boundRect[i].height > minBoxSize) //Only draw bounding box if over minimum size
										{
											drawContours(color, contours, (int)i, c, 1, 8, vector<Vec4i>(), 0, Point());
											rectangle(color, boundRect[i].tl(), boundRect[i].br(), c, 2, 8, 0);

											//Add Centerpoint
											centroidBox.push_back(Point(boundRect[i].x + (boundRect[i].width / 2), boundRect[i].y + (boundRect[i].height / 2)));

											if (centroidBox[i].x != 0.0 && centroidBox[i].y != 0.0)
											{
												distCalc = sqrt(pow(abs(xFocus - centroidBox[i].x), 2) + pow(abs(yFocus - centroidBox[i].y), 2));
												if (distCalc < closestDist)
												{
													closestDist = distCalc;
													closestObj = i;
													closestX = centroidBox[i].x;
													closestY = centroidBox[i].y;
												}
											}
										}
										else
										{
											centroidBox.push_back(Point(0.0, 0.0)); // If the bounding box is too small, add (0, 0) to the vector instead
										}
									}
									//----------------------------------------Object Recognition End----------------------------------------

									//---------------------------Object Recognition <-> Gaze Point Comparison Start---------------------------

									// Highlight Gaze Point
									for (int i = closestX - 5; i < closestX + 5; i++)
									{
										for (int j = closestY - 5; j < closestY + 5; j++)
										{
											Vec3b& target = color.at<Vec3b>(j, i);

											target[0] = 0;
											target[1] = 0;
											target[2] = 255;

											// set pixel
											color.at<Vec3b>(Point(i, j)) = target;
										}
									}
									//Highlight Focus Object
									Scalar red = Scalar(0, 0, 255);
									drawContours(color, contours, (int)closestObj, red, 1, 8, vector<Vec4i>(), 0, Point());
									rectangle(color, boundRect[closestObj].tl(), boundRect[closestObj].br(), red, 2, 8, 0);


									// Get Depth
									float rgb_src_pixel[2] = { closestX, closestY }; // The RGB coordinate for the center of the marble
									float dpt_tgt_pixel[2] = { 0 }; // The depth pixel that has the best match for that RGB coordinate

									auto sensor = selection.get_device().first<rs2::depth_sensor>();
									auto scale = sensor.get_depth_scale();


									// Search along a projected beam from 0.1m to 10 meter. This can be optimized to the concrete scenario, e.g. if you know that the data is bounded within [min..max] range
									rs2_project_color_pixel_to_depth_pixel(dpt_tgt_pixel, reinterpret_cast<const uint16_t*>(depth_frame.get_data()), scale, 0.1f, 10,
										&depth_intrin, &color_intrin,
										&color2depth_extrin, &depth2color_extrin, rgb_src_pixel);

									// Verify that the depth correspondence is valid, i.e within the frame boundaries

									auto distance = depth_frame.get_distance(dpt_tgt_pixel[0], dpt_tgt_pixel[1]);


									cout << "Snapshot #: " << testcount << endl;
									testcount++;
									// cout Distance and Pixel
									cout << "The distance to the object is: " << distance << endl;

									// Focus Pixel
									//cout << "Measured Horizontal Focus Angle: " << angleHor << endl <<
										//"Measured Vertical Focus Angle: " << angleVert //<< endl <<
										///*"x: " << xFocus << "   y: " << yFocus*/ << endl;

									vector<double> objectAngles = pixel2angles(closestX, closestY);
									// Final Data For Detected Object
									cout << /*endl << endl << "The distance to the object is: " << distance << endl <<*/
										"Horizontal Angle to Object: " << objectAngles[0] << endl <<
										"Vertical Angle to Object: " << objectAngles[1] << endl << endl;
								}
							}

							// Draw Gaze Point
							for (int i = xFocus - 5; i < xFocus + 5; i++)
							{
								for (int j = yFocus - 5; j < yFocus + 5; j++)
								{
									Vec3b& target = color.at<Vec3b>(j, i);

									target[0] = 102;
									target[1] = 255;
									target[2] = 0;

									// set pixel
									color.at<Vec3b>(Point(i, j)) = target;
								}
							}
							if (calibState == 11)
							{
								for (int i = rsResX / 2 - 2; i < rsResX / 2 + 2; i++)
								{
									for (int j = rsResY / 2 - 2; j < rsResY / 2 + 2; j++)
									{
										Vec3b& target = color.at<Vec3b>(j, i);

										target[0] = 255;
										target[1] = 0;
										target[2] = 0;

										// set pixel
										color.at<Vec3b>(Point(i, j)) = target;
									}
								}
							}

							namedWindow("Display Image", WINDOW_AUTOSIZE);
							cv::imshow("Display Image", color);
						}
						else
						{
							cout << "Focus outside of FOV" << endl;
						}
				}
				//Realsense End
				//----------------------------------------------

				if (waitKey(30) >= 0) break;
			}
		}// Main capture loop

		return 0;
	}
}