#ifndef HEADER_H
#define HEADER_H



#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
using namespace std;
using namespace cv;
using namespace cv::ml;


#define IMG_W 320
#define IMG_H 240

#define PI 3.14f
#define MIN_AREA 0.001f //For Sign Detect
#define MAX_AREA 0.055f

static float SPEED_TURN = -100;




static string TEAM_NAME = "team1003";
static string SPEED_TOPIC = TEAM_NAME + "/set_speed";
static string STEER_TOPIC = TEAM_NAME + "/set_angle";
static string CAMERA_TOPIC = TEAM_NAME + "/set_camera_angle";
static string IMAGE_RGB_TOPIC = TEAM_NAME + "/camera/rgb";
static string IMAGE_DEPTH_TOPIC = TEAM_NAME + "/camera/depth";
static string SIGN_TOPIC = TEAM_NAME + "/sign";

static int BIRDVIEW_W = 240;
static int BIRDVIEW_H = 320;
static int sky = 90;

static bool show_val = false;

#endif
