#ifndef HEADER_H
#define HEADER_H



#define IMG_W 320
#define IMG_H 240

#define PI 3.14f
#define MIN_AREA 0.0005f //For Sign Detect

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

static string TEAM_NAME = "cdsntu4";
static string SPEED_TOPIC = TEAM_NAME + "/set_speed";
static string STEER_TOPIC = TEAM_NAME + "/set_angle";
static string CAMERA_TOPIC = TEAM_NAME + "/set_camera_angle";
static string IMAGE_RGB_TOPIC = TEAM_NAME + "/camera/rgb";
static string IMAGE_DEPTH_TOPIC = TEAM_NAME + "/camera/depth";

static float velocity = 80;
static bool show_val = false;
#endif
