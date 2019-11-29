#ifndef HEADER_H
#define HEADER_H


#define SPEED_TOPIC "cdsntu2/set_speed"
#define STEER_TOPIC "cdsntu2/set_angle"
#define CAMERA_TOPIC "cdsntu2/set_camera_angle"
#define IMAGE_RGB_TOPIC "cdsntu2/camera/rgb"
#define IMAGE_DEPTH_TOPIC "cdsntu2/camera/depth"
#define SIGN_TOPIC "cdsntu2/sign"


#define IMG_W 320
#define IMG_H 240

#define PI 3.14f
#define MIN_AREA 0.003f //For Sign Detect

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

static int BIRDVIEW_W = 240;
static int BIRDVIEW_H = 320;
static int sky = 90;

static float velocity = 60;

#endif
