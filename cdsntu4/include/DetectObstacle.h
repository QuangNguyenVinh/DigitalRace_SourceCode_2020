//
// Created by quang on 15/10/2019.
//

#ifndef DETECTOBSTACLE_H
#define DETECTOBSTACLE_H

#include "header.h"
#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;

class DetectObstacle
{
    public:
        static Rect null;
        DetectObstacle(string maskSrc);
        ~DetectObstacle();
        Rect showObj(const Mat &depthImg, vector<vector<Point>> treeContours);
        vector<Vec3f> RectSign(const Mat &depthImg);

    private:
        Mat processDepth(const Mat &depthImg);
        Mat roi(const Mat &src);
        Mat thresh(const Mat &gray);
        Rect findMaxRect(const vector<Rect> rect);
        Rect detect(const Mat &bin);
        

        Mat mask;
        int value = 75;
        int width = 30;
        int height = 10;
        int buW = 20, buH = 20;

};

#endif //DETECTOBSTACLE_H
