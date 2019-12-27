//
// Created by quang on 15/10/2019.
//

#ifndef DETECTOBSTACLE_H
#define DETECTOBSTACLE_H

#include "header.h"

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
        int value = 80;
        int width = 30;
        int height = 10;
        int buW = 20, buH = 20;

};

#endif //DETECTOBSTACLE_H
