//
// Created by quang on 15/10/2019.
//

#ifndef DETECTOBSTACLE_H
#define DETECTOBSTACLE_H

#include "header.h"

class DetectObstacle
{
    public:
        static Rect null;
        DetectObstacle(const string maskSrc);
        ~DetectObstacle();
        vector<Vec3f> findRectSign(const Mat &depthImg);
        Rect showObj(const Mat &depthImg, vector<vector<Point>> treeContours);
        void pubObstacle();

    private:
        Mat mask;
        bool obs_flag = false;
        Mat processDepth(const Mat &depthImg);
        Mat revDepth(const Mat &depth);
        Mat thresh(const Mat &src);
        Mat roi(const Mat &src);
        Rect findMaxRect(const vector<Rect> rect);

        Rect detect(const Mat &bin);

        int value = 80;

  
        int buW = 30, buH = 20;
        int minWidth = 30;
        int minHeight = 20;

        ros::NodeHandle obsNode;
        ros::Publisher obsPub;


};

#endif //DETECTOBSTACLE_H
