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
        DetectObstacle();
        ~DetectObstacle();
        Rect showObj(const Mat &depthImg, const Mat &rgbImg);

    private:
        Mat processDepth(const Mat &depthImg);
        Mat cvDepth(const Mat &depth);
        Mat thresh(const Mat &src, int val = 100);
        Mat roi(const Mat &src, int x, int y, int w, int h);
        Mat threshDepthImg(const Mat &gray);
        Rect findMaxRect(const vector<Rect> rect);
        Rect detect(const Mat &bin, int area, int cut = 40);

        int low = 35;
        int up = 110;
        int cut = 50;
        int lenCut = 80;
        int area = 100;
        int bu = 20;


};

#endif //DETECTOBSTACLE_H
