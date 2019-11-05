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
        void showObj(const Mat &depthImg, const Mat &rgbImg);

    private:
        Mat processDepth(const Mat &depthImg);
        Mat cvDepth(const Mat &depth);
        Mat thresh(const Mat &src, int val = 100);
        Mat roi(const Mat &src, int x, int y, int w, int h);
        Mat threshDepthImg(const Mat &gray);
        Rect findMaxRect(const vector<Rect> rect);
        Rect detect(const Mat &bin, int area, int cut = 40);

        int low = 35;
        int up = 80;
        int cut = 120;
        int lenCut = 80;
        int area = 2000;


};

#endif //DETECTOBSTACLE_H
