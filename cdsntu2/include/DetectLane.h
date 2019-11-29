#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "header.h"


class DetectLane
{
	public:
		DetectLane();
		~DetectLane();

        Mat updateLane(const Mat &src, Rect obstacle);
	private:
        Mat allBlack(const Mat &src, const Rect &rect);
        Mat Filter(const Mat &src);
        Mat cutROI(const Mat &src);
        Mat detectShadow(const Mat &src);
	Mat detectShadow1(const Mat &src);
        Mat detectLane(const Mat &img);
        Mat detectLane2(const Mat &src);
        Mat detectSnow(const Mat &src);
        Mat birdView(const Mat &src);
        Mat showRes(const Mat &src);
        Mat dilateLane(const Mat &src);
        Mat erodeLane(const Mat &src);


        Rect rect = Rect(0, 0, 0, 0);
        int minThreshold[3] = {0, 0, 40};
        int maxThreshold[3] = {179, 40, 150};

        int minOther[3] = {0, 0, 80};
        int maxOther[3] = {179, 40, 125};

        int minSnow[3] = {0, 0, 165};
        int maxSnow[3] = {179, 45, 255};
        int minth[3] = { 0,0,25 };
        int maxth[3] = { 179,45,127 };
        int minShadow[3] = {25, 0, 32};
        int maxShadow[3] = {179, 100, 71};
	int minShadow1[3] = {65, 0, 32};
        int maxShadow1[3] = {179, 100, 71};
};
#endif
