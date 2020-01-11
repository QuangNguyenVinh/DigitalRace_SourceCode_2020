#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "header.h"


class DetectLane
{
	public:
		DetectLane();
		~DetectLane();

        Mat updateLane(const Mat &src, Rect obstacle);
        Mat noCutFinal;
	private:
        Mat cutROI(const Mat &src);
        Mat detectShadow(const Mat &src);
        Mat detectLane(const Mat &img);
        Mat detectLane2(const Mat &src);
        Mat detectSnow(const Mat &src);
        Mat dilateLane(const Mat &src);
        Mat erodeLane(const Mat &src);
        Mat removeNoise(const Mat &bin);
        Rect rect = Rect(0, 0, 0, 0);
		int minThreshold[3] = {0, 0, 40};
		int maxThreshold[3] = {179, 40, 150};
		int minShadow[3] = {63, 0, 32};
		int maxShadow[3] = {179, 100, 71};
		int minOther[3] = {0, 0, 80};
		int maxOther[3] = {179, 40, 125};
        int minSnow[3] = {15, 0, 165};
        int maxSnow[3] = {29, 45, 255};
};
#endif
