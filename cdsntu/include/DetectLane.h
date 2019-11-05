#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "header.h"


class DetectLane
{
	public:
		DetectLane();
		~DetectLane();
        Mat cutROI(const Mat &src);
		Mat detectLane(const Mat &img);
		Mat birdView(const Mat &src);
		Mat showRes(const Mat &src); 
	private:
		int minThreshold[3] = {0, 0, 45};
		int maxThreshold[3] = {179, 45, 125};
};
#endif
