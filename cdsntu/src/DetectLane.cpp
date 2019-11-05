#include "DetectLane.h"
DetectLane::DetectLane()
{
	    cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
        cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

        cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
        cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

        cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
        cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);
}
DetectLane::~DetectLane()
{

}
Mat DetectLane::cutROI(const Mat &src)
{
    Mat dst;
    int h = src.rows, w = src.cols;
    Mat mask = Mat::zeros(src.size(), src.type());

    Point pts[4] = {
            Point(0, h),
            Point(80, 80),
            Point(w - 80, 80),
            Point(w, h),
    };

    fillConvexPoly(mask, pts, 4, Scalar(255));
    bitwise_and(src, mask, dst);

    return dst;
}
Mat DetectLane::detectLane(const Mat &img)
{
    Mat laneImg, hsvImg;
    cvtColor(img, hsvImg, COLOR_BGR2HSV);
    inRange(hsvImg, Scalar(minThreshold[0],minThreshold[1],minThreshold[2]),
            Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), laneImg);
    laneImg = cutROI(laneImg).clone();
    return laneImg;
}
Mat DetectLane::birdView(const Mat &srcImg)
{
    Point2f src[4], dst[4];
    int width = srcImg.size().width;
    int height = srcImg.size().height;

    src[0] = Point(0, sky);
    src[1] = Point(width, sky);
    src[2] = Point(width, height);
    src[3] = Point(0, height);

    dst[0] = Point(0,0);
    dst[1] = Point(BIRDVIEW_W, 0);
    dst[2] = Point(BIRDVIEW_W - 105, BIRDVIEW_H);
    dst[3] = Point(105, BIRDVIEW_H);

    Mat M = getPerspectiveTransform(src, dst);

    Mat dstImg(BIRDVIEW_H, BIRDVIEW_W, CV_8UC3);

    warpPerspective(srcImg, dstImg, M, dstImg.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dstImg;
}
Mat DetectLane::showRes(const Mat &src) //For debug only
{
	Mat dst = detectLane(src).clone();
	return birdView(dst);
}
