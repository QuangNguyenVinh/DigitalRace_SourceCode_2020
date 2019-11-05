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
Mat DetectLane::detectShadow(const Mat &src)
{
    Mat shadow, shadowHSV;
    cvtColor(src, shadowHSV, COLOR_BGR2HSV);
    inRange(shadowHSV, Scalar(minShadow[0], minShadow[1], minShadow[2]),
                        Scalar(maxShadow[0], maxShadow[1], maxShadow[2]), shadow);
    return shadow;

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
Mat DetectLane::detectLane2(const Mat &src)
{
    Mat laneImg, hsvImg;
    cvtColor(src, hsvImg, COLOR_BGR2HSV);
    inRange(hsvImg, Scalar(minOther[0],minOther[1],minOther[2]),
            Scalar(maxOther[0], maxOther[1], maxOther[2]), laneImg);
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
Mat DetectLane::updateLane(const Mat &src)
{
    Mat lane = detectLane(src).clone();
    Mat lane2 = detectLane2(src).clone();
    Mat laneComb;
    bitwise_or(lane, lane2, laneComb);
    //bitwise_and(lane, laneComb, laneComb);
    Mat shadow = detectShadow(src).clone();
    Mat finalImg;
    bitwise_or(laneComb, shadow, finalImg);
    finalImg = cutROI(finalImg).clone();
    return finalImg;
}
