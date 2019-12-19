#include "DetectLane.h"
DetectLane::DetectLane()
{
	    cvCreateTrackbar("LowH", "tb_lane", &minThreshold[0], 179);
        cvCreateTrackbar("HighH", "tb_lane", &maxThreshold[0], 179);

        cvCreateTrackbar("LowS", "tb_lane", &minThreshold[1], 255);
        cvCreateTrackbar("HighS", "tb_lane", &maxThreshold[1], 255);

        cvCreateTrackbar("LowV", "tb_lane", &minThreshold[2], 255);
        cvCreateTrackbar("HighV", "tb_lane", &maxThreshold[2], 255);
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
            Point(100, 80),
            Point(w - 100, 80),
            Point(w, h),
    };

    Point pts2[6] = {
            Point(0, h),
	    Point(0, (int)(h*3/4)),
            Point(100, 100),
            Point(w - 100, 100),
	    Point(w, (int)(h*3/4)),
            Point(w, h),
    };

    fillConvexPoly(mask, pts2, 6, Scalar(255));
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
Mat DetectLane::detectSnow(const Mat &src)
{
    Mat snowImg, hsvImg;
    cvtColor(src, hsvImg, COLOR_BGR2HSV);
    inRange(hsvImg, Scalar(minSnow[0],minSnow[1],minSnow[2]),
            Scalar(maxSnow[0], maxSnow[1], maxSnow[2]), snowImg);
    return snowImg;
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
Mat DetectLane::dilateLane(const Mat &src) //Gian anh
{
    Mat dst;
    int dilation_size = 3;
    cv::Mat element = cv::getStructuringElement(MORPH_CROSS, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
    dilate(src, dst, element);
    return dst;
}
Mat DetectLane::erodeLane(const Mat &src) //Co anh
{
    Mat dst;
    int erosion_size = 8;
    cv::Mat element = cv::getStructuringElement(MORPH_CROSS, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
    erode(src, dst, element);
    return dst;
}
Mat DetectLane::removeNoise(const Mat &bin)
{
    double maxArea = 0.0;
    int savedContour = -1;
    Mat dst = Mat::zeros(bin.size(), bin.type());
    vector< vector<Point> > contours(0);
    vector<Vec4i> hierachy(0);

    findContours(bin, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    if(contours.size() > 0)
    {
        int nContours = contours.size();
        double area = 0;
        for(int i = 0; i < nContours; i++)
        {
            area = contourArea(contours[i]);
            if(area > maxArea)
            {
                maxArea = area;
                savedContour = i;
            }
        }
        drawContours(dst, contours, savedContour, Scalar(255), CV_FILLED, 8);
    }
    dst &= bin;
    return dst;
}
Mat DetectLane::noCutFinal(const Mat &src)
{
    
    Mat snow, lane, finalImg;
    //Processing snow image
    snow = detectSnow(src).clone();
    snow = erodeLane(snow).clone();
    snow(Rect(0, 0, snow.cols, 120)) = Scalar(0);
    snow(Rect(0,120, 50, snow.rows - 120)) = Scalar(0);
    snow(Rect(snow.cols - 50, 120, 50 , snow.rows - 120)) = Scalar(0); 
    //Process lane image
    lane = detectLane(src).clone();
    //imshow("tb_lane", lane);
    lane = erodeLane(lane).clone();
    lane = dilateLane(lane).clone();
    lane = removeNoise(lane).clone();
    //Process final image
    finalImg = lane| snow;
    //finalImg = removeNoise(finalImg).clone();
    
    //finalImg(rect) = Scalar(0);
    //rectangle(finalImg, Rect(10,90,80,15), Scalar(255,255,255), 1);
    //rectangle(finalImg, Rect(210, 80, 100, 15), Scalar(255,255,255), 1);
    //rectangle(finalImg, Rect(0,125,40,20), Scalar(255,255,255), 1);
    //rectangle(finalImg, Rect(280, 130, 40, 20), Scalar(255,255,255), 1);
    //imshow("finalnocut", finalImg);
    return finalImg;
}

Mat DetectLane::updateLane(const Mat &src, Rect obstacle)
{
    rect = obstacle;
    Mat snow, lane, finalImg;
    //Processing snow image
    snow = detectSnow(src).clone();
    snow = erodeLane(snow).clone();
    snow(Rect(0, 0, snow.cols, 120)) = Scalar(0);
    snow(Rect(0,120, 50, snow.rows - 120)) = Scalar(0);
    snow(Rect(snow.cols - 50, 120, 50 , snow.rows - 120)) = Scalar(0); 
    //Process lane image
    lane = detectLane(src).clone();
    //imshow("tb_lane", lane);
    lane = erodeLane(lane).clone();
    lane = dilateLane(lane).clone();
    lane = removeNoise(lane).clone();
    //Process final image
    finalImg = lane| snow;
    //finalImg = removeNoise(finalImg).clone();
    
    finalImg(rect) = Scalar(0);
    //rectangle(finalImg, rect, Scalar(0,0,0), -1);
    finalImg = cutROI(finalImg).clone();
    //imshow("final", finalImg);
    return finalImg;
}
