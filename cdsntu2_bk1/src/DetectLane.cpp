#include "DetectLane.h"
DetectLane::DetectLane()
{
	    cvCreateTrackbar("LowH", "Threshold", &minth[0], 179);
        cvCreateTrackbar("HighH", "Threshold", &maxth[0], 179);

        cvCreateTrackbar("LowS", "Threshold", &minth[1], 255);
        cvCreateTrackbar("HighS", "Threshold", &maxth[1], 255);

        cvCreateTrackbar("LowV", "Threshold", &minth[2], 255);
        cvCreateTrackbar("HighV", "Threshold", &maxth[2], 255);
}
DetectLane::~DetectLane()
{

}
Mat DetectLane::allBlack(const Mat &src, const Rect &rect)
{
    Mat binImg = src.clone();
    for(int i = rect.x; i < rect.x + rect.width; i++)
    {
        for(int j = rect.y; j < rect.y + rect.height; j++)
        {
            binImg.at<uchar>(j,i) = 0;
        }
    } 
    return binImg;
}
Mat DetectLane::Filter(const Mat &src)
{
    Mat S1 = Mat::ones(9,9, CV_8UC1);
    S1.at<uchar>(5, 5) = 4;
    Mat dst;
    erode(src, dst, S1);
    dilate(dst, dst, S1);
    return dst;
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
    shadow = Filter(shadow);
    return shadow;

}
Mat DetectLane::detectLane(const Mat &img)
{
    Mat laneImg, hsvImg;
    cvtColor(img, hsvImg, COLOR_BGR2HSV);
    inRange(hsvImg, Scalar(minth[0],minth[1],minth[2]),
            Scalar(maxth[0], maxth[1], maxth[2]), laneImg);
    laneImg = Filter(laneImg);
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
    snowImg = Filter(snowImg);
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
Mat DetectLane::dilateLane(const Mat &src)
{
    Mat dst;
    int dilation_size = 4;
    cv::Mat element = cv::getStructuringElement(MORPH_ELLIPSE, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
    dilate(src, dst, element);
    return dst;
}
Mat DetectLane::erodeLane(const Mat &src)
{
    Mat dst;
    int erosion_size = 4;
    cv::Mat element = cv::getStructuringElement(MORPH_ELLIPSE, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
    erode(src, dst, element);
    return dst;
}


Mat DetectLane::updateLane(const Mat &src, Rect obstacle)
{
    rect = obstacle;
    Mat lane,snow,shadow,dst;


    shadow = detectShadow(src);

    snow = detectSnow(src);

    lane = detectLane(src);
    for(int i =0 ;i<snow.rows;i++)
        for (int j = 0; j < snow.cols; j++)
        {
            if (i < 120 || (j <= 50) || j >= (snow.cols - 50))
            {
                snow.at<uchar>(i, j) = 0;
                shadow.at<uchar>(i, j) = 0;
            }
        }

    bitwise_or(snow, lane, dst);
    bitwise_or(shadow, dst, dst);
    //dst = allBlack(dst, rect).clone();
    dst(rect) = Scalar(0);
    dst = cutROI(dst).clone();
    return dst;

}
