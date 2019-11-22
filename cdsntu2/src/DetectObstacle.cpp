#include "DetectObstacle.h"
Rect DetectObstacle::null = Rect();
DetectObstacle::DetectObstacle()
{
    cvCreateTrackbar("Low", "DepthBin", &low, 255);
    cvCreateTrackbar("High", "DepthBin", &up, 255);
}
DetectObstacle::~DetectObstacle(){}
Mat DetectObstacle::processDepth(const Mat &depthImg)
{
    Mat dst;
    cvtColor(depthImg, dst, COLOR_BGR2GRAY);
    return dst;
}
Mat DetectObstacle::rvDepth(const Mat &depth)
{
    Mat dst = Scalar::all(255) - depth;
    return dst;
}
Mat DetectObstacle::thresh(const Mat &src, int val)
{
    Mat dst;
    threshold(src, dst, val, 255, 1);
    return dst;
}
Mat DetectObstacle::roi(const Mat &src, int x, int y, int w, int h)
{
    Rect roi = Rect(x, y, w, h);
    Mat dst = src(roi);
    return dst;
}
Mat DetectObstacle::filterObj(const Mat &src)
{
    Mat S1 = Mat::ones(9, 9, CV_8UC1);
    S1.at<uchar>(5, 5) = 4;
    Mat S2 = Mat::ones(15, 15, CV_8UC1);
    //S2.at<uchar>(5, 5) = 4;
    Mat dst;
    erode(src, dst, S1);
    dilate(dst, dst, S2);
    return dst;
}
Mat DetectObstacle::threshDepthImg(const Mat &gray)
{
    Mat dst = gray.clone();
    for(int i = 0; i < gray.size().height; i++)
        for(int j = 0; j < gray.size().width; j++)
            if(gray.at<uchar>(i, j) > low && gray.at<uchar>(i, j) < up)
                dst.at<uchar>(i, j) = 255;
            else
                dst.at<uchar>(i, j) = 0;
    dst = filterObj(dst);
    return dst;
}
Rect DetectObstacle::findMaxRect(const vector<Rect> rect)
{
    Rect rc = rect[0];
    int maxArea = rect[0].area();
    for(int i = 1; i < rect.size(); i++)
        if(rect[i].area() > maxArea)
        {
            maxArea = rect[i].area();
            rc = rect[i];
        }
    return rc;
}
Rect DetectObstacle::detect(const Mat &bin, int area, int cut)
{
    Mat dst = bin.clone();
    vector< vector<Point> > contours;
    vector<Vec4i> hierachy;
    findContours(bin, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    vector<Rect> rect(0);

    Rect obs = null;
    if(contours.size() > 0)
    {
        for(int i = 0; i < contours.size(); i++)
        {
            Rect rc = boundingRect(contours[i]);
            if(rc.height >= minHeight && rc.width >= minWidth && rc.width <= 160)
            {
                obs = Rect(rc.x , rc.y + cut , rc.width, rc.height); // + cut because using ROI
                rect.push_back(obs);
            }
        }
        if(rect.size() > 0)
            return findMaxRect(rect);
    }
    return obs;
}
Rect DetectObstacle::showObj(const Mat &depthImg, const Mat &rgbImg)
{
    Rect obs = null;
    Rect rect = Rect(0,0,0,0);

    Mat rgb = rgbImg.clone();
    Mat grayImg = processDepth(depthImg).clone();
    Mat threshImg = threshDepthImg(grayImg);
    imshow("DepthBin", threshImg);

    Mat roiImg = roi(threshImg, 0, cut, threshImg.size().width, lenCut);

    obs = detect(roiImg, area, cut);
    if(obs != null)
    {
        int wRect = obs.width , hRect = obs.height;

        //rectangle(rgb, Point(obs.x, obs.y), Point(wRect, hRect), Scalar(0,0,255), 3);

        rect = Rect(obs.x - buW, obs.y - buH, wRect + 2*buW, hRect + 2*buH);

        if(rect.x < 0)
            rect.x = 0;

        if(rect.width + rect.x > rgb.size().width)
            rect.width = rgb.size().width - rect.x;

        if(rect.y < 0)
            rect.y = 0;

        if(rect.height + rect.y > rgb.size().height)
            rect.height = rgb.size().height - rect.y;

    }
    return rect;
}