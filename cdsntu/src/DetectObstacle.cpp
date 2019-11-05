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
Mat DetectObstacle::cvDepth(const Mat &depth)
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
Mat DetectObstacle::threshDepthImg(const Mat &gray)
{
    Mat dst = gray.clone();
    for(int i = 0; i < gray.size().height; i++)
        for(int j = 0; j < gray.size().width; j++)
            if(gray.at<uchar>(i, j) > low && gray.at<uchar>(i, j) < up)
                dst.at<uchar>(i, j) = 255;
            else
                dst.at<uchar>(i, j) = 0;
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
            if(rc.area() >= area)
            {
                obs = Rect(rc.x , rc.y + cut, rc.width, rc.height);
                rect.push_back(obs);
            }
        }
        if(rect.size() > 0)
            return findMaxRect(rect);
    }
    return obs;
}
void DetectObstacle::showObj(const Mat &depthImg, const Mat &rgbImg)
{
    Rect obs = null;
    Mat rgb = rgbImg.clone();
    Mat grayImg = processDepth(depthImg).clone();
    Mat threshImg = threshDepthImg(grayImg);
    imshow("DepthBin", threshImg);
    Mat roiImg = roi(threshImg, 0, cut, threshImg.size().width, lenCut);
    obs = detect(roiImg, area, cut);
    if(obs != null)
        rectangle(rgb, Point(obs.x, obs.y), Point(obs.x+obs.width, obs.y+obs.height), Scalar(255,0,0), 3);
    imshow("obstacle", rgb);
}
