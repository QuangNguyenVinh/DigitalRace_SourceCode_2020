#include "DetectObstacle.h"
Rect DetectObstacle::null = Rect();
DetectObstacle::DetectObstacle(const string maskSrc)
{
    cvCreateTrackbar("value", "tb_depth", &value,255);
    mask = imread(maskSrc, IMREAD_COLOR);
    cvtColor(mask, mask, COLOR_BGR2GRAY);

    obsPub = obsNode.advertise<std_msgs::Bool>(OBSTACLE_TOPIC,1);

}
DetectObstacle::~DetectObstacle(){}
Mat DetectObstacle::processDepth(const Mat &depthImg)
{
    Mat dst;
    cvtColor(depthImg, dst, COLOR_BGR2GRAY);
    return dst;
}
Mat DetectObstacle::revDepth(const Mat &depth)
{
    Mat dst = Scalar::all(255) - depth;
    return dst;
}
Mat DetectObstacle::thresh(const Mat &src)
{
    Mat dst;
    threshold(src, dst, value, 255, THRESH_BINARY);
    return dst;
}
Mat DetectObstacle::roi(const Mat &src)
{
    Mat dst;
    int h = src.rows, w = src.cols;
    Mat mask = Mat::zeros(src.size(), src.type());

    Point pts[6] = {
            Point(0, h),
            Point(50, 120),
            Point(80, 60),
            Point(w - 80, 60),
            Point(w - 50, 120),
            Point(w, h)
    };

    fillConvexPoly(mask, pts, 6, Scalar(255));
    bitwise_and(src, mask, dst);
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
Rect DetectObstacle::detect(const Mat &bin)
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
                obs = Rect(rc.x , rc.y, rc.width, rc.height);
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
    obs_flag = false;
    Rect rect = Rect(0,0,0,0);

    Mat rgb = rgbImg.clone();
    Mat grayImg = processDepth(depthImg).clone();

    Mat dst;
    absdiff(grayImg, mask, dst);

    Mat threshImg = thresh(dst);
    Mat roiImg = roi(threshImg);
    imshow("tb_depth", roiImg);
    obs = detect(roiImg);
    if(obs != null)
    {
        int wRect = obs.width , hRect = obs.height;

        //rectangle(rgb, Point(obs.x, obs.y), Point(wRect, hRect), Scalar(0,0,255), 3);

        rect = Rect(obs.x - buW, obs.y - buH, wRect + 2*buW, hRect + 2*buH);

        if(rect.x < 0)
            rect.x = 0;

        if(rect.width + rect.x > rgb.size().width)
            rect.width = rgb.size().width - rect.x - 1;

        if(rect.y < 0)
            rect.y = 0;

        if(rect.height + rect.y > rgb.size().height)
            rect.height = rgb.size().height - rect.y - 1;

        obs_flag = true;
    }
    //imshow("obstacle", rgb);
    return rect;
}
void DetectObstacle::pubObstacle()
{
    std_msgs::Bool hasObs;
    hasObs.data = obs_flag;
    obsPub.publish(hasObs);
}
