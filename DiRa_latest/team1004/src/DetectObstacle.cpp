#include "DetectObstacle.h"
Rect DetectObstacle::null = Rect();
DetectObstacle::DetectObstacle(string maskSrc)
{
    if(show_val){
    cvCreateTrackbar("value", "threshDepth", &value,255);
    }
    mask = imread(maskSrc, IMREAD_COLOR);
    cvtColor(mask, mask, COLOR_BGR2GRAY);
}
DetectObstacle::~DetectObstacle(){}
Mat DetectObstacle::processDepth(const Mat &depthImg)
{
    Mat dst;
    cvtColor(depthImg, dst, COLOR_BGR2GRAY);
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
            Point(80, 70),
            Point(w - 80, 70),
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
            if(rc.height >= height && rc.width >= width && rc.width <= 100 && rc.width < 240)
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

vector<Vec3f> DetectObstacle::RectSign(const Mat &depthImg){
    int _min = 10, _max = 318;
    vector<Vec3f> circles;
    Mat dst;
    Mat gray = processDepth(depthImg);
    absdiff(gray,mask, gray);
    Mat temp = Mat::zeros(gray.size(), gray.type());
    Point pts[4] = {
            Point(160, 50),
            Point(300, 50),
            Point(300, 95),
            Point(160, 95),
    };
    fillConvexPoly(temp, pts, 4, Scalar(255));
    bitwise_and(gray, temp, dst);
    GaussianBlur(dst,dst,Size(3,3), 2, 2);
    //imshow("debugSign", dst);

    HoughCircles(dst, circles, HOUGH_GRADIENT, 1,
                     gray.rows/6,  // change this value to detect circles with different distances to each other
                      (double)_max, (double)_min,2, 15 // change the last two parameters
                // (min_radius & max_radius) to detect larger circles
        );

    return circles;
}

Rect DetectObstacle::showObj(const Mat &depthImg, vector<vector<Point>> treeContours)
{
    Rect obs = null;
    Rect rect = Rect(0,0,0,0);
    Mat grayImg = processDepth(depthImg);
    Mat dst;
    absdiff(grayImg, mask, dst);
    if(show_val)
        imshow("Depth", dst);
    Mat threshImg = thresh(dst);
    Mat roiImg = roi(threshImg);
    drawContours(roiImg, treeContours, 0, Scalar(0), -1);
    //imshow("threshDepth", roiImg);
    obs = detect(roiImg);
    if(obs != null){
        int wRect = obs.width , hRect = obs.height;

        rect = Rect(obs.x - buW, obs.y - buH, wRect + 2*buW, hRect + 50);

        rect.x = max(0, rect.x);
        rect.y = max(0, rect.y);
        rect.width = min(depthImg.size().width - 1 - rect.x, rect.width);
        rect.height = min(depthImg.size().height - 1 - rect.y, rect.height);
    }
    return rect;
}
