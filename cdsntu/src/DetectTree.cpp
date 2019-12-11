#include "DetectTree.h"

Mat DetectTree::processImg(const Mat &src)
{
    Mat hsv, tree;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(min[0], min[1], min[2]), Scalar(max[0], max[1], max[2]), tree);
    tree = roi(tree);
    tree = dilateLane(tree);
    return tree;
}

Mat DetectTree::roi(const Mat &src)
{
    Mat dst;
    int h = src.rows, w = src.cols;
    Mat mask = Mat::zeros(src.size(), src.type());

    Point pts[6] = 
    {
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

Mat DetectTree::dilateLane(const Mat &src)
{
    Mat dst;
    int dilation_size = 3;
    cv::Mat element = cv::getStructuringElement(MORPH_CROSS, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
    dilate(src, dst, element);
    return dst;
}

int DetectTree::getMaxAreaContourId(vector <vector<cv::Point>> contours) 
{
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) 
    {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) 
        {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
}

vector<vector<Point>> DetectTree::findTree(const Mat &src)
{
    Mat tree = src.clone();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int index;
    Mat temp = processImg(tree);
    findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_THRESH_TRUNC, Point(0,0));
    if(contours.size() > 0)
    {
        treeContours.clear();
        index = getMaxAreaContourId(contours);
        treeContours.push_back(contours[index]);
    }
    drawContours(tree, treeContours, 0, Scalar(255,0,0), -1);
    imshow("tree", tree);
    return treeContours;
}