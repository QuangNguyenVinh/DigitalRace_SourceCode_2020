#ifndef DETECTTREE_H
#define DETECTTREE_H

#include "header.h"
class DetectTree
{
private:
    int min[3] =  {24,110,22};
    int max[3] =  {39,216,100};
    Mat processImg(const Mat &src);
    int getMaxAreaContourId(vector <vector<cv::Point>> contours);
    Mat dilateLane(const Mat &src);
    Mat roi(const Mat &src);
    vector<vector<Point>> treeContours = {{Point(0,0)}};
public:
    vector<vector<Point>> findTree(const Mat &src);
};


#endif //DETECTTREE_H