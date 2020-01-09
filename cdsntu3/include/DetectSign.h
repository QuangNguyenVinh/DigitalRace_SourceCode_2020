//
// Created by quang on 11/10/2019.
//


#ifndef DETECTSIGN_H
#define DETECTSIGN_H

#include "header.h"
class DetectSign
{
private:

    HOGDescriptor hog;
    Ptr<SVM> svm;
    int minThresholdSign[3] = {90, 40, 20};
    int maxThresholdSign[3] = {120, 255, 180};
    vector<vector<Point>> contours;


    Rect rectSign = Rect(0,0,0,0);
    int classifySVM(const Mat &grayImg, const Rect rect);  
    
    int detect(const Mat &binImg, const Mat &grayImg);

    ros::NodeHandle signNode;

    ros::Publisher signPub;

public:
    DetectSign(const string  svmModel);

    int update(const Mat &src);
    Rect draw();
    int classifyByDepth(const Mat &grayImg, const vector<Vec3f> circles);
    void signClassify(const Mat &src);
};


#endif //DETECTSIGN_H
