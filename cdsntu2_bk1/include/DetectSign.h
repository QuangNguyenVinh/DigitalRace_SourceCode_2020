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
    vector<bool> detectedContours;
    vector<Rect> rects;

    Rect rectSign = Rect(0,0,0,0);
    bool detect(const Mat &binImg);
    int useHOG_SVM(const Mat &grayImg);

    ros::NodeHandle signNode;

    ros::Publisher signPub;

public:
    DetectSign(const string  svmModel);
    int update(const Mat &src);
    Rect draw();
    void signClassify(const Mat &src);
};


#endif //DETECTSIGN_H
