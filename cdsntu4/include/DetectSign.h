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
    vector<Rect> rects;
    Rect rectSign = Rect(0,0,0,0);
    int useHOG_SVM(const Mat &grayImg);

public:
    DetectSign(const string svmModel);
    int UpdateFromCircle(const Mat &src, vector<Vec3f> circles);
    Rect draw();
};


#endif //DETECTSIGN_H
