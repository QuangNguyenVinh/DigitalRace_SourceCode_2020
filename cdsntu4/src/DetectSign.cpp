//
// Created by quang on 11/10/2019.
//

#include "DetectSign.h"
DetectSign::DetectSign(const string svmModel){
    hog = HOGDescriptor(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 9);
    svm = SVM::load(svmModel);
}

int DetectSign::UpdateFromCircle(const Mat &src, vector<Vec3f> circles){
    rects.clear();
    rects.resize(circles.size());
    for(int i = 0; i < circles.size(); i++ ){
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        int radius = c[2];
        rects[i] = Rect(center.x - radius, center.y - radius, 2 * radius, 2 * radius);
    }
    Mat hsvImg, grayImg;
    Mat dst = src.clone();

    cvtColor(dst, hsvImg, CV_BGR2HSV);
    cvtColor(dst, grayImg, CV_BGR2GRAY);

    return useHOG_SVM(grayImg);

}
int DetectSign::useHOG_SVM(const Mat &grayImg)
{
    int flag = 0;
    for(size_t i = 0; i < rects.size(); i++)
    {
        Rect rect = rects[i];

        Mat graySign;
        /*if (rect.x < 0 || rect.y < 0 || rect.x - rect.width < 0 || rect.y - rect.height < 0 ||
            rect.x + rect.width > 320 || rect.y + rect.height > 240)
            continue;*/
        resize(grayImg(rect),graySign, Size(32,32)); //Resize gray image to fit model
        
        //Compute HOG
        vector<float> descriptors;
        hog.compute(graySign, descriptors);

        Mat fm(descriptors, CV_32F);

        int classID = static_cast<int>(svm->predict(fm.t()));

        if(classID == 1) //Turn left sign
            flag = 1;

        if(classID == 2) //Turn right sign
            flag = 2;

        if(flag != 0)
        {
            rectSign = rect;
            return flag;
        }
    }
    return flag;
}
Rect DetectSign::draw()
{
    return rectSign;
}
