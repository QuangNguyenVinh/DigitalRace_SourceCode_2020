//
// Created by quang on 11/10/2019.
//

#include "DetectSign.h"

DetectSign::DetectSign(const string svmModel)
{
    hog = HOGDescriptor(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 9);
    svm = SVM::load(svmModel);
    if(show_val)
    {
        cvCreateTrackbar("LowH", "tb_sign", &minThresholdSign[0], 179);
        cvCreateTrackbar("HighH", "tb_sign", &maxThresholdSign[0], 179);

        cvCreateTrackbar("LowS", "tb_sign", &minThresholdSign[1], 255);
        cvCreateTrackbar("HighS", "tb_sign", &maxThresholdSign[1], 255);

        cvCreateTrackbar("LowV", "tb_sign", &minThresholdSign[2], 255);
        cvCreateTrackbar("HighV", "tb_sign", &maxThresholdSign[2], 255);
    }


    signPub = signNode.advertise<std_msgs::Float32>(SIGN_TOPIC,1);
}
int DetectSign::classifySVM(const Mat &grayImg, const Rect rect)
{
    Mat graySign, grayClone = grayImg.clone();
    //GaussianBlur(grayClone, grayClone, Size(3,3), 2, 2);
    vector<Vec3f> circles(0);
    resize(grayImg(rect),graySign, Size(32,32));
	
    //Change constrast
    float alpha = 2, beta = 50;
    graySign.convertTo(graySign, -1, alpha, beta);
    //Remove noise
    GaussianBlur(graySign, graySign, Size(3,3), 2, 2);
    if(show_val)
    {
    	imshow("Error", graySign);   
    }

    //Compute HOG
    vector<float> descriptors;
    hog.compute(graySign, descriptors);

    Mat fm(descriptors, CV_32F);

    return static_cast<int>(svm->predict(fm.t()));

}
int DetectSign::detect(const Mat &binImg, const Mat &grayImg)
{
    contours.clear();

    //Find contours;
    vector<Vec4i> hierarchy;
    findContours(binImg, contours, hierarchy, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    int imgArea = binImg.size().width * binImg.size().height;


    for(size_t i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);
	
        double rectPerFrame = static_cast<double>(rect.area())/(imgArea);
        double ratio = static_cast<double>(rect.width)/(rect.height);

        if(rectPerFrame > MIN_AREA)
            if(ratio >= 0.6 && ratio <= 1.4)
                {
                    int flag = classifySVM(grayImg, rect);
                    if(flag == 1 || flag == 2)
                    {
                        rectSign = rect;
                        return flag;
                    }
                }

    }
    return 0;
}
int DetectSign::update(const Mat &src)
{
    
    Mat hsvImg, grayImg;
    Mat dst = src.clone();

    cvtColor(dst, hsvImg, CV_BGR2HSV);
    cvtColor(dst, grayImg, CV_BGR2GRAY);

    Mat binImg;
    inRange(hsvImg, Scalar(minThresholdSign[0], minThresholdSign[1], minThresholdSign[2]) ,
            Scalar(maxThresholdSign[0], maxThresholdSign[1], maxThresholdSign[2] ),binImg);
    if(show_val)
    {
    	imshow("tb_sign", binImg);
    }

    return detect(binImg, grayImg);

}
Rect DetectSign::draw()
{
    return rectSign;
}
void DetectSign::signClassify(const Mat &src)
{
    std_msgs::Float32 sign;
    sign.data = (float)(update(src));
    signPub.publish(sign);
}
