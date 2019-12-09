//
// Created by quang on 11/10/2019.
//

#include "DetectSign.h"
Mat DetectSign::cutROI(const Mat &src)
{
    Mat dst;
    int h = src.rows, w = src.cols;
    Mat mask = Mat::zeros(src.size(), src.type());


    Point pts[4] = {
            Point(0, h),
            Point(100, 80),
            Point(w - 100, 80),
            Point(w, h),
    };

    Point pts2[6] = {
            Point(0, h),
	    Point(0, (int)(120)),
            Point(60, 60),
            Point(w - 60, 60),
	    Point(w, (int)(120)),
            Point(w, h),
    };

    fillConvexPoly(mask, pts2, 6, Scalar(255));
    bitwise_and(src, mask, dst);

    return dst;h*3/4;
}
DetectSign::DetectSign(const string svmModel)
{
    hog = HOGDescriptor(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 9);
    svm = SVM::load(svmModel);

    cvCreateTrackbar("LowH", "Threshold Sign", &minThresholdSign[0], 179);
    cvCreateTrackbar("HighH", "Threshold Sign", &maxThresholdSign[0], 179);

    cvCreateTrackbar("LowS", "Threshold Sign", &minThresholdSign[1], 255);
    cvCreateTrackbar("HighS", "Threshold Sign", &maxThresholdSign[1], 255);

    cvCreateTrackbar("LowV", "Threshold Sign", &minThresholdSign[2], 255);
    cvCreateTrackbar("HighV", "Threshold Sign", &maxThresholdSign[2], 255);

    signPub = signNode.advertise<std_msgs::Float32>(SIGN_TOPIC,1);
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

    Mat binImg;
    inRange(hsvImg, Scalar(minThresholdSign[0], minThresholdSign[1], minThresholdSign[2]) ,
            Scalar(maxThresholdSign[0], maxThresholdSign[1], maxThresholdSign[2] ),binImg);
    //rectangle(binImg, Rect(0,0,320,60), Scalar(0,0,0), -1);
    imshow("sign", binImg);
    return useHOG_SVM(grayImg);

}

bool DetectSign::detect(const Mat &binImg)
{
    contours.clear();
    detectedContours.clear();
    rects.clear();

    //Find contours;
    vector<Vec4i> hierarchy;
    findContours(binImg, contours, hierarchy, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    //Re-arrange detected contours from largest to smallest
    if(contours.size())
    {
        sort(contours.begin(), contours.end(),[](const vector<Point> &p1, const vector<Point> &p2)
        {
            Rect r1 = boundingRect(p1);
            Rect r2 = boundingRect(p2);

            return r1.area() > r2.area();
        });
    }
    detectedContours.resize(contours.size());
    //Fill vector by all false-value
    fill(detectedContours.begin(), detectedContours.end(), false);

    rects.resize(contours.size());

    bool detected = false;

    for(size_t i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);

        double ellipseArea = PI * (rect.width/2) * (rect.height/2);
        double area = contourArea(contours[i]);
        rects[i] = rect;

        double boundWperH = static_cast<double>(rect.width)/rect.height;
        double areaPerEllipse = static_cast<double>(area) / ellipseArea;
        double rectPerFrame = static_cast<double>(rect.area())/(binImg.size().width * binImg.size().height);
        //If detected contour is similar with ellipse or circle means it could be sign
        if(rectPerFrame > MIN_AREA)
            if( 0.5 < boundWperH && boundWperH < 1.5)
                if( 0.6 < areaPerEllipse && areaPerEllipse < 1.4)
                {
                    detectedContours[i] = true;
                    
                    detected = true;
                }

    }
    return detected;
}
Rect DetectSign::getRect(const Mat &binImg)
{
    contours.clear();
    detectedContours.clear();
    rects.clear();
    Rect rectsign = Rect(0,0,0,0);
    //Find contours;
    vector<Vec4i> hierarchy;
    findContours(binImg, contours, hierarchy, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    //Re-arrange detected contours from largest to smallest
    if(contours.size())
    {
        sort(contours.begin(), contours.end(),[](const vector<Point> &p1, const vector<Point> &p2)
        {
            Rect r1 = boundingRect(p1);
            Rect r2 = boundingRect(p2);

            return r1.area() > r2.area();
        });
    }
    detectedContours.resize(contours.size());
    //Fill vector by all false-value
    fill(detectedContours.begin(), detectedContours.end(), false);

    rects.resize(contours.size());

    bool detected = false;

    for(size_t i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);

        double ellipseArea = PI * (rect.width/2) * (rect.height/2);
        double area = contourArea(contours[i]);
        rects[i] = rect;

        double boundWperH = static_cast<double>(rect.width)/rect.height;
        double areaPerEllipse = static_cast<double>(area) / ellipseArea;
        double rectPerFrame = static_cast<double>(rect.area())/(binImg.size().width * binImg.size().height);
        //If detected contour is similar with ellipse or circle means it could be sign
        if(rectPerFrame > MIN_AREA)
            if( 0.5 < boundWperH && boundWperH < 1.5)
                if( 0.6 < areaPerEllipse && areaPerEllipse < 1.4)
                {
                    detectedContours[i] = true;
                    rectsign = rects[i];
                    
                }

    }
    return rectsign;
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
Rect DetectSign::getCenterSign(const Mat &src)
{
    Mat hsvImg, grayImg;
    Mat dst = src.clone();

    cvtColor(dst, hsvImg, CV_BGR2HSV);
    cvtColor(dst, grayImg, CV_BGR2GRAY);

    Mat binImg;
    inRange(hsvImg, Scalar(minThresholdSign[0], minThresholdSign[1], minThresholdSign[2]) ,
            Scalar(maxThresholdSign[0], maxThresholdSign[1], maxThresholdSign[2] ),binImg);
    cutROI(binImg);
    
    return getRect(binImg);
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
    rectangle(binImg, Rect(0,0,320,60), Scalar(0,0,0), -1);
    imshow("sign", binImg);
    
    if(detect(binImg))
        return useHOG_SVM(grayImg);
    else return 0;
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
