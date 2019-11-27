#include "header.h"
#include "DetectSign.h"
#include "DetectObstacle.h"
DetectSign *sign;
DetectObstacle *obstacle;

string path = ros::package::getPath("roadml");
string svmModel = path + "/model/svm.xml";
Mat rgbImg(240, 320, CV_8UC3, Scalar(0,0,0)), depthImg;

/* Dirty code */
Rect rect = Rect(0,0,0,0);

// VideoWriter depth("/home/transon/output_depth.avi",CV_FOURCC('M','J','P','G'),10, Size(320,240));
// VideoWriter video("/home/transon/output_rgb.avi",CV_FOURCC('M','J','P','G'),10, Size(320,240));

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    int _turn ;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(rgbImg);
        out = cv_ptr->image.clone();
        rectangle(out, rect, Scalar(0,0,255));
	    /*Dirty code */
	    _turn = sign->update(cv_ptr->image);
        //cout<<_turn<<endl;
        if(_turn == 1 || _turn == 2)
        {
            //cout<<"c_data: " << _turn <<endl;
	        sign->signClassify();
            rectangle(out, sign->draw(), Scalar(255,0,0));
            putText(out, ((_turn == 1)?"turn_left":"turn_right"),Point(sign->draw().x,sign->draw().y),CV_FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(255,0,0));
        }
	    sign->signClassify();
        cv::imshow("View", out);
	    waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(depthImg);
        out = cv_ptr->image.clone();
        rect = obstacle->showObj(out, rgbImg);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Depth Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "roadml");
    cv::namedWindow("sign");
    cv::namedWindow("View");
    cv::namedWindow("DepthBin");

    sign = new DetectSign(svmModel);
    obstacle = new DetectObstacle();
    if (true) 
    {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(IMAGE_RGB_TOPIC, 1, imageCallback);

        image_transport::ImageTransport dt(nh);
        image_transport::Subscriber subd = dt.subscribe(IMAGE_DEPTH_TOPIC, 1, depthCallback);

        ros::spin();
    } 
    cv::destroyAllWindows();
}
