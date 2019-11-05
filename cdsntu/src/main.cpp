#include "header.h"
#include "DetectLane.h"
#include "ControlCar.h"
#include "DetectSign.h"
#include "DetectObstacle.h"
DetectLane *lane;
ControlCar *car;
DetectSign *sign;
DetectObstacle *obstacle;

string path = ros::package::getPath("cdsntu");
string svmModel = path + "/model/svm.xml";
Mat rgbImg(240, 320, CV_8UC3, Scalar(0,0,0)), depthImg;

/* Dirty code */
vector<int> flag1;
bool flag2 = false;
int decision ;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    int _turn ;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //cv::imshow("View", cv_ptr->image);
        lane->detectLane(cv_ptr->image).copyTo(out);
        cv_ptr->image.copyTo(rgbImg);
	    sign->signClassify(cv_ptr->image);
	    /*Dirty code */
	    _turn = sign->update(cv_ptr->image);
        if(_turn == 1 || _turn == 2)
        {
            flag1.push_back(1);
            decision = _turn;
        }
        else flag1.push_back(0);

        if(flag1.size()>2)
            if(flag1[flag1.size()-1] == 0 && flag1[flag1.size()-2] == 1) flag2 = true;
            else flag2 = false;


        car->driveCar(out, velocity,decision, flag2);
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
        obstacle->showObj(out, rgbImg);


        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Depth Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cdsntu");
    cv::namedWindow("steer");
    cv::namedWindow("sign");
    cv::namedWindow("Threshold Sign");
    cv::namedWindow("Threshold");
    cv::namedWindow("Depth");
    cv::namedWindow("DepthBin");
    cv::namedWindow("obstacle");

    lane = new DetectLane();
    car = new ControlCar();
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
