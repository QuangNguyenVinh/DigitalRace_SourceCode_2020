#include "header.h"
#include "DetectLane.h"
#include "ControlCar.h"
#include "DetectSign.h"
#include "DetectObstacle.h"
#include "DetectTree.h"
#include <string.h>
#include <stdlib.h>
DetectLane *lane;
ControlCar *car;
DetectSign *sign;
DetectObstacle *obstacle;
DetectTree *tree;

string path = ros::package::getPath(TEAM_NAME);
string svmModel = path + "/model/svm.xml";
string maskSrc = path + "/model/mask.png";
Mat rgbImg(240, 320, CV_8UC3, Scalar(0,0,0)), depthImg;
Rect rectObs = Rect(0,0,0,0);
vector<vector<Point>> treeContours = {{Point(0,0)}};
/* Dirty code */
bool flag2 = false;
bool flag3 = false;
int decision = 0;
vector<Vec3f> circles;

int frame = 1;
double start = 0;
float FPS = 0;
/* End dirty code */

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out, out1, view;
    int _turn = 0 ;
    try
    {
        double current = ros::Time::now().toSec();

        if(frame == 1){
            start = ros::Time::now().toSec();
        }
        if((current-start) >= 1.0){
            FPS = (frame-1)/(current-start);
            //cout << "frame: " << frame << "\tTime: " << (current-start)
            //    << "\tFPS: " << FPS <<endl;
            frame=0;
        } 
        frame++;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        view = cv_ptr->image.clone();
        lane->updateLane(cv_ptr->image, rectObs).copyTo(out);
        lane->noCutFinal(cv_ptr->image).copyTo(out1);
        cv_ptr->image.copyTo(rgbImg);
        treeContours = tree->findTree(cv_ptr->image);
        if(circles.size() != 0)
	        _turn = sign->UpdateFromCircle(cv_ptr->image, circles);
        
        rectangle(view, rectObs, Scalar(0,0,255)); //Obstacles
        
        if(_turn == 1 || _turn == 2)
        {   
	        decision = _turn;
            rectangle(view, sign->draw(), Scalar(255,0,0));
            putText(view, ((_turn == 1)?"left":"right"),Point(sign->draw().x,sign->draw().y),CV_FONT_HERSHEY_COMPLEX_SMALL,              0.8,Scalar(255,0,0));
            flag2 = true;
        }
        else {
            flag2 = false;
        }
        car->driveCar(out,out1, velocity,decision, flag2, rectObs, FPS);
        //imshow("View", view);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(depthImg);
        rectObs = obstacle->showObj(depthImg, treeContours);
        circles = obstacle->RectSign(depthImg);

        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Depth Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, TEAM_NAME);
    //cv::namedWindow("steer");
    //cv::namedWindow("threshDepth");
    //cv::namedWindow("View");

    lane = new DetectLane();
    car = new ControlCar();
    sign = new DetectSign(svmModel);
    tree = new DetectTree();
    obstacle = new DetectObstacle(maskSrc);
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