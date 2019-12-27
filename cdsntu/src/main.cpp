#include "header.h"
#include "DetectLane.h"
#include "ControlCar.h"
#include "DetectSign.h"
#include "DetectObstacle.h"
#include "DetectTree.h"

DetectLane *lane;
ControlCar *car;
DetectSign *sign;
DetectObstacle *obstacle;
DetectTree *tree;

string path = ros::package::getPath(TEAM_NAME);
string svmModel = path + "/model/svm.xml";
string maskSrc = path + "/model/mask.png";

Rect rect = Rect(0,0,0,0); //Find obstacle from depth image
vector<Vec3f> circles(0); //Find circles sign
vector<vector<Point>> treeContours = {{Point(0,0)}}; //Find tree from depth image
/* Dirty code */
vector<int> flag1;
bool flag2 = false, flag3 = false;
int decision = 0;
/* End dirty code */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out, view;
    int _turn = 0, _turn2 = 0 ;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        view = cv_ptr->image.clone();

        //Process lane
        lane->updateLane(cv_ptr->image, rect).copyTo(out);

        //Process tree obstacle
        treeContours = tree->findTree(cv_ptr->image);

        //Publish sign data
        sign->signClassify(cv_ptr->image, circles);

	    /*Dirty code */
	    _turn = sign->update(cv_ptr->image, circles);
        Mat graySign;
        cvtColor(cv_ptr->image, graySign, CV_BGR2GRAY);
        _turn2 = sign->classifyByDepth(graySign, circles);
        cout << "Depth sign: " << _turn2 << endl;
        rectangle(view, rect, Scalar(255,0,0)); //Obstacles

        if(_turn == 1 || _turn == 2)
        {
            flag3 = true;
            flag1.push_back(1);
	    decision = _turn;
            rectangle(view, sign->draw(), Scalar(255,0,0));
            putText(view, ((_turn == 1)?"left":"right"),Point(sign->draw().x,sign->draw().y),CV_FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,0,255));
        }
        else flag1.push_back(0);
        flag2 = false;
        if(flag1.size() > 2)
        {
            if(flag1[flag1.size()-1] == 0 && flag1[flag1.size()-2] == 1) 
            {
                flag2 = true;
            }
            else flag2 = false;
        }

        if(flag2 == true)
	    {
		    flag1.clear();
		    flag3 = false;
	    }
        cv::imshow("View", view);
        /*end dirty code*/
        //cout << "Turn: " << _turn << " Flag: " << flag2 << " Decision: " << decision << endl ;
        car->driveCar(out, decision, flag3, rect);
	    //waitKey(1);
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
        out = cv_ptr->image.clone();
        rect = obstacle->showObj(out, treeContours);
        circles = obstacle->findRectSign(out);
        //Publish obstacle data
        obstacle->pubObstacle();
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
    //cv::namedWindow("tb_sign");
    //cv::namedWindow("tb_lane");
    //cv::namedWindow("Depth");
    //cv::namedWindow("RGB");
    //cv::namedWindow("tb_depth");
    //cv::namedWindow("obstacle");

    lane = new DetectLane();
    car = new ControlCar();
    sign = new DetectSign(svmModel);
    tree = new DetectTree();
    obstacle = new DetectObstacle(maskSrc);
    if (true) 
    {
        //cv::startWindowThread();

        ros::NodeHandle nh;

        image_transport::ImageTransport dt(nh);
        image_transport::Subscriber subd = dt.subscribe(IMAGE_DEPTH_TOPIC, 1, depthCallback);

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(IMAGE_RGB_TOPIC, 1, imageCallback);



        ros::spin();
    } 
    //cv::destroyAllWindows();
}
