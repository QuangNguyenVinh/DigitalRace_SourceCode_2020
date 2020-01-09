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

Mat rgbImg(240, 320, CV_8UC3, Scalar(0,0,0));
Rect rect = Rect(0,0,0,0);
Rect _rectsign = Rect(0,0,0,0);
vector<vector<Point>> treeContours = {{Point(0,0)}};
/* Dirty code */
vector<int> flag1;
bool flag2 = false;
bool flag3 = false;
int decision = 0;
int check = 0;
string name = "";
int white = 0;
int white1 = 0;
vector<Vec3f> circles(0);
/* End dirty code */

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out, out1, view;
    int _turn = 0 ;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        view = cv_ptr->image.clone();
        lane->updateLane(cv_ptr->image, rect).copyTo(out);
        lane->noCutFinal(cv_ptr->image).copyTo(out1);
        cv_ptr->image.copyTo(rgbImg);
        treeContours = tree->findTree(cv_ptr->image);
        sign->signClassify(cv_ptr->image);
	    /*Dirty code */
        cout<<"circles size: " << circles.size() << endl;
        //if(circles.size() > 0)
	    _turn = sign->update(cv_ptr->image);
        
        //rectangle(view, rect, Scalar(0,0,255)); //Obstacles
        
        if(_turn == 1 || _turn == 2)
        {   
            
            name = to_string(check) + ".jpg";
            cout<< name <<endl;
            //imwrite(name, view);
            //cout << _rectsign << endl;
            flag1.push_back(1);
	        decision = _turn;
            rectangle(view, sign->draw(), Scalar(255,0,0));
            putText(view, ((_turn == 1)?"left":"right"),Point(sign->draw().x,sign->draw().y),CV_FONT_HERSHEY_COMPLEX_SMALL,              0.8,Scalar(255,0,0));
            //name = to_string(check) + "_sign.jpg";
            //imwrite(name, view);
            flag2 = true;
        }
        else 
        {
            flag1.push_back(0);
            flag2 = false;
        }
            
        
        
        //if(flag1.size()>2)
            //if(flag1[flag1.size()-1] == 0 && flag1[flag1.size()-2] == 1) flag2 = true;
            //else flag2 = false;
        if(flag2 == true)
            {
	            flag1.clear();
                
                //cout << "rectSign: " << _rectsign << endl;
                //cout << check << endl;
            }
      
        
        /*end dirty code*/
        
        //cout << "Turn: " << _turn << " Flag: " << flag2 << " Decision: " << decision << endl ;
        //cout<<white1<< "\t" << white << '\t' << flag2 << endl;
        car->driveCar(out,out1,decision, flag2, rect);

        check++;
        white = 0;
        white1 = 0;
        //video.write(view);
        if(show_val)
        {
        	imshow("View", view);
        }

        cout<<"---------------------\n";
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
        rect = obstacle->showObj(out, rgbImg, treeContours);
        circles = obstacle->RectSign(out);

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
    if(show_val){
    cv::namedWindow("steer");
    cv::namedWindow("sign");
    cv::namedWindow("Threshold Sign");
    cv::namedWindow("tb_lane");
    cv::namedWindow("threshImg");
    cv::namedWindow("DepthBin");
    cv::namedWindow("View");
    }
    lane = new DetectLane();
    car = new ControlCar();
    sign = new DetectSign(svmModel);
    tree = new DetectTree();
    obstacle = new DetectObstacle(maskSrc);
    if (true) 
    {
        //cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(IMAGE_RGB_TOPIC, 1, imageCallback);

        image_transport::ImageTransport dt(nh);
        image_transport::Subscriber subd = dt.subscribe(IMAGE_DEPTH_TOPIC, 1, depthCallback);

        ros::spin();
    } 
    //cv::destroyAllWindows();
}
