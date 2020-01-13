#include "ControlCar.h"
ControlCar::ControlCar()
{
    steerPub = nodeObj1.advertise<std_msgs::Float32>(STEER_TOPIC,1);
    speedPub = nodeObj2.advertise<std_msgs::Float32>(SPEED_TOPIC,1);
}
ControlCar::~ControlCar(){}
Mat ControlCar::cutROI(const Mat &src)
{
    Mat dst;
    int h = src.rows, w = src.cols;
    Mat mask = Mat::zeros(src.size(), src.type());


    Point pts[4] = {
            Point(0, h),
            Point(100, 100),
            Point(w - 100, 100),
            Point(w, h),
    };

    Point pts2[6] = {
            Point(0, h),
            Point(0, (int)(h*3/4)),
            Point(100, 100),
            Point(w - 100, 100),
            Point(w, (int)(h*3/4)),
            Point(w, h),
    };

    fillConvexPoly(mask, pts2, 6, Scalar(255));
    bitwise_and(src, mask, dst);

    return dst;
}
int ControlCar::getArea(const Mat &bin)
{
    Mat dst = bin(Rect(0,130, 320,60));
    return countNonZero(dst);
}
Rect ControlCar::danger_zone(const Rect &obs)
{
   int area = 60;
   int wRect = obs.width , hRect = obs.height;
   Rect rect = Rect(0,0,0,0);
   if (obs != Rect(0,0,0,0))
	{
	    rect = Rect(obs.x - area, obs.y, obs.x+ wRect + area, hRect + 20);       
	}
      
   return rect;
}
Point ControlCar::getPoint(const Mat &src)
{
    int midX = 0, midY = 0;
    int count = 0;
    int limit = (int) (IMG_H * 1 / 3);
    for (int i = limit; i < IMG_H; i++)
    {
        count += 1;
        int left = 0;
        while (src.at<uchar>(i, left) != 255 && left < IMG_W / 2)
        {
            left += 1;
        }
        int right = IMG_W - 1;
        while (src.at<uchar>(i, right) != 255 && right >= IMG_W / 2)
        {
            right -= 1;
        }
        midX += (int) ((left + right) / 2);
        midY += i;
    }
    return Point((int) (midX / count), (int) (midY / count));
}
Point ControlCar::getPoint2(const Mat &src, Rect rect)
{
    Rect obs = Rect(0,0,0,0);
    obs = danger_zone(rect);

    int midX = 0, midY = 0;
    int count = 0;
    int limit = (int) (IMG_H * 1 / 5);
    for (int i = limit; i < IMG_H - 100; i++)
    {
        count += 1;
        int left = 0;
        while (src.at<uchar>(i, left) != 255 && left < IMG_W / 2)
        {
            left += 1;
        }
        int right = IMG_W - 1;
        while (src.at<uchar>(i, right) != 255 && right >= IMG_W / 2)
        {
            right -= 1;
        }
        midX += (int) ((left + right) / 2);
	    midY += i;
  
    	int centerX = (int)(midX / count);

    	if (obs != Rect(0,0,0,0))
       		if (centerX > obs.x && centerX < (obs.x + obs.width) && (obs.y > 40))
	   		midY = 225;
       		else
           		midY = 190;
   	    else    
       		if ((midX / count) < 159 || (midX / count) > 161)
           		midY = 220;
       		else
           		midY = 140;

    }

        
 

    return Point((int) (midX / count), (int) (midY));
}
Point ControlCar::getPoint3(const Mat &bin) //Don't use this function!!!
{
    int midX = 0, midY = 0, left = 0, right = 0, count = 0;
    int center = IMG_W / 2;
    int limit = (int) (IMG_H * 1 / 3);
    for (int i = limit; i < IMG_H ; i++) //160
    {
        count += 1;
        left = 0, right = 0;
        for(int j = center; j > 0; j--)
        {
            if (bin.at<uchar>(i, j) == 255)
                left += 1;
        }
        for(int k = center; k < IMG_W; k++)
        {
            if (bin.at<uchar>(i, k) == 255)
                right += 1;
        }

        midX += (center - (left - right));
        midY += i;
    }
    return Point((int) (midX / count), (int) (midY / count));
}
float ControlCar::getSteer(const Point &p)
{
    float dx = p.x - IMG_W/2 + 1;
    float dy = IMG_H - (float)p.y;
    float steer = atan(dx/dy) * 57.32; // = 180/PI
    
    if(steer < -60)
        return -60.0f;
    if(steer > 60)
        return 60.0f;
    return steer;
}
float ControlCar::dynamicSpeed(const float &v, const float &steer)
{
    return v * cos(abs(steer) * 0.0174); // = PI/180
}
float ControlCar::pid(const float &cte)
{
    error_i += cte;
    error_d = cte - error_p;
    error_p = cte;
    return k_p * error_p + k_i * error_i + k_d * error_d;
}
void ControlCar::pubSpeed(const float &veloc)
{
    std_msgs::Float32 speed;
    speed.data = veloc;
    speedPub.publish(speed);
}
void ControlCar::pubSteer(const float &angle)
{
    std_msgs::Float32 steer;
    steer.data = angle;
    steerPub.publish(steer);

}
void ControlCar::driveCar(const Mat &view, int flag, bool flag2, Rect rect)
{
    float errorAngle = 0, errorSpeed = 60;
    Point center(0,0);//Initial point to control
    Mat dst = view.clone(), dst2 = view.clone();
    int white = getArea(dst);

    dst2 = cutROI(dst2).clone();

    cout << "White: " << white << endl;
    if(flag2 == true)
    {
        current_flag = true;
        errorSpeed = 40;
        pubSpeed(errorSpeed);
    }
    if (current_flag == true && white > 15300)
    {
	   
            errorSpeed = 40;
            if(flag == 1)
                errorAngle = -30;
            else if(flag == 2)
                errorAngle = 40;
            pubSpeed(errorSpeed);
            pubSteer(errorAngle);
            current_flag = false;
            ros::Duration(1.1).sleep();    
    }
    /*if(flag2 == true)
    {
	if(flag == 1)
	    errorAngle = -30;
        else if(flag == 2)
            errorAngle = 40;
        pubSpeed(errorSpeed);
        pubSteer(errorAngle);
        ros::Duration(1.1).sleep();
    }*/    
    center = getPoint2(dst2, rect);
    errorAngle = pid(center.x - IMG_W/2 + 1);

    line(dst2, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    if(show_val)
	imshow("steer", dst);


    //errorAngle = pid(getSteer(center));
    //errorAngle = getSteer(center)*0.72 - preSteer*0.28;
    //preSteer = errorAngle;


    	


    pubSpeed(errorSpeed);
    pubSteer(errorAngle);
	  
}
