#include "ControlCar.h"
ControlCar::ControlCar()
{
    steerPub = nodeObj1.advertise<std_msgs::Float32>(STEER_TOPIC,1);
    speedPub = nodeObj2.advertise<std_msgs::Float32>(SPEED_TOPIC,1);
}
ControlCar::~ControlCar(){}
Rect ControlCar::danger_zone(const Rect &obs)
{
   int area = 60;
   int wRect = obs.width , hRect = obs.height;
   Rect rect = Rect(0,0,0,0);
   if (obs != Rect(0,0,0,0))
	{
	    rect = Rect(obs.x - area, obs.y, obs.x+ wRect + area, hRect + 20);
	    if(rect.x < 0)
            	rect.x = 0;
	    if(rect.width + rect.x > IMG_W)
            	rect.width = IMG_W - rect.x - 1;
	    if(rect.y < 0)
            	rect.y = 0;
	    if(rect.height + rect.y > IMG_H)
            	rect.height = IMG_H - rect.y - 1;         
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
    
    int flag = 1;
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
        int centerX = (int)(midX / count);
        midY += i;
        int centerY = midY/count;
        //cout << "midX: " << midX/count << "\t";
        
        if (obs != Rect(0,0,0,0))
	    if (centerX > obs.x && centerX < (obs.x + obs.width) && (obs.y > 40))
		midY = 225;
            else
                midY = 190;
        else    
             if ((midX / count) < 159 || (midX / count) > 161)
                midY = 200;
             else
                midY = 140;   
        //cout << "midX: " << midX/count << "\t";
        //cout << "midY: " << midY << "\t";
        //cout << "Rect" << obs << endl;
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
Point ControlCar::dangerPoint(const Mat &bin, int flag) //Don't use this function!!!
{

	int square = 3, margin = 30;
	int meanSquare = square * square;
	int border = int((square - 1) / 2);
	int i = IMG_H - border - 1;
	int minS = meanSquare * 0.5;
	int i_l = border;
	int i_r = IMG_W - 1 - border;
	bool turn_left = false;
	bool turn_right = false;
	float roi = 2 / 3;

	if(flag == 1 || flag == 2)
	{
	    turn_left = true;
	    turn_right = true;
	}
	while (i - border > 0)
	{
		Mat check = bin(Rect(int(IMG_W / 2) - border, i - border, 2*border + 1, 2*border + 1));
		int white = countNonZero(check);
		if (white < minS)
			break;
		i -= 1;
	}
	if ((i - border > 0) && i < int(IMG_H * roi))
	{
		i = int(IMG_H * roi);
	}
	if(flag != 1 && flag != 2)
	{
	    while (i_l <= IMG_W / 2)
	    {
		    Mat check = bin(Rect(i_l - border, i - border, 2*border + 1, 2*border + 1));
		    int white = countNonZero(check);
		    if (white > minS && i_l <= margin)
		    {
			    turn_left = true;
			    break;
		    }
		    else if (white > minS and i_l > margin)
		    {
			    break;
		    }
		    else
			    i_l += (border + 1);
	        }
	    while (i_r > IMG_W / 2)
	    {
		    Mat check = bin(Rect(i_r - border, i - border, 2 * border + 1, 2 * border + 1));
		    int white = countNonZero(check);
		    if (white > minS && i_r >= IMG_W - margin)
		    {
			    turn_right = true;
			    break;
		    }
		    else if (white > minS and i_r < IMG_W - margin)
		    {
			    break;
		    }
		    else
			    i_r -= (border + 1);
	    }
	}
	if (turn_left == false && turn_right == false)
	{
		return Point(int((i_l + i_r) / 2), i);
	}
	else if (turn_left == true && turn_right == true)
	{
		if (flag == 2) //Turn right
		{
			while (bin.at<uchar>(i, i_r) == 255 && i > 0)
			{
				i -= 1;
			}
			return Point(i_r, i);
		}
		else if (flag == 1) //Turn left
		{
			while (bin.at<uchar>(i, i_l) == 255 && i > 0)
			{
				i -= 1;
			}
			return Point(i_l, i);
		}
		else
		{
			return Point(int((i_l + i_r) / 2), i);
		}

	}
	else if (turn_left == true)
	{
		while (bin.at<uchar>(i, i_l) == 255 && i > 0)
			i -= 1;
		return Point(i_l, i);
	}
	else
	{
		while (bin.at<uchar>(i, i_r) == 255 && i > 0)
			i -= 1;
		return Point(i_r, i);
	}


}
float ControlCar::getSteer(const Point &p)
{
    float dx = p.x - IMG_W/2 + 1;
    float dy = IMG_H - (float)p.y;
    float steer = atan(dx/dy) * 57.32; // = 180/PI
    return steer;
}
float ControlCar::dynamicSpeed(const float &velocity, const float &steer)
{
    return velocity * cos(abs(steer) * 0.0174); // = PI/180
}
float ControlCar::pid(const float &cte)
{
    error_i += cte;
    error_d = cte - error_p;
    error_p = cte;
    return (k_p * error_p + k_i * error_i + k_d * error_d);
}
void ControlCar::pubSteerAndSpeed(const float &velocity, const float &errorAngle)
{
    std_msgs::Float32 steer, speed;
    steer.data = errorAngle;
    speed.data = velocity;

    steerPub.publish(steer);
    speedPub.publish(speed);
}
void ControlCar::driveCar(const Mat &view, float velocity,int flag, bool flag2, Rect rect)
{
    float errorAngle , errorSpeed;
    Point center(0,0);//Initial point to control
    Mat dst = view.clone();

    center = getPoint2(dst, rect);


    if(flag2 == true)
    {
       current_flag = true; 
       begin = ros::Time::now().toSec();   
    }
    if(current_flag == true)
    {
        double end = ros::Time::now().toSec();
        if(end - begin <= 3 && end - begin > 0.05)
        {
                center = dangerPoint(dst, flag);
        }
        else if(end - begin > 3)
        {
            begin = 0;
            current_flag = false;
        }
    }

    
    line(dst, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    imshow("steer", dst);



    //errorAngle = getSteer(center)*0.72 - preSteer*0.28;
    //preSteer = errorAngle;
    errorAngle = pid(center.x - IMG_W/2 + 1);
    errorSpeed = dynamicSpeed(velocity, errorAngle);
    /*if(flag2 == true)
    {
        if(flag == 1)
        {
            errorAngle = -20.0;
            errorSpeed = 40;
        }
        else if(flag == 2)
        {
            errorAngle = 20.0;
            errorSpeed = 40;
        }
        pubSteerAndSpeed(errorSpeed, errorAngle);
        sleep(2);
    }*/

    pubSteerAndSpeed(errorSpeed, errorAngle);

}
