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
    Mat dst = bin(Rect(0,135, 320,2));
    return countNonZero(dst);
}
int ControlCar::turnArea(const Mat &bin, int flagSign, int lowBound , int upperBound)
{
    int dem = 0;
    if(flagSign == 1)
    {
	    for(int i = lowBound; i <= upperBound; i++)
		    if( bin.at<uchar>(i,0) == 255)
            {
                dem++;
            }
		if(dem < (upperBound-lowBound) * 0.8)
            return 1; //Small
	    return 2; //Big
	}
    else if(flagSign == 2)
	{
	    for(int i = lowBound; i < upperBound; i++)
		    if(bin.at<uchar>(i, 320) == 255)
            {
                dem++;
            }
		if(dem < (upperBound-lowBound) * 0.8) 
            return 1; //Small    
	    return 2; //Big
	}
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
	if(flag == 1) 
	{    
	     turn_left = true;
        
	}
        else if(flag == 2)
        {
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
    
    if(steer < -60)
        return -60.0f;
    if(steer > 60)
        return 60.0f;
    return steer;
}
bool ControlCar::checkLeftTurn(const Mat &bin, Rect leftRect)
{
    Mat left = bin(leftRect);
    imshow("Left", left);
    cout << "Left: " << countNonZero(left) << endl;
    if(countNonZero(left) > (leftRect.area() * 0.8))
        return true;
    return false;
}
bool ControlCar::checkRightTurn(const Mat &bin, Rect rightRect)
{
    Mat right = bin(rightRect);
    imshow("Right", right);
    cout << "Right: " << countNonZero(right) << endl;
    if(countNonZero(right) > (rightRect.area() * 0.8))
        return true;
    return false;
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
    float pid = (k_p * error_p + k_i * error_i + k_d * error_d);
    if(pid > 60)
	return 60.0f;
    if(pid < -60)
        return -60.0f;
    return pid;


}
void ControlCar::pubSteerAndSpeed(const float &v, const float &errorAngle)
{
    std_msgs::Float32 steer, speed;
    steer.data = errorAngle;
    speed.data = v;

    steerPub.publish(steer);
    speedPub.publish(speed);
}
void ControlCar::driveCar(const Mat &view, int flag, bool flag2, Rect rect)
{
    float errorAngle , errorSpeed, velocity = 60, brake = 0;
    Point center(0,0);//Initial point to control
    Mat dst = view.clone(), dst2 = view.clone();
    int white = getArea(dst);

    
    
    
    bool leftTurn = checkLeftTurn(dst, Rect(0,130,40,50));
    bool rightTurn = checkRightTurn(dst, Rect(IMG_W - 40, 130, 40, 50));
    center = getPoint2(dst, rect);


    line(dst2, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    imshow("steer", dst);


    //errorAngle = pid(getSteer(center));
    //errorAngle = getSteer(center)*0.72 - preSteer*0.28;
    //preSteer = errorAngle;

    	

    errorSpeed = dynamicSpeed(velocity, errorAngle);

    if(flag2 == true)
    {
        current_flag = true;
        turnSign = flag;
        begin = ros::Time::now().toSec();

    }
    if(current_flag == false) 
    {
    	errorAngle = pid(center.x - IMG_W/2 + 1);
       	pubSteerAndSpeed(errorSpeed, errorAngle);
    }

    else if (current_flag == true)
    {
	    cout << "White: " << white << endl;
        errorSpeed = 40;
        if(white > 500)
        {
            if(turnSign == 1)
                errorAngle = -60;
            else if(turnSign == 2)
                errorAngle = 60;
            pubSteerAndSpeed(errorSpeed, errorAngle);
            isSleep = true;
        }
        else if(isSleep == true)
        {
            double time_to_sleep = 0;
            if(turnArea(dst, turnSign) == 1)
            {
                if(turnSign == 1)
                    errorAngle = -30;
                else if(turnSign == 2)
                    errorAngle = 30;
	            time_to_sleep = 1.0;
                cout << "Small Area \n";
	        }
	        else
            {
                if(turnSign == 1)
                    errorAngle = -20;
                else if(turnSign == 2)
                    errorAngle = 20;
                time_to_sleep = 0.5;
                cout << "Large Area \n";
	        }
            begin = 0;
            current_flag = false;
            turnSign = 0;
            isSleep = false;
	        pubSteerAndSpeed(errorSpeed, errorAngle);
            ros::Duration(time_to_sleep).sleep();
        }
        else if(isSleep == false)
        {
            center = getPoint2(dst, rect);
	        errorAngle = pid(center.x - IMG_W/2 + 1);
            pubSteerAndSpeed(errorSpeed, errorAngle);	
	}
        

    }





    	


}
