#include "ControlCar.h"



ControlCar::ControlCar()
{
    steerPub = nodeObj1.advertise<std_msgs::Float32>(STEER_TOPIC,1);
    speedPub = nodeObj2.advertise<std_msgs::Float32>(SPEED_TOPIC,1);
}
ControlCar::~ControlCar(){}
int ControlCar::getArea(const Mat &bin)
{
    Mat dst = bin(Rect(0,150, 320,30));
    cout << "Area: " << countNonZero(dst) << endl;
    return countNonZero(dst);
}
bool ControlCar::checkLeftTurn(const Mat &bin, Rect leftRect)
{
    Mat left = bin(leftRect);
    if(show_val)
    	imshow("StopLeft", left);
    cout << "StopLeft: " << countNonZero(left) << endl;
    if(countNonZero(left) > 60 && countNonZero(left) < 300 )
        return true;
    return false;
}
bool ControlCar::checkRightTurn(const Mat &bin, Rect rightRect)
{
    Mat right = bin(rightRect);
    if(show_val)
    	imshow("StopRight", right);
    cout << "StopRight: " << countNonZero(right) << endl;
    if(countNonZero(right) > 60 && countNonZero(right) < 300)
        return true;
    return false;
}
bool ControlCar::checkLeft(const Mat &bin, Rect leftRect)
{
    Mat left = bin(leftRect);
    if(show_val)
    	imshow("Left-Turn", left);
    cout << "Left-Turn: " << countNonZero(left) << endl;
    if(countNonZero(left) >= 100 )
        return true;
    return false;
}
bool ControlCar::checkRight(const Mat &bin, Rect rightRect)
{
    Mat right = bin(rightRect);
    if(show_val)
    	imshow("Right-Turn", right);
    cout << "Right-Turn: " << countNonZero(right) << endl;
    if(countNonZero(right) >= 100 )
        return true;
    return false;
    
    
}
Rect ControlCar::danger_zone(const Rect obs)
{
   int area = 60;
   int wRect = obs.width , hRect = obs.height;
   Rect rect = Rect(0,0,0,0);
   if (obs != Rect(0,0,0,0))
      rect = Rect(obs.x - area, obs.y, obs.x+ wRect + area, hRect + 20);
   return rect;
}

Point ControlCar::getPoint(const Mat &src, Rect rect)
{
    Rect obs = Rect(0,0,0,0);
    obs = danger_zone(rect);
    
    int flag = 1;
    int midX = 0, midY = 0;
    int count = 0;
    int limit = (int) (IMG_H * 1 / 5);
    for (int i = limit; i < IMG_H - 100; i++) {
        count += 1;
        int left = 0;
        while (src.at<uchar>(i, left) != 255 && left < IMG_W / 2) {
            left += 1;
        }
        int right = IMG_W - 1;
        while (src.at<uchar>(i, right) != 255 && right >= IMG_W / 2) {
            right -= 1;
        }
        midX += (int) ((left + right) / 2);
        int centerX = (int)(midX / count);
        midY += i;
        int centerY = midY/count;
        //cout << "midX: " << midX/count << "\t";140
	    if (centerX > obs.x && centerX < (obs.x + obs.width) && (obs.y > 40))
		        midY = 225;
        else
                midY = 190;
            
             if ((midX / count) < 159 || (midX / count) > 161)
                midY = 220;
             else
                midY = 140;   
        //cout << "midX: " << midX/count << "\t";
        //cout << "midY: " << midY << "\t";
        //cout << "Rect" << obs << endl;
    }
    return Point((int) (midX / count), (int) (midY));
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

float ControlCar::getSteer(const Point &p)
{
    float dx = p.x - IMG_W/2 + 1;
    float dy = IMG_H - (float)p.y;
    float steer = atan(dx/dy) * 57.32; // 180/PI
    return steer;
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
void ControlCar::pubSpeedAndSteer(const float &veloc, const float &angle)
{
    std_msgs::Float32 steer, speed;
    steer.data = angle;
    speed.data = veloc;

    steerPub.publish(steer);
    speedPub.publish(speed);
}
void ControlCar::driveCar(const Mat &view, const Mat &view1, int flag, bool flag2, Rect obj)
{
    cout << "============DeBug=============" << endl;
    float errorAngle = 0 , errorSpeed;
    Point center(0,0);//Initial point to control
    Mat dst = view1.clone(), dst2 = view1.clone(), dst4 = view1.clone();
    int white = getArea(dst);
    
    bool leftTurn = checkLeftTurn(dst, Rect(10,90,80,15));
    bool rightTurn = checkRightTurn(dst, Rect(210, 80, 100, 15));
    bool left = checkLeft(dst2, Rect(0,125,40,20));
    int right = checkRight(dst4, Rect(280, 130, 40, 20));

    Mat dst3 = view.clone();
    center = getPoint(view, obj);
    line(dst, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    if(show_val)
    	imshow("steer", dst3);


    //errorAngle = getSteer(center)*0.7 - preSteer*0.3;
    errorAngle = pid(center.x - IMG_W/2 + 1);
    
   
    if(flag2 == true){
        temp = true;
        turn = flag;
        cout << "check-here-1" << endl;
        
    }
    if (right >= true)
    {
        flagRightTurn = true;
    }
    if (left == true)
    {
        flagLeftTurn = true;
    }
    if ( white > 6000)
    {
        point = white;
    }
    if (flagLeftTurn || flagRightTurn)
    {
        dem++;
    }
    
    if (white > 7000)
    {
        velocity = 60;
        errorAngle = errorAngle + 2;
    }else if (white > 6000 && white < 7000)
    {
        velocity = 60;
        errorAngle = errorAngle + 2 ;
    }
    else
    {
        velocity = 60;
        errorAngle = errorAngle + 2;
    }
    
    
    if (temp == true)
    {
        velocity = 30;
        pubSpeed(velocity);
        if (white < 9600)
        {
            velocity = 55;
            pubSpeed(velocity);
        }
        
        if(turn == 1 && flagLeftTurn == true){
            errorAngle = -30.0;
            velocity = SPEED_TURN;
            cout << "Turn-left!!!" << endl;
            pubSpeedAndSteer(SPEED_TURN, errorAngle);   
            if (leftTurn == true)
            {
                turn = 0;
                temp = false;
                point = 0;
                flagLeftTurn = false;
                demtemp = 0;
            } 

        }
        
        if (turn == 2 && flagRightTurn == true){
		    errorAngle = 30.0;
            velocity = SPEED_TURN;
            cout << "Turn-right!!!" << endl;
            pubSpeedAndSteer(velocity, errorAngle);
            if (rightTurn == true)
            {
                turn = 0;
                temp = false;
                point = 0;
                flagRightTurn = true;
                demtemp = 0;
            } 
        }
       
    }
    
    if (dem == 35 && flagRightTurn == true)
    {
        flagRightTurn = false;
        temp = false;
        dem = 0;
    } else if (dem == 35 && flagLeftTurn == true)
    {
        
        flagLeftTurn = false;
        temp = false;
        dem = 0;
    }
    
    
    cout << "CheckLeft: " << left << "\tCheckRight: "<< right << endl;
    cout << "FlagLeftTurn: " << flagLeftTurn << "\tFlagRightTurn: " << flagRightTurn << endl;
    cout << "CheckSign: " << temp << endl;
    cout << "Reset_flag(=35): " << dem << endl;
    cout << "Reset_temp(=50): " << demtemp << endl;
    cout << "==============================" << endl;
   
    /*if (temp == true && white > 15300)
    {
        velocity = -velocity;
        if(turn == 1){
                errorAngle = -30.0;
                

        }
        if (turn == 2){
		        errorAngle = 30.0;
                
        }
        steer.data = errorAngle;
        speed.data = velocity;
        steerPub.publish(steer);
        speedPub.publish(speed);
        cout << "Slowing" << endl;
        ros::Duration(1.2).sleep();
        temp = false;
    }*/
    /*if (flag3 == true && flag2 == false)
    {
        velocity = -velocity;
        if(turn == 1){
                errorAngle = -0.0;
                velocity = 20;

        }
        if (turn == 2){
		        errorAngle = 0.0;
                velocity = 20;
        }
        steer.data = errorAngle;
        speed.data = velocity;
        steerPub.publish(steer);
        speedPub.publish(speed);
        cout << "Slowing" << endl;
        ros::Duration(1.3).sleep();
        cout << speed << endl;
    }
   
    
    //cout << "Slowing" << endl;
    if(temp == true && flag2 == true)
    {
       if ( rectsign.y > 64 && rectsign.x >=270){
           cout << "checkpoint1" << endl;
           ros::Duration(0.7).sleep();
       
       }
       else if (rectsign.y > 64 && rectsign.x < 270) {
           cout << "checkpoint2" << endl;
           
           
           ros::Duration(0.0).sleep();
       }
       else {
           cout << "checkpoint3" << endl;velocity = 20;
       
        
       }
        if (index < frame)
        {
         index++;
         if(turn == 1){
            errorAngle = -25.0;
            velocity = 20;

        }
        if (turn == 2){
		    errorAngle = 25.0;
            velocity = 20;
        }
        steer.data = errorAngle;
	    speed.data = velocity;
        steerPub.publish(steer);
        speedPub.publish(speed);
		cout << turn << "\t" << index << "\n";
		if(index == frame){
		    temp = false;
		    turn = 0;
		    index = 0;
		}
        ros::Duration(0.4).sleep();//
        
        }
    }
    */
    
    

    pubSpeedAndSteer(velocity, errorAngle);
    
}
