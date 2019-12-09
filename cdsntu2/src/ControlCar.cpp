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
    imshow("Left", left);
    cout << "Left: " << countNonZero(left) << endl;
    if(countNonZero(left) > 100 && countNonZero(left) < 300 )
        return true;
    return false;
}
bool ControlCar::checkRightTurn(const Mat &bin, Rect rightRect)
{
    Mat right = bin(rightRect);
    imshow("Right", right);
    cout << "Right: " << countNonZero(right) << endl;
    if(countNonZero(right) > 60&& countNonZero(right) < 300)
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

void ControlCar::driveCar(const Mat &view, const Mat &view1,float velocity,int flag, bool flag2, Rect obj, Rect sign)
{
    
    
    float errorAngle , errorSpeed;
    Point center(0,0);//Initial point to control
    Mat dst = view1.clone(), dst2 = view1.clone();
    int white = getArea(dst);
    
    bool leftTurn = checkLeftTurn(dst, Rect(20,80,100,15));
    bool rightTurn = checkRightTurn(dst, Rect(200, 80, 100, 15));
    
    Rect rectsign = sign;

    Mat dst3 = view.clone();
    center = getPoint(view, obj);
    int x_center = center.x;
    line(dst, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    imshow("steer", dst3);

    std_msgs::Float32 steer, speed;

    //errorAngle = getSteer(center)*0.7 - preSteer*0.3;
    preSteer = errorAngle;
    //if(abs(errorAngle) > 4)
        //errorAngle *= 2.5;
    
    
    errorAngle = pid(center.x - IMG_W/2 + 1);
    //errorSpeed = dynamicSpeed(velocity, errorAngle);
   
    if(flag2 == true){
        temp = true;
        turn = flag;
        cout << "check-here1" << endl;
       
    }
    
    if ( white > 8100 && temp == true)
    {
        point = white;
    }
    if (white > 7000)
    {
        speed.data = 60;
        steer.data = errorAngle;
    }
    else
    {
        speed.data = 55;
        steer.data = errorAngle;
    }
    
    cout << temp << endl;
    if (temp == true && point > 8100)
    {
        speed.data = 30;
        speedPub.publish(speed);
        if (white < 9600)
        {
            speed.data = 30;
            speedPub.publish(speed); 
        }
        
        if(turn == 1){
            errorAngle = -30.0;
            steer.data = errorAngle;
        
            steerPub.publish(steer);
            speedPub.publish(speed);   
            if (leftTurn == true)
            {
                turn = 0;
                temp = false;
                point = 0;
            } 

        }
        
        if (turn == 2){
		    errorAngle = 30.0;
            steer.data = errorAngle;
        
            steerPub.publish(steer);
            speedPub.publish(speed);
            if (rightTurn == true)
            {
                turn = 0;
                temp = false;
                point = 0;
            } 
        }
       
    }
   
    /*cout << "dem: " << dem << endl;
      if (dem == 50)
    {
        temp = false;
        dem = 0;
    }*/
   
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
    
    

    steerPub.publish(steer);
    speedPub.publish(speed);
    
}
