#include "ControlCar.h"


ControlCar::ControlCar()
{
    steerPub = nodeObj1.advertise<std_msgs::Float32>(STEER_TOPIC,1);
    speedPub = nodeObj2.advertise<std_msgs::Float32>(SPEED_TOPIC,1);
}
ControlCar::~ControlCar(){}
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
        cout << "midX: " << midX/count << "\t";
        //cout << "midY: " << midY << "\t";
        //cout << "Rect" << obs << endl;
    }
    return Point((int) (midX / count), (int) (midY));
}
float ControlCar::getSteer(const Point &p)
{
    float dx = p.x - IMG_W/2 + 1;
    float dy = IMG_H - (float)p.y;
    float steer = atan(dx/dy) * 57.32; // 180/PI
    return steer;
}
float ControlCar::dynamicSpeed(const float &velocity, const float &steer)
{
    return velocity * cos(abs(steer)* 0.0174); //Pi/180
}
void ControlCar::driveCar(const Mat &view, float velocity,int flag, bool flag2, Rect obj, Rect sign)
{
    int frame = 2;
    int dem = 0;
    
    float errorAngle , errorSpeed;
    Point center(0,0);//Initial point to control
    
    Rect rectsign = sign;

    Mat dst = view.clone();
    center = getPoint(view, obj);
    int x_center = center.x;
    line(dst, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    imshow("steer", dst);

    std_msgs::Float32 steer, speed;

    errorAngle = getSteer(center)*0.7 - preSteer*0.3;
    preSteer = errorAngle;
    //if(abs(errorAngle) > 4)
        //errorAngle *= 2.5;
    
    //errorSpeed = dynamicSpeed(velocity, errorAngle);
    steer.data = errorAngle;
    speed.data = velocity;
    if(flag2 == true){
        temp = true;
        turn = flag;
    }
    
    if(temp == true && rectsign.x > 276 && rectsign.y > 50){
        int dem = rectsign.y;
        if (dem < 64 )
            delay = 0.16;
        else{
            frame = 2;
            delay = 0.00;
        }
            
        
        do {
           dem++;
           cout << "delay: " << delay << endl;
           ros::Duration(float(delay)).sleep();
        }
        while (dem <= rectsign.y + 1);
        cout << "delay: " << float(delay) << "\t";
        cout << "Y: " << rectsign.y << "\t";
        cout << "dem: " << dem <<endl;
        if (dem > rectsign.y + 1){
            index++;
        if (index <= frame){
            if(turn == 1){
			    errorAngle = -40.0;
                velocity = 35;

            }
            if (turn == 2){
			    errorAngle = 40.0;
                velocity = 35;
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
        ros::Duration(0.7).sleep();
        }
        
        }
        
        
    }	
    /*if(flag2 == true)
    {
        if(flag == 1)
        {   
            
            velocity = 40;
            steer.data = errorAngle;
            speed.data = velocity;
            steerPub.publish(steer);
            speedPub.publish(speed);
            sleep(1);
            errorAngle = -20.0;
            velocity = 40;
        }
        else if(flag == 2)
        {
            velocity = 40;
            steer.data = errorAngle;
            speed.data = velocity;
            steerPub.publish(steer);
            speedPub.publish(speed);
            sleep(1);
            errorAngle = 20.0;
            velocity = 40;
        }
        steer.data = errorAngle;
        speed.data = velocity;

        steerPub.publish(steer);
        speedPub.publish(speed);
        sleep(2);
    }*/
    

    steerPub.publish(steer);
    speedPub.publish(speed);
}
