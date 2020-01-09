#include "ControlCar.h"



ControlCar::ControlCar()
{
    steerPub = nodeObj1.advertise<std_msgs::Float32>(STEER_TOPIC,1);
    speedPub = nodeObj2.advertise<std_msgs::Float32>(SPEED_TOPIC,1);
}
ControlCar::~ControlCar(){}

Rect ControlCar::danger_zone(const Rect obs){
   int area = 60;
   int wRect = obs.width , hRect = obs.height;
   Rect rect = Rect(0,0,0,0);
   if (obs != Rect(0,0,0,0))
      rect = Rect(obs.x - area, obs.y, obs.x+ wRect + area, hRect + 20);
   return rect;
}

Point ControlCar::getPoint(const Mat &src, Rect rect){
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

float ControlCar::pid(const float &cte){
    error_i += cte;
    if(error_i > 500)
        error_i = 500;
    if(error_i < -500)
        error_i = -500;
    error_d = cte - error_p;
    error_p = cte;
    return (k_p * error_p + k_i * error_i + k_d * error_d);
}

float ControlCar::getSteer(const Point &p){
    float dx = p.x - IMG_W/2 + 1;
    float dy = IMG_H - (float)p.y;
    float steer = atan(dx/dy) * 57.32; // 180/PI
    return steer;
}

double ControlCar::checkRigh(const Mat &src){
    Mat right = src(rectRight);
    return countNonZero(right);
}
double ControlCar::checkLeft(const Mat &src){
    Mat left = src(rectLeft);
    return countNonZero(left);
}
double ControlCar::checkRoad(const Mat &src){
    Mat road = src(rectRoad);
    return countNonZero(road);
}

void ControlCar::driveCar(const Mat &view, const Mat &view1,float velocity,int flag, bool flag2, Rect obj, float fps){
    std_msgs::Float32 steer, speed;
    float errorAngle;

    center = getPoint(view, obj);

    errorAngle = pid(center.x - IMG_W/2 + 1) + 4;

    double RA = checkRoad(view)/roadArea;
    if(RA >= 0.45)
        velocity = velocity - 10;
    else if (RA >= 0.4)
        velocity = velocity - 10;
    else if (RA >= 0.35)
        velocity = velocity - 10;
    else if (RA >= 0.25)
        velocity = velocity - 15;
    else
        velocity = velocity - 20;
    if(false){
        cout << "area: " << RA << "\tvelocity: " << velocity << endl;
    }

    if(flag2 == true && doTurn == false){
        FPS = fps;
        frame = 0;
        isSign = true;
        turn = flag;
    }
    if(isSign == true){
        velocity = 50;
        if(false)
            cout << endl << "turn: " << turn << "\t";
    }
    if(isSign == true && doTurn == false){
        int whitePoint;
        frame++;
        if(turn == 1)
            whitePoint = checkLeft(view1);
        if(turn == 2)
            whitePoint = checkRigh(view1);
        if(whitePoint/area > 0.6){
            doTurn = true;
        }
        if(false)
            cout << frame << "\tarea: " << round(whitePoint*100/area)/100;
    }
    if(frame >= (int)FPS*2){
        frame = 0;
        isSign = false;
    }
    if(doTurn == true){
        int whitePointLeft, whitePointRight;
        index++;
        if(turn == 1)
            errorAngle = -55;
        if(turn == 2)
            errorAngle = 55;
        whitePointLeft = checkLeft(view1);
        whitePointRight = checkRigh(view1);
        if(false){
            cout<<index <<"\t"<<round(whitePointLeft*100/area)/100
                <<"\t"<<round(whitePointRight*100/area)/100 
                << "\terrorAngle: " << errorAngle<<endl;
        }
        int temp = 0;
        if (FPS < 13)
            temp = FPS*2;
        else
            temp = FPS;

        if(temp < 18)
            temp = 18;
        if(whitePointLeft/area < 0.3 && whitePointRight/area < 0.3 && index > temp){
            index = 0;
            frame = 0;
            isSign = false;
            doTurn = false;
            if(false){
                cout<<endl <<"Fps: "<<FPS << "\ttemp: " << temp <<endl;
                cout<<"\n----------------------\n";
            }
        }
    }
    if(true){
        Mat noCut = view1.clone();
        line(noCut,center, Point(160,239), Scalar(0), 2);
        rectangle(noCut, obj, Scalar(0), -1);
        rectangle(noCut, rectLeft, Scalar(255),1);
        rectangle(noCut, rectRight, Scalar(255),1);
        imshow("steer", noCut);
    }
    steer.data = errorAngle;
    speed.data = velocity;
   
    steerPub.publish(steer);
    speedPub.publish(speed);
    
}
