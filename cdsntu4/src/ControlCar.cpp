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
    imshow("road", road);
    return countNonZero(road);
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

void ControlCar::driveCar(const Mat &view, const Mat &view1,float velocity,int flag, bool flag2, Rect obj, float fps){
    std_msgs::Float32 steer, speed;
    Mat noCut = view1.clone();
    float errorAngle;

    center = getPoint(view, obj);

    errorAngle = pid(center.x - IMG_W/2 + 1);

    double RA = checkRoad(view)/roadArea;
    //cout << "road area: " << RA << endl;
    if(RA >= 0.45)
        velocity = velocity;
    else if (RA >= 0.35)
        velocity = velocity - 10;
    else if (RA >= 0.28)
        velocity = velocity - 15;
    else
        velocity = velocity - 20;


    if(flag2 == true){
        FPS = fps;
        frame = 0;
        isSign = true;
        turn = flag;
    }
    if(isSign == true){
        velocity = 50;
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
        cout << frame << "\tarea: " << round(whitePoint*100/area)/100;
    }
    if(frame >= (int)FPS*2){
        frame = 0;
        isSign = false;
    }
    if(doTurn == true){
        int whitePointLeft, whitePointRight;
        index++;
        try{
            center = dangerPoint(view1, turn);
        }
        catch(Exception e){
            cout<<"\nbug dangerPoint\n";
            cout<<"message: "<<e.msg<<endl;
            center = Point(160,120);
        }
        errorAngle = getSteer(center);
        //if(turn == 1)
            whitePointLeft = checkLeft(view1);
        //if(turn == 2)
            whitePointRight = checkRigh(view1);
        //cout << index << "\tarea: "<< round(whitePoint*100/area)/100 << "\terrorAngle: " << errorAngle;
        cout<<index <<"\t"<<round(whitePointLeft*100/area)/100<<"\t"<<round(whitePointRight*100/area)/100<<endl;
        int temp = 0;
        if (FPS < 13)
            temp = FPS*2.2;
        else if(FPS < 20)
            temp = FPS*1.5;
        else
            temp = FPS*1.2;
        if(whitePointLeft/area < 0.4 && whitePointRight/area < 0.4 && index > temp){
            index = 0;
            frame = 0;
            isSign = false;
            doTurn = false;
            cout<<endl <<"Fps: "<<FPS << "\ttemp: " << temp <<endl;
            cout<<"\n----------------------\n";
        }
    }
    try{
        line(noCut,center, Point(160,239), Scalar(0), 2);
        rectangle(noCut, obj, Scalar(0), -1);
        rectangle(noCut, rectLeft, Scalar(255),1);
        rectangle(noCut, rectRight, Scalar(255),1);
    }
    catch (Exception e){
        cout<<"\nDraw bug: "<<e.msg<<endl;
    }
    imshow("steer", noCut);
    steer.data = errorAngle;
    speed.data = velocity;
   
    steerPub.publish(steer);
    speedPub.publish(speed);
    
}
