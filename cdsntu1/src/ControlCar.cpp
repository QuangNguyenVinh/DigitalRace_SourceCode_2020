#include "ControlCar.h"
ControlCar::ControlCar()
{
    steerPub = nodeObj1.advertise<std_msgs::Float32>(STEER_TOPIC,1);
    speedPub = nodeObj2.advertise<std_msgs::Float32>(SPEED_TOPIC,1);
}
ControlCar::~ControlCar(){}
Point ControlCar::getPoint(const Mat &src)
{
    int midX = 0, midY = 0;
    int count = 0;
    int limit = (int) (IMG_H * 1 / 3);
    for (int i = limit; i < IMG_H; i++) {
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
        midY += i;
    }
    return Point((int) (midX / count), (int) (midY / count));
}
Point ControlCar::getPoint2(const Mat &bin)
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
    return steer;
}
float ControlCar::dynamicSpeed(const float &velocity, const float &steer)
{
    return velocity * cos(abs(steer) * 0.0174); // = PI/180
}
void ControlCar::driveCar(const Mat &view, float velocity,int flag, bool flag2)
{
    float errorAngle , errorSpeed;
    Point center(0,0);//Initial point to control

    Mat dst = view.clone();
    center = getPoint(view);
    line(dst, center, Point((int)(IMG_W/2), (int)(IMG_H -1)), (0, 0, 0), 2);
    imshow("steer", dst);

    std_msgs::Float32 steer, speed;

    errorAngle = getSteer(center)*0.72 - preSteer*0.28;
    preSteer = errorAngle;
    errorSpeed = dynamicSpeed(velocity, errorAngle);
    if(flag2 == true)
    {
        if(flag == 1)
        {
            errorAngle = -20.0;
            velocity = 40;
        }
        else if(flag == 2)
        {
            errorAngle = 20.0;
            velocity = 40;
        }
        steer.data = errorAngle;
        speed.data = velocity;

        steerPub.publish(steer);
        speedPub.publish(speed);
        sleep(2);
    }
    steer.data = errorAngle;
    speed.data = velocity;

    steerPub.publish(steer);
    speedPub.publish(speed);
}
