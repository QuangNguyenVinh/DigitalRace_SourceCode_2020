#ifndef CONTROLCAR_H
#define CONTROLCAR_H
 
#include "header.h"
class ControlCar
{
    public:
        static Rect null;
        ControlCar();
        ~ControlCar();
        void driveCar(const Mat &view,const Mat &view1,int flag, bool flag2,bool flag3,  Rect obj);
        bool flagLeftTurn = false;
        int flagRightTurn = 0;
    private:
        ros::NodeHandle nodeObj1;
        ros::NodeHandle nodeObj2;
        ros::NodeHandle n;

        ros::Publisher pub;
        ros::Publisher steerPub;
        ros::Publisher speedPub;

        float preSteer = 0;
        float velocity = 70;
        
        float k_p = 0.9;
        float k_i = 0.01;
        float k_d = 0.25;

        float error_p = 0.0;
        float error_i = 0.0;
        float error_d = 0.0;

        float delay = 0.00;
        int index = 0;
        bool temp = false;
        int turn = 0;
        int dem = 0;
        int demtemp = 0;
        int point = 0;
       

        int getArea(const Mat &bin);
        Point getPoint(const Mat &src, Rect obj);
        float getSteer(const Point &p);
        Rect danger_zone(const Rect obs);
       
        float pid(const float &cte);
        float dynamicSpeed(const float &velocity, const float &steer);

        bool checkLeftTurn(const Mat &bin, Rect leftRect);
        bool checkRightTurn(const Mat &bin, Rect rightRect);
        bool checkLeft(const Mat &bin, Rect leftRect);
        bool checkRight(const Mat &bin, Rect rightRect);


        /*Publish speed, steer data */
        void pubSpeed(const float &veloc);
        void pubSteer(const float &angle);
        void pubSpeedAndSteer(const float &veloc, const float &angle);
};
#endif
