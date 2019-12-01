#ifndef CONTROLCAR_H
#define CONTROLCAR_H

#include "header.h"
class ControlCar
{
    public:
        static Rect null;
        ControlCar();
        ~ControlCar();
        void driveCar(const Mat &view, float velocity,int flag, bool flag2,  Rect obj, Rect sign);

    private:
        ros::NodeHandle nodeObj1;
        ros::NodeHandle nodeObj2;
        ros::NodeHandle n;

        ros::Publisher pub;
        ros::Publisher steerPub;
        ros::Publisher speedPub;

        float preSteer = 0;
        
        float k_p = 0.5;
        float k_i = 0.007;
        float k_d = 0.2;
        float inc_p = 0.01;
        float inc_i = 0.0001;
        float inc_d = 0.01;
        float error_p = 0.0;
        float error_i = 0.0;
        float error_d = 0.0;

        float delay = 0.00;
        int index = 0;
        bool temp = false;
        int turn = 0;


        Point getPoint(const Mat &src, Rect obj);
        float getSteer(const Point &p);
        Rect danger_zone(const Rect obs);
       
        float pid(const float &cte);
        float dynamicSpeed(const float &velocity, const float &steer);
};
#endif
