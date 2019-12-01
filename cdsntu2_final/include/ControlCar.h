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

        float delay = 0.00;
        int index = 0;
        bool temp = false;
        int turn = 0;


        Point getPoint(const Mat &src, Rect obj);
        float getSteer(const Point &p);
        Rect danger_zone(const Rect obs);
        float dynamicSpeed(const float &velocity, const float &steer);
};
#endif
