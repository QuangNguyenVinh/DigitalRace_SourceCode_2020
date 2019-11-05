#ifndef CONTROLCAR_H
#define CONTROLCAR_H

#include "header.h"
class ControlCar
{
    public:
        ControlCar();
        ~ControlCar();
        void driveCar(const Mat &view, float velocity,bool flag, bool flag2);

    private:
        ros::NodeHandle nodeObj1;
        ros::NodeHandle nodeObj2;
        ros::NodeHandle n;

        ros::Publisher pub;
        ros::Publisher steerPub;
        ros::Publisher speedPub;



        Point getPoint(const Mat &src);
        float getSteer(const Point &p);
};
#endif