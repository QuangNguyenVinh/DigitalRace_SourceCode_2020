#ifndef CONTROLCAR_H
#define CONTROLCAR_H

#include "header.h"
class ControlCar
{
    public:
        ControlCar();
        ~ControlCar();
        void driveCar(const Mat &view, int flag, bool flag2, Rect rect);

    private:
        ros::NodeHandle nodeObj1;
        ros::NodeHandle nodeObj2;
        ros::NodeHandle n;

        ros::Publisher pub;
        ros::Publisher steerPub;
        ros::Publisher speedPub;

        float k_p = 0.7;
        float k_i = 0.01;
        float k_d = 0.15;


        float error_p = 0.0;
        float error_i = 0.0;
        float error_d = 0.0;

        float preSteer = 0;

        //For count time to turn
        bool current_flag = false;

        Mat cutROI(const Mat &src);
        int getArea(const Mat &bin);
        Rect danger_zone(const Rect &obs);

        //Get center point to follow
        Point getPoint(const Mat &src);
        Point getPoint2(const Mat &src, Rect rect);
        Point getPoint3(const Mat &src);


        float getSteer(const Point &p);
        float dynamicSpeed(const float &v, const float &steer);


        float pid(const float &cte);
        void pubSpeed(const float &veloc);
        void pubSteer(const float &angle);
};
#endif
