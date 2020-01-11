#ifndef CONTROLCAR_H
#define CONTROLCAR_H
 
#include "header.h"
class ControlCar
{
    public:
        static Rect null;
        ControlCar();
        ~ControlCar();
        void driveCar(const Mat &view,const Mat &view1 ,float velocity,int decision, bool flag2,  Rect obj, float FPS);

    private:
        ros::NodeHandle nodeObj1;
        ros::NodeHandle nodeObj2;
        ros::NodeHandle n;

        ros::Publisher pub;
        ros::Publisher steerPub;
        ros::Publisher speedPub;
        
        float k_p = 0.9;
        float k_i = 0.01;
        float k_d = 0.25;
        float inc_p = 0.01;
        float inc_i = 0.0001;
        float inc_d = 0.01;
        float error_p = 0.0;
        float error_i = 0.0;
        float error_d = 0.0;

        Rect rectRight = Rect(240, 120, 45, 20);
        Rect rectLeft = Rect(35, 120, 45, 20);
        Rect rectStopRight = Rect(265, 140, 45, 20);
        Rect rectStopLeft = Rect(10, 140, 45, 20);
        Rect rectRoad = Rect(0, 110, 320, 30);

        double roadArea = rectRoad.area();
        double area = rectLeft.area();
        int turn = 0;
        bool isSign = false;
        bool haveSign = false;
        bool doTurn = false;
        int index = 0;
        int frame = 0;
        int _frame = 0;
        float FPS = 0;

        Point center = Point(0,0);//Initial point to control

        Point getPoint(const Mat &src, Rect obj);
        float getSteer(const Point &p);
        Rect danger_zone(const Rect obs);
       
        float pid(const float &cte);

        double turnProcess(const Mat &src, Rect rect);
        double checkRoad(const Mat &src);
};
#endif
