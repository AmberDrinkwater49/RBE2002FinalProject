#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.2; 
        const float P_Kp = 10;
        const float P_Ki = 2;
        float E_left = 0; 
        float E_right = 0;
        int counts = 8; //assignment
        float xPositionError_previous = 0;
        float yPositionError_previous = 0;

    public:
        void Init(void);
        void Run(float, float); //speed left, speed right
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        void Stop(void);
        void PositionController(float,float,float,float);
};

#endif