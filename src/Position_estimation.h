#ifndef POSITION_ESTIMATION
#define POSITION_ESTIMATION

#include <Romi32U4.h>
#include <Encoders.h>


class Position : public Encoder

{
    private:
        float x_calculated =0, y_calculated=0, 
        theta_calculated=0, x_theoretical=0, y_theoretical=0, 
        theta_theoretical=0;
        unsigned long time_prev, time_now;
        const float l = 142.875; //assignment
        //const float l = 200; //assignment
        float x = 0;
        float y = 0;
        float theta = 0;
        float waypoint[18][2];
        int waypoiontCounter;

    public:
        struct pose_data {
            float X;
            float Y;
            float THETA;
        };
        void Init(void);
        int getCurrentCoordinate(void);
        void UpdatePose(float,float);
        void printMap(void);
        float getXCoordinate(int);
        float getYCoordinate(int);
        //void UpdatePose(float,float,float,float);
        float getX(void);
        float getY(void);
        float getTheta(void);
        float getThetaDeg(void);
        void PrintPose(void);
        void Stop(void);
        void makeWaypoint(void);
        void cleanMapFirst(void);
        void resetOdomytry(void);
        int getCrrentHeading(void);
        void followWaypoints(void);

};

#endif