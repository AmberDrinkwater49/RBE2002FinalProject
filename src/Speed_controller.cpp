#include <Romi32U4.h>
#include "Encoders.h"
#include "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;

//Encoder MagneticEncoder; 
//Position odometry;
//Position theoretical;
float baseSpeed = 75;
void SpeedController::setEfforts(int left, int right)
{
    motors.setEfforts(left, right);
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{       
        //UpdateEncoderCounts();
        float e_left = target_velocity_left - ReadVelocityLeft();
        float e_right = target_velocity_right - ReadVelocityRight();
        // Serial.print("ReadVelocityLeft()");
        // Serial.println(ReadVelocityLeft());
        // Serial.println(" ");
        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        
}

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    int turns = counts*(degree);
    int count_turn = ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(ReadEncoderCountLeft())) <= turns)
    {      
        setEfforts(-(direction*baseSpeed),(direction*baseSpeed));  
        if(UpdateEncoderCounts()){
        UpdatePose(ReadVelocityLeft(), ReadVelocityRight());
        //Serial.println("loopiong infinetly");
        }
      
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    //Serial.println(target_velocity);
    delay(1000);
    unsigned long now = millis();
    while ((unsigned long)(millis() - now) <= time*1000){
        if(UpdateEncoderCounts()){
            Run(target_velocity,target_velocity);
        }
    }
    motors.setEfforts(0, 0);

    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
}



