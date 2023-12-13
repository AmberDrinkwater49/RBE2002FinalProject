#include <Arduino.h>
//#include "Behaviors.h"
#include "Speed_controller.h"
#include "line_follow.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"
#include "SerialM.h"

//Behaviors positionEstimation;


Romi32U4ButtonB buttonB;
Romi32U4ButtonA buttonA;
LineFollow robot; 
enum ROBOT_STATE {IDLE, SEARCH, DRIVE_LINE, TURN, OPEN_LEFT, CLOSED_LEFT, CLOSED_FRONT, END};
ROBOT_STATE robot_state = IDLE; //initial state: IDLE

//enum ACOMPLACESTATE_STATE {IDLE, DRIVE};
//ACCOMPLICE_STATE accomplice_state = IDLE;

int count = 0;


SerialM mqtt;
IRsensor irSensor;
IRsensor irSensorFront;
SonarSensor sonarSensor;
const float TOO_CLOSE = 40, TOO_CLOSE_BUT_THIS_TIME_WITH_THE_IR_SENSOR = 15; //cm


void setup() {
  //positionEstimation.Init();
  robot.Init();
  irSensorFront.Init(A11);
  irSensor.Init(A0);
  Serial1.begin(115200);
}



void loop() {
  //positionEstimation.Run();
  if(count >= 27){
    robot_state = END;
  }
  if(robot.UpdateEncoderCounts()){
    robot.UpdatePose(robot.ReadVelocityLeft(), robot.ReadVelocityRight());
    //Serial.println(robot.getThetaDeg());
    switch(robot_state){
      case IDLE:

        if(buttonB.getSingleDebouncedRelease()){
          robot_state = DRIVE_LINE;
          Serial.println("DRIVE_LINE");
        }
        break;

        case DRIVE_LINE:
          if(robot.reachedIntersection()){
            //robot.makeWaypoint();
            robot.centerVTC();
            robot.resetOdomytry();
            robot_state = TURN;
            count++;
                      Serial.println("TURN");

          }else{
            robot.lineFollow(45);
          }
          if(buttonB.getSingleDebouncedRelease()){
            robot_state = DRIVE_LINE;
                      Serial.println("DRIVE_LINE");

          }
        break;

        case TURN:
        /*
          Check ultrasonic at intersections to see if it's "too close"
            If too close, check IR to see if we can turn left
              if we can, turn left
              if we cannot, turn right
                if the ultrasonic says there's a wall there, turn right again

        */
       //if nop wall on left
        if(abs(irSensor.ReadData()) >= TOO_CLOSE){
            //Open on left
            robot_state = OPEN_LEFT;
                      Serial.println("OPEN_LEFT");

        //if wall is to front of robot
        }else if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
          //Closed on left
          robot_state = CLOSED_LEFT;
                    Serial.println("CLOSED_LEFT");

        }else{
          //Closed on left open on front
          robot_state = DRIVE_LINE;
                    Serial.println("DRIVE_LINE");

        }
          // if(robot.turnToNextline(-100)){
          //   robot_state = DRIVE_LINE;
          // }

        break;
        case OPEN_LEFT:
            if(robot.turnToNextline(75)){
              robot_state = DRIVE_LINE;
                        Serial.println("DRIVE_LINE");

            }
        break;
        case CLOSED_LEFT:
          if(robot.turnToNextline(-75)){
            if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
              //closed on front
              robot_state = CLOSED_FRONT;
                        Serial.println("CLOSED_FRONT");

            }else{
              robot_state = DRIVE_LINE;
                        Serial.println("DRIVE_LINE");

            }
          }
        break;
        case CLOSED_FRONT:
          if(robot.turnToNextline(-75)){
             robot_state = DRIVE_LINE;
                       Serial.println("DRIVE_LINE");

          }
        break;
        case END:
        if(count){
          robot.printMap();
            robot.cleanMapFirst();
        robot.printMap();
         for(int i = 0; i < 54; i++){
            Serial.print("X: ");
            Serial.print(robot.getXCoordinate(i));
            mqtt.sendMessage("X" + String(i), String(robot.getXCoordinate(i)));
            Serial.print(" Y: ");
            Serial.println(robot.getYCoordinate(i));
            mqtt.sendMessage("Y" + String(i), String(robot.getYCoordinate(i)));
        }
        count = 0;
        }

        break;
        if(buttonB.getSingleDebouncedRelease()){
          robot.Stop();
          robot_state = IDLE;
                    Serial.println("IDLE");

        }
    }

  }


  //state(accomplice)

}