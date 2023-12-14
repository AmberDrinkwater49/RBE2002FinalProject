#include <Arduino.h>
//#include "Behaviors.h"
#include "Speed_controller.h"
#include "line_follow.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"
#include "SerialM.h"
#include <openmv.h>
OpenMV camera;

//Behaviors positionEstimation;

Romi32U4ButtonC buttonC;
Romi32U4ButtonB buttonB;
Romi32U4ButtonA buttonA;
LineFollow robot; 
//ENDPRIMARY means that we found the garage first, and then the key, ENDSECONDARY means we found the key first and then the garage
enum ROBOT_STATE {IDLE, DRIVE_LINE, TURN, OPEN_LEFT, CLOSED_LEFT, CLOSED_FRONT, ENDPRIMARY, ENDSECONDARY, REC_MAP};
ROBOT_STATE robot_state = IDLE; //initial state: IDLE

//enum ACOMPLACESTATE_STATE {IDLE, DRIVE};
//ACCOMPLICE_STATE accomplice_state = IDLE;

int count = 0;
bool garageFound = false, keyFound = false, primary;

SerialM mqtt;
IRsensor irSensor;
IRsensor irSensorFront;
SonarSensor sonarSensor;
const float TOO_CLOSE = 30; //cm
const float BASE_SPEED = 40;

void setup() {
  //positionEstimation.Init();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000ul);
  robot.Init();
  irSensorFront.Init(A11);
  irSensor.Init(A0);
  Serial1.begin(115200);
}

uint8_t getID() { //get the apriltag id
    uint8_t tagCount = camera.getTagCount();
    if(tagCount) 
    {
      AprilTagDatum tag;
      if(camera.readTag(tag)){ 
          mqtt.sendMessage("Tag ID: ", String(tag.id));
          return tag.id;
      }
    }
}
//see if it's the key which will have an ID of 3
void isKey() {
//  mqtt.sendMessage("Key", "Looking");
  if(getID() == 3) {
   // mqtt.sendMessage("Key", "Found"); 
       if(garageFound){
      primary = true;
    }else{
      primary = false;
    }
    keyFound = true;
  }
}

//see if it's the garage whuch will have an ID of 4
void isGarage() {
    //mqtt.sendMessage("Garage", "Looking");
  if(getID() == 4) {
    //mqtt.sendMessage("Garage", "Found");
    garageFound = true;
    if(!keyFound){
      robot.cleanMapFirst();
        for(int i = 0; i < 54; i++){
          mqtt.sendMessage("X" + String(i), String(robot.getXCoordinate(i)));
          mqtt.sendMessage("Y" + String(i), String(robot.getYCoordinate(i)));
        }
    }
  }
}

void loop() {
  mqtt.sendMessage("ROBOT STATE", String(robot_state));
  Serial1.println("Robot State" + String(':') + String(robot_state));
  if(robot.UpdateEncoderCounts()){
    robot.UpdatePose(robot.ReadVelocityLeft(), robot.ReadVelocityRight());
    switch(robot_state){
      //The IDLE state is used to allow us to position the romi while it is on, before the task begins. It is also used in debugging
      case IDLE:
            // Serial.print("IN IDLE");
            // isKey();
            // isGarage();
            // if(garageFound && keyFound && primary){
            //   // robot_state = ENDPRIMARY;
            //   Serial.println("endprimary");
            // }else if(garageFound && keyFound && !primary){
            //   // robot_state = ENDSECONDARY;
            //   Serial.println("ENDSECONDARY");

            // }else{
            //   // robot_state = TURN;
            //   Serial.println("TURN"); 
            // }  

        //The B button chooses the romi to be the main scout
        if(buttonB.getSingleDebouncedRelease()){
          robot_state = DRIVE_LINE;
        }
        //The C button designates this romi as an accomplice
        else if(buttonC.getSingleDebouncedRelease()){
          robot_state = REC_MAP;
        }
        break;

        // The DRIVE_LINE case has the romi line follow until it reaches an intersection. 
        case DRIVE_LINE:

          if(robot.reachedIntersection()){
            //robot.makeWaypoint();

            robot.centerVTC();
            robot.resetOdomytry();
            //Here the romi checks if the garage or key is in view
            isGarage();
            isKey();
            //Here the romi checks if the end condition has been reached and which order the garage and key were found.
            if(garageFound && keyFound && primary){
              robot_state = ENDPRIMARY;
            }else if(garageFound && keyFound && !primary){
              robot_state = ENDSECONDARY;

            //If the task has not been complete: we must continue the maze and call the turn function.
            }else{
              robot_state = TURN;
            }
            // count++;
          }else{
            robot.lineFollow(BASE_SPEED);
            if(millis() % 500){
            isGarage();
            isKey();
            }

          }
        break;

        /**
         * The TURN case begins the logic necessary to navigate every part of the maze. By turning left whenever possible we can ensure we 
         * view every wall. 
         */
        case TURN:
        /*
          Check ultrasonic at intersections to see if it's "too close"
            If too close, check IR to see if we can turn left
              if we can, turn left
              if we cannot, turn right
                if the ultrasonic says there's a wall there, turn right again

        */
       //if no wall on left
        if(abs(irSensor.ReadData()) >= TOO_CLOSE){
            //Open on left
            robot_state = OPEN_LEFT;

        //if wall is to front of robot
        }else if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
          //Closed on left
          robot_state = CLOSED_LEFT;
        }else{
          //Closed on left open on front
          robot_state = DRIVE_LINE;
        }
        break;
        //open left case --> this is for when there is no wall to the left of the romi
        case OPEN_LEFT:
          //we turn left
            isGarage();
            isKey();
            if(robot.turnToNextline(BASE_SPEED)){
              //we check for the garage and key and primary ->

              if(garageFound && keyFound && primary){
                robot_state = ENDPRIMARY;
              }else if(garageFound && keyFound && !primary){
                robot_state = ENDSECONDARY;
              }else{
                robot_state = DRIVE_LINE;
              }


            }
        break;
        /**
         * The CLOSED_LEFT case is called when the left IR sensor detects a wall on the left. But there still could be a wall in front of us,
         * so the robot will check in front of it using the front IR. If there is a wall there, the robot will turn right, otherwise it will continue.
        */
        case CLOSED_LEFT:
            isGarage();
            isKey();
          if(robot.turnToNextline(-1 * BASE_SPEED)){

            if(garageFound && keyFound && primary){
              robot_state = ENDPRIMARY;
            }else if(garageFound && keyFound && !primary){
              robot_state = ENDSECONDARY;
            }else if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
              //closed on front
              robot_state = CLOSED_FRONT;
            }else{
              robot_state = DRIVE_LINE;
            }
          }
        break;
        //closed front case --> if the robot has a wall in front of it
        case CLOSED_FRONT:
          //we turn right
              isGarage();
              isKey();
          if(robot.turnToNextline(-1 * BASE_SPEED)){
            if(garageFound && keyFound && primary){
              robot_state = ENDPRIMARY;
            }else if(garageFound && keyFound && !primary){
              robot_state = ENDSECONDARY;
            }else{
             robot_state = DRIVE_LINE;
            }


          }
        break;
        case ENDPRIMARY:
          mqtt.sendMessage("Key Found ", "TRUE");
        break;
        case ENDSECONDARY:
        if(count){
          robot.printMap();
            robot.cleanMapFirst();
        robot.printMap();
         for(int i = 0; i < 54; i++){
            mqtt.sendMessage("X" + String(i), String(robot.getXCoordinate(i)));
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
        case REC_MAP:
        String result[10];
        while(mqtt.checkSerial1()){
          Serial.print(mqtt.serString1);
        }
            
        break;
    }

  }


  



}