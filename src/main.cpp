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
/**
 * Note on inheritence:
 * LineFollow is a child of Speed Controller which is a child of Position Estimation which is a chile of Encoders
*/
LineFollow robot; 
//ENDPRIMARY means that we found the garage first, and then the key, ENDSECONDARY means we found the key first and then the garage
enum ROBOT_STATE {IDLE, DRIVE_LINE, TURN, OPEN_LEFT, CLOSED_LEFT, CLOSED_FRONT, ENDPRIMARY, ENDSECONDARY};
ROBOT_STATE robot_state = IDLE; //initial state: IDLE

//enum ACOMPLACESTATE_STATE {IDLE, DRIVE};
//ACCOMPLICE_STATE accomplice_state = IDLE;

//These boolean values store the status of whether the garage or key has been found, primary keeps track of which was found first.
bool garageFound = false, keyFound = false, primary;
SerialM mqtt;
IRsensor irSensor;
IRsensor irSensorFront;
SonarSensor sonarSensor;
const float TOO_CLOSE = 30; //cm - This value is used to indicate whether there is a wall to the left or in front of the romi
const float BASE_SPEED = 50; 

/**
 * The setup function is called at startup and sets clocks and ports so that the robot can function
*/
void setup() {
  //positionEstimation.Init();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000ul);
  robot.Init();
  irSensorFront.Init(A11);
  irSensor.Init(A0);
  Serial1.begin(115200);
  digitalWrite(0, HIGH);

}

/**
 * GetID searches the camera to find any apriltags and returns the ID that is found, if no april tag is found it returns 0
*/
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
    return 0;
}

/**
 * isKey() takes the ID from getID() and sees if the ID is a 3, if it is this means that that april tag is the key.
 * It then turns key found to true, if the garage has been found then we enter the primary ending. 
*/
void isKey() {
//  mqtt.sendMessage("Key", "Looking");
  if(getID() == 2) {
   // mqtt.sendMessage("Key", "Found"); 
    if(garageFound){
      primary = true;
    }
    keyFound = true;
  }
}

/**
 * isGarage() takes the ID from getID() and sees if the ID is a 1, if it is this means that the april tag is the garage.
 * It then turns garageFound to true, if the key has not been found, the robot transmits the map now. Otherwise, the second ending activates.
*/
void isGarage() {
    //mqtt.sendMessage("Garage", "Looking");
  if(getID() == 1) {
    //mqtt.sendMessage("Garage", "Found");
    garageFound = true;
    if(!keyFound){
      robot.cleanMapFirst();
        mqtt.sendMessage("CC", String(robot.getCurrentCoordinate()-1));
        for(int i = 0; i <+ robot.getCurrentCoordinate(); i++){
          mqtt.sendMessage("X" + String(i), String(robot.getXCoordinate(i)));
          mqtt.sendMessage("Y" + String(i), String(robot.getYCoordinate(i)));
        }
    }else{
      primary = false;
    }
  }
}

void loop() {
//First the robot updates its pose to be used in later calculations
  if(robot.UpdateEncoderCounts()){
    robot.UpdatePose(robot.ReadVelocityLeft(), robot.ReadVelocityRight());
    
    switch(robot_state){
      //The IDLE state is used to allow us to position the romi while it is on, before the task begins. It is also used in debugging
      case IDLE:
        
        //The B button chooses the romi to be the main scout
        if(buttonB.getSingleDebouncedRelease()){
          mqtt.sendMessage("RS", "DRIVE LINE");
          robot_state = DRIVE_LINE;
        }
        break;

        // The DRIVE_LINE case has the romi line follow until it reaches an intersection. 
        case DRIVE_LINE:

          if(robot.reachedIntersection()){
            //Here the romi checks if the garage or key is in view
            robot.centerVTC();
            robot.resetOdomytry();
            float now = millis();
            while(millis() <= now + 1500){
              isGarage();
              isKey();
            }
            //Here the romi checks if the end condition has been reached and which order the garage and key were found.
            if(garageFound && keyFound && primary){
              mqtt.sendMessage("RS", "ENDPRIMARY");
              robot_state = ENDPRIMARY;
            }else if(garageFound && keyFound && !primary){
              mqtt.sendMessage("RS", "ENDSECONDARY");
              robot_state = ENDSECONDARY;
            //If the task has not been complete: we must continue the maze and call the turn function.
            }else{
              mqtt.sendMessage("RS", "TURN");
              robot_state = TURN;
            }
          }else{
            //If the robot has not reached an intersection, it will continue following the line it is on
            robot.lineFollow(BASE_SPEED);
            //While driving the robot checks the walls for the key and garage
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
        Summary of logic:
          Check frontIR at intersections to see if it's "too close"
            If too close, check leftIR to see if we can turn left
              if we can, turn left
              if we cannot, turn right
                if the frontIR says there's a wall there, turn right again

        */
       //if no wall on left
        if(abs(irSensor.ReadData()) >= TOO_CLOSE){
            //Open on left
            mqtt.sendMessage("RS", "OPEN_LEFT");
            robot_state = OPEN_LEFT;

        //if wall is to front of robot
        }else if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
          //Closed on left
          mqtt.sendMessage("RS", "CLOSED_LEFT");
          robot_state = CLOSED_LEFT;
        }else{
          //Closed on left open on front
          mqtt.sendMessage("RS", "DRIVE_LINE");
          robot_state = DRIVE_LINE;
        }
        break;
        //open left case --> this is for when there is no wall to the left of the romi
        case OPEN_LEFT:
          //we turn left

            if(robot.turnToNextline(BASE_SPEED)){
              //we check for the garage and key and primary ->

              if(garageFound && keyFound && primary){
                mqtt.sendMessage("RS", "ENDPRIMARY");
                robot_state = ENDPRIMARY;
              }else if(garageFound && keyFound && !primary){
                mqtt.sendMessage("RS", "ENDSECONDARY");
                robot_state = ENDSECONDARY;
              }else{
                mqtt.sendMessage("RS", "DRIVE LINE");
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
              mqtt.sendMessage("RS", "ENDPRIMARY");
              robot_state = ENDPRIMARY;
            }else if(garageFound && keyFound && !primary){
              mqtt.sendMessage("RS", "ENDSECONDARY");
              robot_state = ENDSECONDARY;
            }else if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
              //closed on front
              mqtt.sendMessage("RS", "CLOSED_FRONT");
              robot_state = CLOSED_FRONT;
            }else{
              mqtt.sendMessage("RS", "DRIVE LINE");
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
              mqtt.sendMessage("RS", "ENDPRIMARY");
              robot_state = ENDPRIMARY;
            }else if(garageFound && keyFound && !primary){
              mqtt.sendMessage("RS", "ENDSECONDARY");
              robot_state = ENDSECONDARY;
            }else{
             mqtt.sendMessage("RS", "DRIVE LINE");
             robot_state = DRIVE_LINE;
            }

          }
        break;
        case ENDPRIMARY:
          mqtt.sendMessage("Key Found ", "TRUE");
          
        break;
        case ENDSECONDARY:
          Serial.println("DONE");
          robot.printMap();
          robot.cleanMapFirst();
          robot.printMap();
          mqtt.sendMessage("CC", String(robot.getCurrentCoordinate()-1));
         for(int i = 0; i <+ robot.getCurrentCoordinate(); i++){
            mqtt.sendMessage("X" + String(i), String(robot.getXCoordinate(i)));
            mqtt.sendMessage("Y" + String(i), String(robot.getYCoordinate(i)));
        }

        mqtt.sendMessage("RS", "IDLE");
        robot_state = IDLE;

        break;
        
    }
  }
}