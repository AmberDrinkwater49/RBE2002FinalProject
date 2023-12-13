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
const float BASE_SPEED = 50;

void setup() {
  //positionEstimation.Init();
  robot.Init();
  irSensorFront.Init(A11);
  irSensor.Init(A0);
  String getSerString(void);
  Serial1.begin(115200);
}

uint8_t getID() { //id
    uint8_t tagCount = camera.getTagCount();
    Serial.println("GET ID");
    if(tagCount) 
    {
      AprilTagDatum tag;
      if(camera.readTag(tag)){ 
          Serial.println(tag.id);
          mqtt.sendMessage("Tag ID: ", String(tag.id));
          return tag.id;
      }
    }
}
//see if it's the key which will have an ID of 3
bool isKey() {
  if(uint8_t id = getID() == 3) {
    Serial.print("Key found!");
    if(garageFound){
      primary = true;
    }else{
      primary = false;
    }
    keyFound = true;
    return true;
  }
  return false;
}

//see if it's the garage whuch will have an ID of 4
bool isGarage() {
  if(getID() == 4) {
    Serial.print("Garage found!");
    garageFound = true;
    if(!keyFound){
      robot.cleanMapFirst();
        for(int i = 0; i < 54; i++){
          mqtt.sendMessage("X" + String(i), String(robot.getXCoordinate(i)));
          mqtt.sendMessage("Y" + String(i), String(robot.getYCoordinate(i)));
        }
    }
    return true;
  }
  return false;
}

void loop() {
  //positionEstimation.Run();
  // if(count >= 15){
  //   robot_state = END;
  // }
  if(millis() % 100 == 0){
        mqtt.sendMessage("Front IR", String(irSensorFront.ReadData()));
  }
  if(robot.UpdateEncoderCounts()){
    robot.UpdatePose(robot.ReadVelocityLeft(), robot.ReadVelocityRight());
    //Serial.println(robot.getThetaDeg());
    switch(robot_state){
      case IDLE:

            if(isGarage() && isKey() && primary){
              robot_state = ENDPRIMARY;
              Serial.println("endprimary");
            }else if(isGarage() && isKey() && !primary){
              robot_state = ENDSECONDARY;
              Serial.println("ENDSECONDARY");

            }else{
              robot_state = TURN;
              Serial.println("TURN"); 
            }

        if(buttonB.getSingleDebouncedRelease()){
          robot_state = DRIVE_LINE;
          Serial.println("DRIVE_LINE");
        }
        else if(buttonC.getSingleDebouncedRelease()){
          robot_state = REC_MAP;
          Serial.println("REC_MAP");
        }
        break;

        case DRIVE_LINE:

          if(robot.reachedIntersection()){
            //robot.makeWaypoint();
            robot.centerVTC();
            robot.resetOdomytry();
            Serial.println("REACHED INTERSECTION");
            if(isGarage() && isKey() && primary){
              robot_state = ENDPRIMARY;
              Serial.println("endprimary");
            }else if(isGarage() && isKey() && !primary){
              robot_state = ENDSECONDARY;
              Serial.println("ENDSECONDARY");

            }else{
              robot_state = TURN;
              Serial.println("TURN"); 
            }
            // count++;
          }else{
            robot.lineFollow(BASE_SPEED);

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
            if(isGarage() && isKey() && primary){
              robot_state = ENDPRIMARY;
            }else if(isGarage() && isKey() && !primary){
              robot_state = ENDSECONDARY;
            }else{
              robot_state = DRIVE_LINE;
              Serial.println("DRIVE_LINE");
            }


            }
        break;
        case CLOSED_LEFT:
          if(robot.turnToNextline(-75)){

            if(isGarage() && isKey() && primary){
              robot_state = ENDPRIMARY;
            }else if(isGarage() && isKey() && !primary){
              robot_state = ENDSECONDARY;
            }else if(abs(irSensorFront.ReadData()) <= TOO_CLOSE){
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

            if(isGarage() && isKey() && primary){
              robot_state = ENDPRIMARY;
            }else if(isGarage() && isKey() && !primary){
              robot_state = ENDSECONDARY;
            }else{
             robot_state = DRIVE_LINE;
             Serial.println("DRIVE_LINE");
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
        case REC_MAP:
        String result[10];
        while(mqtt.checkSerial1()){
          Serial.print(mqtt.serString1);
        }
            
        break;
    }

  }


  


  /*
  state(accomplice)
  accomplice(){
    
  }
  state at 00 in reall position and 00 in struct
  line follow untill intersection is reached
  look at next index in map
  if x increments positively, turn to x positive direction and contine
  if x incements negatively, turn to x negative dirction and move
  if y increments positively, turn to y positive direction and move
  if y increments negatively, turn to y negative direction and move
  if it's the same coordinate do nothing
  comppare current position with 
  
  for(int i = 0; i < 54; i++){
    if(robot.reachedIntersection())
      robot.centerVTC();
      float xCurrent = accompliceMap.xCoords[i];
      float yCurrent = accompliceMap.yCoords[i];
      float xPast = accompliceMap.xCoords[i-1];
      float yPast = accompliceMap.yCoords[i-1];
      if (xcurrent !== xPast){
        if(yCurrent > yPast){
          robot.turnToNextLine(90);
        }
        else{
          robot.turnToNextLine(-90);
        }
      }
      else if(yCurrent == yPast){
        if(xCurrent > xPast){
          robot.turnToNextLine(0);
        }
        else{
          robot.turnToNextLine(180);
        }
      }
      else{
        Serial.println("ERROR");
      }

  }
  else{
    robot.lineFollow(45);
  }
  
  
  
  */
}