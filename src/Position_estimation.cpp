#include  "Position_estimation.h"
#include "Encoders.h"

/**
    Arrays of x and y, when there are coordinates smooshed between others we will set those in between ones to the outside ones
    {1,2},{1,3},{1,2} means that {1,3} is a dead end, so we will change this to {1,2},{1,2},{1,2}
*/
Encoder RomiEncoders;

/*
    Use arrays for x and y, then repeat the last real coordinate when we delete dead ends. Double the size of the array when it 
    gets too big
*/
unsigned long time_prev = 0;
unsigned long time_now = millis();
float timeIncrement = 0.050;
const int coordinateListSize = 54;
int currentCoordinate = 0;
bool second = false;

//Based on how things are run right now, we can go up to 54 coordinates
struct coordinates{
    float xCoords[coordinateListSize] = {0};
    float yCoords[coordinateListSize] = {0};
};
coordinates myCoordinates;

void Position::Init(void)
{
    time_now = millis();
    x = 0;
    y = 0;
    theta = 0;
    myCoordinates.xCoords[currentCoordinate] = 0;
    myCoordinates.yCoords[currentCoordinate] = 0;
    currentCoordinate++;
}
float Position::getXCoordinate(int i){
    return myCoordinates.xCoords[i];
}
float Position::getYCoordinate(int i){
    return myCoordinates.yCoords[i];
}
void Position::makeWaypoint(void){
    if(currentCoordinate < coordinateListSize){
        myCoordinates.xCoords[currentCoordinate] = x;
        myCoordinates.yCoords[currentCoordinate] = y;
        currentCoordinate++;
    }

}

void Position::cleanMapFirst(void){
    //Find where coordinates are sandwiched between identical coordinates
        //find identical coordinates that are not next to eachother
        //replace the coordinates in between them with that coordinate (this is easier than deleting and redoing the map)
    for(int i = 0; i < currentCoordinate-1; i++){
        for(int j = ++i; j < currentCoordinate; j++){
            //Check for bread of sandwich
            if(myCoordinates.xCoords[i] == myCoordinates.xCoords[j] 
                && myCoordinates.yCoords[i] == myCoordinates.yCoords[j]){
                for(int a = i; a < j; a++){
                    myCoordinates.xCoords[a] = myCoordinates.xCoords[i];
                    myCoordinates.yCoords[a] = myCoordinates.yCoords[i];
                }
            }
        }
    }
}
void Position::printMap(void){
    Serial.println("THE MAP OH MY GOD");
    for(int i = 0; i < currentCoordinate; i++){
        Serial.print(myCoordinates.xCoords[i]);
        Serial.print("  ");
        Serial.println(myCoordinates.yCoords[i]);
    }
}
void Position::Stop(void)
{
    time_now = millis();
    x = 0; 
    y = 0;
    theta = 0;
}

float Position::getX()
{
    return x;
}

float Position::getY()
{
    return y;
}

float Position::getTheta()
{
    return theta;
}

float Position::getThetaDeg()
{
    return theta*(180/PI);
}


void Position::PrintPose(void)
{
    // Serial.print(x);
    // Serial.print("  ");
    // Serial.print(y);
    // Serial.print("  ");
    // Serial.print(theta);
    // Serial.print(" ");
//     Serial.print("|");
//     Serial.print(" ");
//     Serial.print(x_theoretical);
//     Serial.print("  ");
//     Serial.print(y_theoretical);
//     Serial.print("  ");
//     Serial.println(theta_theoretical);
}

void Position::UpdatePose(float measured_speed_left, 
    float measured_speed_right) //target speed in mm/s
{
        time_prev = millis();
         if(abs(measured_speed_right - measured_speed_left) > .25 ) {
            //Serial.println("Curved");
           //measured calculations
            float R_measured = (l/2)*((measured_speed_right + measured_speed_left) 
                / (measured_speed_right - measured_speed_left));
            float w_measured = (measured_speed_right - measured_speed_left) / l;

            x_calculated = x_calculated - (R_measured * sin(theta_calculated)) + 
                (R_measured * sin((theta_calculated + (w_measured * timeIncrement))));

            y_calculated += (R_measured * cos(theta_calculated)) - 
                (R_measured * cos(theta_calculated + (w_measured * timeIncrement)));
            
            theta_calculated += w_measured * timeIncrement;
            

        } else {
            //straight movement
            //Serial.println("Straight");

            //measured calculations
            float V_calculated = (measured_speed_left + measured_speed_right)/2;
            x_calculated += V_calculated * cos(theta_calculated) * timeIncrement; //50ms is the set time update
            y_calculated += V_calculated * sin(theta_calculated) * timeIncrement;
            
        }

        if(theta_calculated > 2*PI){
            theta_calculated -= 2*PI;
        }else if(theta_calculated < 0){
            theta_calculated += 2*PI;
        }

        x = x_calculated;
        y = y_calculated;
        theta = theta_calculated;
        //PrintPose();
}

//untested depricated
void Position::resetOdomytry(){
    float heading = getThetaDeg();
    
    Serial.print("heading");

    Serial.println(heading);

    //0 degree case
    if (heading > 315 || heading < 45){
            Serial.print("0 degree case");
                        Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("Old Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
            myCoordinates.xCoords[currentCoordinate] = 
            myCoordinates.xCoords[currentCoordinate-1]+1;
            myCoordinates.yCoords[currentCoordinate] = 
            myCoordinates.yCoords[currentCoordinate-1];
                        Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("New Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
    }
    //90 degree case
    else if (heading > 45 && heading < 135){
                    Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("Old Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
            myCoordinates.yCoords[currentCoordinate] = 
            myCoordinates.yCoords[currentCoordinate-1]+1;
            myCoordinates.xCoords[currentCoordinate] = 
            myCoordinates.xCoords[currentCoordinate-1];
                        Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("New Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
    }
    // 180 degree case
    else if (heading > 135 && heading < 225){
                    Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("Old Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
            myCoordinates.xCoords[currentCoordinate] = 
            myCoordinates.xCoords[currentCoordinate-1]-1;
            myCoordinates.yCoords[currentCoordinate] = 
            myCoordinates.yCoords[currentCoordinate-1];
                        Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("New Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
        Serial.print("180 degree case");
    }
    //270 degree case
    else if (heading > 225 && heading < 315){
            Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("Old Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
            myCoordinates.yCoords[currentCoordinate] = myCoordinates.yCoords[currentCoordinate-1]-1;
            myCoordinates.xCoords[currentCoordinate] = myCoordinates.xCoords[currentCoordinate-1];
            Serial.print("Current Coordinate: ");
            Serial.println(currentCoordinate);
            Serial.print("New Coordinates: (x,y) ");
            Serial.print(myCoordinates.xCoords[currentCoordinate]);
            Serial.print("  ");
            Serial.println(myCoordinates.yCoords[currentCoordinate]);
        Serial.print("270 degree case");
    }
    else{
        Serial.print("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<___________________error_________________>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    }
    
    currentCoordinate++;


  //always orent the robot with x facing x+
//increment world waypoint counter
// if we are at 0, increment x by 1
// if we are at 90, increment y by 1
//if we are at 180, decrement x by 1
//if we are at 270, decrement y by 1 
//assume x and y unit are equivelent to 40cm -> 400mm
  //if(robot.getTheta())
}