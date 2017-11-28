#include <Arduino.h>
#include <AutoNavigate.h>
//
///*
//  Auto Navigation
//  This code is written for Team RoverX.
//  The usage of this code is selectively licensed.
// */
//
//
//
uint8_t leftAW = 5,leftCW = 6,leftSpeed = 7,rightAW = 8,rightCW = 9,rightSpeed = 10 ;
AutoNavigate rover(leftCW,leftAW,leftSpeed,rightCW,rightAW,rightSpeed);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Started...");



}

// the loop routine runs over and over again forever:
void loop() {

    rover.setDestination(12.97060,79.1572);

    float d = rover.getDistance();
    Serial.print("distance: ");
    Serial.println(d);
    float heading = rover.getDirection();
    Serial.print("Required Angle: ");
    Serial.println(heading);
    rover.smartdelay(500);
    delay(500);
    rover.action();
}
