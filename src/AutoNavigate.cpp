//
// Created by rahul on 23/10/17.
//

#include "AutoNavigate.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>

// Create a compass
HMC5883L_Simple Compass;
TinyGPS gps;
SoftwareSerial ss(4, 3);


AutoNavigate::AutoNavigate(uint8_t _leftCW,uint8_t _leftAW,uint8_t _rightCW,uint8_t _rightAW,uint8_t _leftSpeed,uint8_t _rightSpeed) {
    Serial.println("Automation Stated...");


    leftCW = _leftCW ;
    leftAW = _leftAW;
    rightCW = _rightCW;
    rightAW = _rightAW;
    leftSpeed = _leftSpeed;
    rightSpeed = _rightSpeed;
    Serial.begin(115200);
    ss.begin(4800);
    Wire.begin();

}

void AutoNavigate::setDestination(int ft, int fg) {
    flat = ft;
    flon = fg;
    Serial.print("set lat: ");
    Serial.println(flat);
    Serial.print("set lon: ");
    Serial.println(flon);

}

float AutoNavigate::getDistance() {
    //--get location from gps--

//    float clat, clon;
    unsigned long age, date, time, chars = 0;
    unsigned short sentences = 0, failed = 0;
    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
    gps.f_get_position(&clat, &clon, &age);
    if (clat == TinyGPS::GPS_INVALID_F_ANGLE)
    {
        Serial.println("*invalid*");
    }
    else
    {
        Serial.print(clat, 6);
        int vi = abs((int)clat);
        int flen = 6 + (clat < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i=flen; i<10; ++i)
            Serial.print(' ');
    }
    if (clon == TinyGPS::GPS_INVALID_F_ANGLE)
    {
        Serial.println("*invalid lon*");
    }
    else
    {
        Serial.print(clon, 6);
        int vi = abs((int)clon);
        int flen = 6 + (clon < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i=flen; i<10; ++i)
            Serial.print(' ');
    }
    Serial.println();

    smartdelay(1000);

    delT = radians(flat-clat);
    delG = radians(flon-clon);
    delT=radians(flat-clat);
    delG=radians((flon)-(clon));
    flat=radians(flat);
    clat=radians(clat);

    dist_calc = (sin(delT/2.0)*sin(delT/2.0));
    dist_calc2= cos(clat);
    dist_calc2*=cos(flat);
    dist_calc2*=sin(delG/2.0);
    dist_calc2*=sin(delG/2.0);
    dist_calc +=dist_calc2;
    dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
    dist_calc*=6371000.0; //Converting to meters
    Serial.println("distance");
    Serial.println(dist_calc);    //print the distance in meters
    return dist_calc;
}

 void AutoNavigate::smartdelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}

float AutoNavigate::getDirection()
{
    float heading = atan2(sin(flon - clon) * cos(flat), cos(clat) * sin(flat) - sin(clat) * cos(flat) * cos(flon - clon));

    heading = heading * 180 / 3.1415926535;  // convert from radians to degrees

    int head = heading; //make it a integer now

    if (head < 0) {

        heading += 360;   //if the heading is negative then add 360 to make it positive

    }

    Serial.println("heading:");
    Serial.println(heading);   // print the heading.
    return heading;
}
float AutoNavigate::myDirection()
{

    Compass.SetDeclination(-1, 14, 'W');
    Compass.SetSamplingMode(COMPASS_SINGLE);
    Compass.SetScale(COMPASS_SCALE_088);
    Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
    currentDirection = Compass.GetHeadingDegrees();
    return currentDirection;

}
void AutoNavigate::action()
{
    //--calculate which direction to move--
    delta = currentDirection-AutoNavigate::getDirection();
    char turn = 's';
    bool neg = 0;
    if(delta>=-180){

        if(delta<=0){

            turn= 'r';    // turn  "right"
            neg = 1;

        }

    }

    if(delta<-180){

        turn= 'l';      // turn  "left"
        neg = 1;

    }

    if(delta>=0){

        if(delta<180){

            turn= 'l';   // turn  "left"
            neg = 0;

        }

    }

    if(delta>=180){     // turn "right"

        turn = 'r';
        neg = 0 ;

    }
    if(delta==0){

        turn= 's';   // go "straight"

    }
    if (!neg && turn=='l')
    {
        delta = map((long)delta,0,180,0,510);
        analogWrite(rightSpeed,abs(510+delta));
        analogWrite(leftSpeed,abs(510-delta));
        digitalWrite(leftAW,LOW);
        digitalWrite(leftCW,HIGH);
        digitalWrite(rightAW,LOW);
        digitalWrite(rightCW,HIGH);

    }

    if (!neg && turn=='r')
    {
        delta = map((long)delta,0,180,0,510);
        analogWrite(rightSpeed, abs(510-delta));
        analogWrite(leftSpeed,abs(510+delta));
        digitalWrite(leftAW,LOW);
        digitalWrite(leftCW,HIGH);
        digitalWrite(rightAW,LOW);
        digitalWrite(rightCW,HIGH);

    }
    if (neg && turn=='l')
    {
        delta = map((long)delta,0,180,0,510);
        analogWrite(rightSpeed, abs(510-delta));
        analogWrite(leftSpeed,abs(510+delta));
        digitalWrite(leftAW,LOW);
        digitalWrite(leftCW,HIGH);
        digitalWrite(rightAW,LOW);
        digitalWrite(rightCW,HIGH);

    }
    if (neg && turn=='r')
    {
        delta = map((long)delta,0,180,0,510);
        analogWrite(rightSpeed, abs(510+delta));
        analogWrite(leftSpeed,abs(510-delta));
        digitalWrite(leftAW,LOW);
        digitalWrite(leftCW,HIGH);
        digitalWrite(rightAW,LOW);
        digitalWrite(rightCW,HIGH);

    }
    if (turn=='s')
    {
        analogWrite(rightSpeed, 510);
        analogWrite(leftSpeed,510);
        digitalWrite(leftAW,LOW);
        digitalWrite(leftCW,HIGH);
        digitalWrite(rightAW,LOW);
        digitalWrite(rightCW,HIGH);
    }

}


