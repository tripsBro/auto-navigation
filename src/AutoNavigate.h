//
// Created by rahul on 23/10/17.
//

#ifndef AUTONAV_AUTONAVIGATE_H
#define AUTONAV_AUTONAVIGATE_H

#include <avr/io.h>

class AutoNavigate {

public:
    //--constructor--
    AutoNavigate(uint8_t leftCW,uint8_t leftAW,uint8_t rightCW,uint8_t rightAW,uint8_t leftSpeed,uint8_t rightSpeed);

    //--functions--
    void setDestination(int flat,int flon);
    float getDistance();
    float getDirection();
    float myDirection();
    void action();
    static void smartdelay(unsigned long ms);

private:
    //--defining variables--
    float flat,flon,clat,clon,dist_calc,dist_calc2,delT,delG,currentDirection;
    float delta;
    uint8_t  leftCW, leftAW, rightCW,rightAW, leftSpeed,rightSpeed;


};

#endif //AUTONAV_AUTONAVIGATE_H
