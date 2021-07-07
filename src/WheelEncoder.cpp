#include "WheelEncoder.h"



WheelEncoder::WheelEncoder(int EncoderPin)
{
    e_pin = EncoderPin;
    pinMode(e_pin, INPUT); //Config encoder pin
}

void WheelEncoder::tick(){
    this->updateSpeed();
}

float WheelEncoder::getLinearSpeed(float wheelRadius){
    return currentSpeed_RpS*wheelRadius;
}

void WheelEncoder::updateSpeed(){
    if (abs(odom-prevousOdom) > 0)
    {
        currentSpeed_RpS=(ANGLE_PER_TICK*(odom-prevousOdom))/(millis()-e_pr_millis)*1000; 
        prevousOdom = odom;
        e_pr_millis = millis();
        return;
    }
        if (millis()-e_pr_millis > 500) //Wheel Stopped
        {
            currentSpeed_RpS=0;
            e_pr_millis = millis();
            prevousOdom = odom;
        }
}

