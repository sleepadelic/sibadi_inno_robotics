#include <Arduino.h>
#define ANGLE_PER_TICK 0.083775804
class WheelEncoder
{
private:
    
    void updateSpeed();
    long prevousOdom = 0;
    unsigned long e_pr_millis = 0;
    unsigned long e_state_millis = 0;
    boolean e_change = false;

public:
    uint8_t e_pin;
    WheelEncoder(int EncoderPin);
    float getLinearSpeed(float wheelRadius);
    long odom = 0; //Current position in ticks
    float currentSpeed_RpS = 0.0;
    void tick();
};


