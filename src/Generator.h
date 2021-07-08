#include "robot_config.h"
#include "Arduino.h"

class Generator
{
private:
    /* data */
public:
    bool isStarted = false;
    Generator(/* args */);
    ~Generator();
    void start();
    void stop();
};

Generator::Generator(/* args */)
{
}

Generator::~Generator()
{
    Generator::stop();
}

void Generator::start(){
    
    digitalWrite(RELAY_GEN_STOPPER, HIGH);
    digitalWrite(RELAY_GEN_STARTER, HIGH);
    delay(GEN_START_TIME*1000);
    digitalWrite(RELAY_GEN_STARTER, LOW);
}
void Generator::stop(){
    digitalWrite(RELAY_GEN_STARTER, LOW);
    digitalWrite(RELAY_GEN_STOPPER, LOW);
}