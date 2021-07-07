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
    if (Generator::isStarted == false)
    {
        digitalWrite(RELAY_GEN_STOPPER, LOW);
        digitalWrite(RELAY_GEN_STARTER, HIGH);
        delay(1000*GEN_START_TIME);
        digitalWrite(RELAY_GEN_STARTER, LOW);
        Generator::isStarted = true;
    }
}
void Generator::stop(){
    digitalWrite(RELAY_GEN_STARTER, LOW);
    digitalWrite(RELAY_GEN_STOPPER, HIGH);
    Generator::isStarted = false;
}