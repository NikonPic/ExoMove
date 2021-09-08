#include "HM10Controller.h"

HM10Controller *HM10Controller::instance = new HM10Controller();

HM10Controller::HM10Controller() : iface(1, 0),
                                   initialized(false),
                                   connected(false),
                                   watchConnected("OK+CONN"),
                                   watchLost("OK+LOST"),
                                   watchStart("Start"),
                                   watchStop("Stop")
{
}

void HM10Controller::init()
{
    iface.begin(115200);
    int first, second;

    for (uint8_t trie = 0; trie < 5 && !initialized; ++trie)
    {

        iface.write("AT");

        delay(250);

        Serial.println(iface.available());

        if (iface.available() >= 2)
        {
            first = iface.read();
            second = iface.read();
            if (first == 'O' && second == 'K')
            {
                initialized = true;
            }
            Serial.print(first);
            Serial.print(second);
        }
    }
}

void HM10Controller::update(bool ignoreState)
{

    if (ignoreState || initialized)
    {
        while (iface.available())
        {
            uint8_t c = (uint8_t)iface.read();

            watchConnected.push(c);
            watchLost.push(c);
            watchStart.push(c);
            watchStop.push(c);

            if (watchConnected.matches())
            {
                connected = true;
                watchConnected.reset();
            }
            else if (watchLost.matches())
            {
                connected = false;
                watchLost.reset();
            }
        }
    }
}

bool HM10Controller::hasStarted()
{
    if (watchStart.matches())
    {
        watchStart.reset();
        return true;
    }
    else
        return false;
}

bool HM10Controller::hasStopped()
{
    if (watchStop.matches())
    {
        watchStop.reset();
        return true;
    }
    else
        return false;
}

void HM10Controller::write(const char *input)
{
    iface.write(input);
}

size_t HM10Controller::write(uint8_t *buffer, size_t size)
{
    return iface.write(buffer, size);
}

//
void getConnection()
{
    Serial.print("Initialising HM10 Module");

    delay(50);
    int go = 0;
    int init = 0;

    //initialise
    while (init == 0)
    {
        HM10Controller::instance->init();
        delay(500);
        if (HM10Controller::instance->testModule())
        {
            init = 1;
            Serial.println("Initialised");
        }
    }

    //connect
    while (go == 0)
    {
        HM10Controller::instance->update();
        delay(10);
        if (HM10Controller::instance->isConnected())
        {
            go = 1;
            Serial.println("Connected");
        }
    }
}

void testBinding()
{
    int go = 0;
    while (go++ < 10)
    {
        delay(1000);
        Serial.println("Test");
    }
}