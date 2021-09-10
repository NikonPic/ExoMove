#include "HM10Controller.h"

HM10Controller *HM10Controller::instance = new HM10Controller();

HM10Controller::HM10Controller() : iface(0, 1),
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
    delay(5000);

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
void preStart()
{
    initHM10();
    connectHM10();
    startHM10();
}

void initHM10()
{
    int init = 0;
    //initialise
    while (init == 0)
    {
        HM10Controller::instance->init();
        delay(250);
        if (HM10Controller::instance->testModule())
        {
            init = 1;
            Serial.println("Initialised");
        }
    }
}

void connectHM10()
{
    int conn = 0;
    //connect
    while (conn == 0)
    {
        HM10Controller::instance->update();
        delay(10);
        if (HM10Controller::instance->isConnected())
        {
            conn = 1;
            Serial.println("Connected");
        }
    }
}

void startHM10()
{
    int go = 0;
    while (go == 0)
    {
        HM10Controller::instance->update();
        delay(10);
        if (HM10Controller::instance->hasStarted())
        {
            go = 1;
        }
    }
}

void checkHM10()
{
    if (HM10Controller::instance->hasStarted())
    {
        return;
    }
    if (HM10Controller::instance->isConnected())
    {
        startHM10();
        return;
    }
    if (HM10Controller::instance->testModule())
    {
        connectHM10();
        startHM10();
        return;
    }
    initHM10();
    connectHM10();
    startHM10();
    return;
}

void write2ptr(int pos, unsigned int value, uint8_t *ptr)
{
    ptr[pos + 0] = value >> 8;     // high byte (0x12)
    ptr[pos + 1] = value & 0x00FF; // low byte (0x34)
}

void sendMessageHM10(unsigned int ms, unsigned int angleB, unsigned int angleA, unsigned int angleK, unsigned int forceB, unsigned int forceA)
{
    // init array
    uint8_t messageApp[12];
    uint8_t *ptr = messageApp;

    write2ptr(0, ms, ptr);
    write2ptr(2, angleB, ptr);
    write2ptr(4, angleA, ptr);
    write2ptr(6, angleK, ptr);
    write2ptr(8, forceB, ptr);
    write2ptr(10, forceA, ptr);

    HM10Controller::instance->write(ptr, 12);
}