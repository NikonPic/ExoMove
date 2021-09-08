#include "HM10Controller.h"

void HM10Controller::init()
{
    iface.begin(115200);

    for (uint8_t trie = 0; trie < 5 && !initialized; ++trie)
    {

        iface.write("AT");

        delay(250);

        if (iface.available() >= 2)
        {
            if (iface.read() == 'O' && iface.read() == 'K')
            {
                initialized = true;
            }
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