#ifndef _HM10CONTROLLER_h_
#define _HM10CONTROLLER_h_

#include <SoftwareSerial.h>
#include <StreamWatcher.h>

//CODE: 001122
class HM10Controller
{
protected:
    StreamWatcher<7> watchConnected, watchLost;
    StreamWatcher<5> watchStart;
    StreamWatcher<4> watchStop;
    SoftwareSerial iface;
    bool initialized, connected;
    void update(bool ignoreState);

public:
    static HM10Controller *instance;

    HM10Controller();
    void init();
    inline void update() { update(false); }
    inline bool isConnected() const { return connected; }
    inline bool testModule() const { return initialized; }
    bool hasStarted();
    bool hasStopped();
    void write(const char *input);
    size_t write(uint8_t *buffer, size_t size);
};

#endif