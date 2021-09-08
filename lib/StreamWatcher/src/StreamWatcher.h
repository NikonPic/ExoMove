// StreamWatcher.h

#ifndef _STREAMWATCHER_h
#define _STREAMWATCHER_h

#include "arduino.h"

template<uint8_t string_length>
class StreamWatcher
{
 protected:
	 char reference[string_length];
	 uint8_t index;

 public:
	 StreamWatcher(char* reference);
	 inline bool matches() { return index == string_length; }
	 inline void reset() { index = 0; }
	 void push(uint8_t item);
};

template<uint8_t string_length>
inline StreamWatcher<string_length>::StreamWatcher(char * reference) : index(0)
{
	memcpy(this->reference, reference, string_length);
}

template<uint8_t string_length>
inline void StreamWatcher<string_length>::push(uint8_t item)
{
	if (index < string_length) {
		if (reference[index] == item)
			index++;
		else
			index = 0;
	}
}

#endif
