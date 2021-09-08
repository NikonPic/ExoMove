#ifndef _SDCARD_h_
#define _SDCARD_h_

#include <SD.h>

extern File dataFile; //Fileobjekt für SD Karte erzeugen

void initCard();
bool openFile_write(const char *filename);
void print2dataFile(unsigned long ms, unsigned int angleB, unsigned int angleA, unsigned int angleK, unsigned int forceB, unsigned int forceA);
void closeDataFile();

#endif