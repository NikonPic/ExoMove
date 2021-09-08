#include "SDCard.h"
#include <Arduino.h>

File dataFile;

//Aufruf in Setup, Karteninitialisierung
void initCard()
{
  bool ret;
  Serial.print("Initialisiere SD Karte...");
  ret = SD.begin(BUILTIN_SDCARD);
  if (!ret)
  {
    Serial.println("Card failed, or not present");
    while (1)
    {
    }
  }
  Serial.println("Karte initialisiert.");
}

//Aufruf in Setup, Vorbereitung des dataFile zum beschreiben
bool openFile_write(const char *filename)
{
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.flush();
  return (dataFile == true);
}
///////////////////////////////////////////////////////////////////////

void print2dataFile(unsigned long ms, unsigned int angleB, unsigned int angleA, unsigned int angleK, unsigned int forceB, unsigned int forceA)
{
  dataFile.printf("%5u;%4u;%4u;%4u;%4u;%4u\r\n", ms, angleB, angleA, angleK, forceB, forceA);
}

void closeDataFile() {
  dataFile.close();
}