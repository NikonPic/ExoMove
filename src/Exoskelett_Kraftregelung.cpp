#include <Arduino.h>
#include <SD.h>
#include <math.h>

//personal libraries
#include <HM10Controller.h>
#include <SDCard.h>
#include <utils.h>
#include <SoftwareSerial.h>

//Programm fuer Experimente
//TODO: Anpassen der Kalibrierkurven fuer Kraftsensoren und Annaehrung Winkel in Gelenk B (Input Kraftregelung, wird nicht gespeichert)
//TODO: Anpassen der Kinematischen Parameter des Systems (Fingerglieder, Abstaende etc.)
//TODO: Test Aktorik

unsigned long breakTime; //Zeitstempel zu Anfang des Loops -> Ablegen des Zeitpunkts für Loopende

int start_per = 25;
int max_runs = 1;

//Counter für gewünschte Anzahl an Loopdurchläufen

int t = 0;        //Zählervariable für Loopdurchläufe
int num_runs = 0; //Zählervariable Anzahl der Durchläufe
int start_pos = (totalRuns * start_per) / 100;
int k = start_pos; //Zählervariable für Funktion Aktortrajektorie
///////////////////////////////////////////////////////////////////////

unsigned long setupTime;

//Kraftregelung
int dir = 1; //Flag für Richtung R=1 vorwärts, R=0 rückwärts
int per = 1; //Flag für aktuelle Periode 1:(0,pi) oder Periode 2:(pi,2pi)
int valSens; //Analog eingelesenen Sensorwert, soll Kraftwert simulieren

void setup()
{

  //Anfahren der Ausgangsposition des Aktors
  driveStart(start_pos);
  delay(5000);

  // map k to period
  per = setPeriod(k);

  Serial.begin(9600); //Starten Serial Monitor
  //while(!Serial); //Warten, bis Serial Monitor geöffnet
  delay(100);

  //Einstellungen für den ADC: Auflösung und Glättung der Messwerte
  analogReadResolution(12);
  analogReadAveraging(32); //8, 16, 32

  //Vorbereiten der SD Karte für Schreiben (Funktionen von Frank)
  initCard();
  openFile_write("log_1.txt");

  setupTime = millis();
  preStart();
}

void loop()
{
  // check HM10 first
  HM10Controller::instance->update();

  //Prüfen der Abbruchbedingung: Beenden der Messungen nach Anzahl totalRuns Loopdurchläufen
  if (HM10Controller::instance->hasStopped())
  {
    closeDataFile();
    Serial.println("Messungen abgeschlossen, Programm beendet");
    checkHM10();
  }

  //Positionieren des Aktors nach Loopdurchlaufsnummer
  positionActuator(k);

  //Zeitstempel: Aktueller Loopdurchlauf (Startet bei 0)
  unsigned long ms = millis() - setupTime;

  //Wert für Endzeitpunkt von aktuellem Loop erzeugen
  breakTime = millis() + loopTime;
  //Serial.print("Loop startet: ");

  //Einlesen der Positionssensorik (Potentiometer)
  unsigned int angleA = analogRead(potiA);
  unsigned int angleB = analogRead(potiB);
  unsigned int angleK = analogRead(potiK);

  //Mappen der Sensordaten auf Winkelwerte:
  Serial.print("   Winkel B: ");
  Serial.print(angleB);
  Serial.print("   Winkel A: ");
  Serial.print(angleA);
  Serial.print("   Winkel K: ");
  Serial.println(angleK);

  //Einlesen der Kraftsensorik (A201s)
  unsigned int forceB = analogRead(fSenB);
  unsigned int forceA = analogRead(fSenA);

  //Mappen der Sensordaten auf Winkelwerte:
  Serial.print("   Kraft Sensor B (vorne, Druck, Blau): ");
  Serial.print(forceB);
  Serial.print("   Kraft Sensor A (hinten, Zug, Gelb): ");
  Serial.println(forceA);

  //Bestimmen des momentanen Schwingenwinkels
  float phi_B_d = 0.0533 * angleB - 9.27; //Umrechnung des ADC Werts über lineare Regression von für Positionssensorik an Gelenk B
  float phi_B = phi_B_d * (pi / 180);     //Umrechnung des Winkels in Radiant

  //Bestimmen der Kräfte
  float F_back = 0.00477458 * forceA - 6.32508; //Kraftsensor A
  float F_front = 0.0049497 * forceB - 7.17406; //Kraftsensor B

  //Berechnen des Antriebsmoments aus Aktorkraft
  float F_akt = F_front - F_back;                        //Differenz der Kraftsensoren ist aktuelle Aktorkraft auf Kinematik
  float M_akt = getActuationTorque(phi_B, F_akt) / 1000; //Nm aus Nmm

  float F_reg = abs(F_akt);
  Serial.print("F_reg = ");
  Serial.println(F_reg);

  //Ueberschreiben mit fixem Wert fuer Demo an Hand (keine Positionssensorik verbaut)

  //float M_akt = 10; //Regler umgehen, immer Vorwärts
  Serial.print("M_akt = ");
  Serial.println(M_akt);

  //Zweipunktregler für Kraftregelung
  //Flagprüfung: Betriebsmodus feststellen
  switch (dir)
  {
  case 1: //Richtung = Vorwärts, Oberer Grenzwert relevant
    //Prüfung Kraftgrenzwert
    if (F_reg < F_upper)
    {
      //Grenzwert nicht erreicht, im Modus Vorwärts bleiben
      //Begrenzung auf maximalen Wert totalRuns
      k++; //Variable für Schwingenwinkel um 1 erhöhen
    }
    else
    {
      //Grenzwert überschritten, umkehren der Bewegungsrichtung in Rückwärts
      dir = 0; //Umschalten der Flagvariable für Richtung
      Serial.println("oberer Grenzwert überschritten, Wechsel in Rückwärtsbewegung");
    }
    break;
  case 0: //Richtung = Rückwärts, unterer Grenzwert relevant
    //Prüfung Kraftgrenzwert
    if (F_reg > F_lower)
    {
      //Grenzwert immer noch zu hoch, im Modus rückwärts bleiben

      //WICHTIG: Periodizität beachten!
      //In erster Periode darf nach einer Bewegungsumkehrung nicht unter 0 gefahren werden
      //In zweiter Periode darf nach Bewegungsumkehrung nicht unter pi gefahren werden
      //Andernfalls wird der kritische Winkel nach Umkehr der Bewegung wegen Periodizität erzeut durchschritten!

      switch (per)
      {
      case 1: //Periode 1
        if (k > 0)
        {
          k--;
        }
        else
        {
          //nicht weiter reduzieren (Periodizität)
        }
        break;
      case 2: //Periode 2
        if (k > totalRuns / 2)
        {
          k--;
        }
        else
        {
          //nicht weiter reduzieren (Periodizität)
        }
        break;
      } //Ende der Fallunterscheidung Periode
    }   //Ende Fall Grenzwert zu hoch

    else
    {
      //Kraftwert ausreichen abgefallen, umkehren der Bewegungsrichtung in Vorwärts
      dir = 1; //Umschalten der Flagvariable für Richtung
      Serial.println("unterer Grenzwert unterschritten, wechsel in Vorwärtsbewegung");
    }
    break; //Ende Case Rückwärts
  }        //Ende Zweipunktregelung

  per = setPeriod(k);

  //Schreiben der erhobenen Daten auf die SD Karte als 32Byte String
  print2dataFile(ms, angleB, angleA, angleK, forceB, forceA);

  //Prüfen, ob gesetzte Soll-Loopzeit verstrichen ist - wenn nicht, warten bis neuer Loop beginnen kann
  while (millis() < breakTime)
  {
    //waiting for loop time to finish - do nothing
  } //Loopdauer

} //Loop Funktion
