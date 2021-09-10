#ifndef _UTILS_h_
#define _UTILS_h_
#include <Arduino.h>

const unsigned int loopTime = 10;       //Wert für gewünschtes Loopintervall in Millisekunden
const int totalRuns = 22000 / loopTime; //(10 Sekunden pro Bewegung)

//Parametrierung der Aktorik///////////////////////////////////////////
const float l_akt = 139.3f;        //102mm Aktorlänge von Loch zu Loch + Kraftsensorlänge von 37.3mm
const float d_b = 161.062f;        //Abstand Glelenk B zu Mountgelenk Aktorrückseite
const float akt_x = -152.12f;      //Abstand in X-Richtung Welt KS Gelenk B zu Mountgelenk Aktorrückseite in mm
const float akt_y = -21.79f;       //Abstand Y-Richtung Gelenk B zu Mountgelenk Aktorrückseite in mm
const float alpha_const = 0.0612f; //Winkel zwischen d_B und X-Achse Weltsysten in rad (3.5°)
const float theta = 0.6109f;       //Winkel zwischen l_1 und l_2 in rad (35°)
const float l_s = 20.0f;           //Kurzer Arm der Antriebsschwinge
//const float phi_B_min = 0.3491f; //20°, Minimalwert des Schwingenwinkels -> Berechnung max. Aktorstellung
const float phi_B_max = 2.2340f; //128°, Maximalwert des Schwingenwinkels -> Berechnung min. Aktorstellung
//Überschreiben für Demo: Weniger Auslenkung
const float phi_B_min = 0.9235f;           //30°
const float delta_phi_max = 108.000f;      //Maximales Intervall der Antriebsschwinge in Grad
const float val_count_phi_B_max = 5.0f;    //Startwert für Loopcounter, sodass Aktor aus seiner Minimalposition heraus startet (Anfahren in Setup-Funktion!)
const float pi = 3.1416f;                  //Konstante für Wert von pi erzeugen
const float multip = (2 * pi) / totalRuns; //Aufteilung von 2pi auf die gesamte Anzahl an Durchläufen
const int aktorpin = 29;                   //Signalpin PWM-Output für Aktor (Kopie von oben)
///////////////////////////////////////////////////////////////////////

//Positionssensorik
const int potiA = A2;
const int potiB = A1;
const int potiK = A0;

//Kraftsensorik
const int fSenB = A10; // Blau
const int fSenA = A12; //Gelb

////////////////////////////////////////////////////////////////////////
const float M_upper = 0.05; //Oberer Grenzwert für Moment [Nm] -> Umkehrung von Vorwärts nach Rückwärts
const float M_lower = 0.01; //Unterer Grenzwert für Moment -> Umkehrung von Rückwärts nach Vorwärts

const float F_upper = 5;   //Oberer Grenzwert für Moment [Nm] -> Umkehrung von Vorwärts nach Rückwärts
const float F_lower = 0.4; //Unterer Grenzwert für Moment -> Umkehrung von Rückwärts nach Vorwärts

// Function definitions
float getAngle(float p1_x, float p1_y, float p2_x, float p2_y);
float getActuationSignal(float phi);
float getNomPosition(int k);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float mapActuatorInput(float nomPosition);
void positionActuator(int k);
void driveStart(int k);
void posManual(int inputpin, int aktorpin);
int setPeriod(int k);
float getActuationTorque(float phi_B, float F_Aktor);
void getReadyToStart();
#endif