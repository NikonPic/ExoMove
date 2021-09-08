#include <Arduino.h>
#include <SD.h>
#include <math.h>

//personal libraries
//#include <HM10Controller.h>
#include <SDCard.h>

//Programm fuer Experimente
//TODO: Anpassen der Kalibrierkurven fuer Kraftsensoren und Annaehrung Winkel in Gelenk B (Input Kraftregelung, wird nicht gespeichert)
//TODO: Anpassen der Kinematischen Parameter des Systems (Fingerglieder, Abstaende etc.)
//TODO: Test Aktorik

//Laufzeit und Loggingfrequenz einstellen ////////////////////////////
//Setzen von fester Loopedauer über Timer im Hintergrund, sodass der Controller parallel arbeiten kann
unsigned int loopTime = 10; //Wert für gewünschtes Loopintervall in Millisekunden
unsigned long breakTime;    //Zeitstempel zu Anfang des Loops -> Ablegen des Zeitpunkts für Loopende

int start_per = 25;
int max_runs = 1;

//Counter für gewünschte Anzahl an Loopdurchläufen
int totalRuns = 22000 / loopTime; //(10 Sekunden pro Bewegung)
int t = 0;                        //Zählervariable für Loopdurchläufe
int num_runs = 0;                 //Zählervariable Anzahl der Durchläufe
int start_pos = (totalRuns * start_per) / 100;
int k = start_pos; //Zählervariable für Funktion Aktortrajektorie
///////////////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////////////

// Function definitions
float getAngle(float p1_x, float p1_y, float p2_x, float p2_y);
float getActuationSignal(float phi);
float getNomPosition(int k);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float mapActuatorInput(float nomPosition);
void positionActuator(int k);
void driveStart(int k);
void posManual(int inputpin, int aktorpin);
void setPeriod();
float getActuationTorque(float phi_B, float F_Aktor);
void initCard();
void getReadyToStart();

//Funktionen zur Berechnung der Aktorvariablen/////////////////////////
//Berechnungen von Sollstellung s des Aktors aus Schwingenwinkelvorgabe [Verifiziert durch Vergleich mit MATLAB]
float getActuationSignal(float phi)
{
  float alpha = pi - theta - phi + alpha_const;
  //Serial.print("alpha= ");
  //Serial.println(alpha);
  float l = sqrtf((powf(l_s, 2) + powf(d_b, 2) - 2 * l_s * d_b * cosf(alpha))); //Kosinussatz zur Berechnung der Gesamtlänge des Aktorverbands l
  //Serial.print("l= ");
  //Serial.println(l);
  float s_nom = l - l_akt;
  return s_nom;
}

// Funktionen zur Aktorsteuerung //////////////////////////////////////

//Signalpin PWM-Output für Aktor (Kopie von oben)
const int aktorpin = 29;

//Berechnung der nominellen Position des Aktors, aus nominellem Schwingenwinkel phi_B, aus Timer/Counter t (Zahl Loopdurchläufe) [Verifiziert mit MATLAB]
float getNomPosition(int k)
{
  //Berechnung des nominellen Schwingenwinkels aus Loopcounter
  float sinsig = k * multip; //Winkelstellung des Sinussignals basierend auf Loopdurchlaufnummer berechnen (Intervall 0-2*pi auf 10s)
  //Serial.print("sinsig= ");
  //Serial.println(sinsig);
  float phi_raw = sinf(sinsig + 0.5 * pi); //Sinusschwingung 0.1Hz mit Amplitude [-1,1], Phasenverschiebung um pi/2, sodass bei counter = 0 (Bewegungsbeginn) mit Maximalwert gestartet wird
  //Serial.print("phi_raw= ");
  //Serial.println(phi_raw);
  float phi_d = (phi_raw * delta_phi_max / 2); //Skalierungsfakor ist halbes maximales Winkelintervall
  //Serial.print("phi_d ");
  //Serial.println(phi_d);
  phi_d = phi_d + delta_phi_max / 2 + (phi_B_min / pi) * 180; // Offsetfaktor positioniert das Intervall richtig in y-Richtung, sodass phi_B_max erreicht wird (Angegeben in Grad)
  //Serial.print("phi_d ");
  //Serial.println(phi_d);
  float phi_r = (phi_d / 180) * pi; //Konvertierung des nominellen Schwingenwinkels von Deg nach Rad
  //Serial.print("phi_r ");
  //Serial.println(phi_r);
  //Berechnung der korrespondierenden Schubstangenposition
  float nomPosition = getActuationSignal(phi_r);
  return nomPosition;
}

//Funktion zum Mappen von Kommazahlen [Tested]
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  float val_mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return val_mapped;
}

//Mappen des Ausgangssignals auf max. Range des Aktors (0-3.3V PWM = 0-33mm Aktorzustellung) [Tested]
float mapActuatorInput(float nomPosition)
{
  //Max. Range Aktor 0-33mm, max. Range Input = 0-34.01mm
  float nomPositionMapped = mapf(nomPosition, 0, 34.01, 0, 33);
  return nomPositionMapped;
}

//Ansteuern des Aktors basierend auf in Loop vorgegebenem Sollwert für s
void positionActuator(int k)
{
  //Bestimmen der nominellen Aktorposition basieren auf Loopdurchlauf
  float nomPos = getNomPosition(k);
  //Mappen der nominellen Position auf Range des Aktors
  float nomPosMapped = mapActuatorInput(nomPos);
  //Schreiben der Position als PWM Signal am Aktorpin //Begrenzte Auflösung AnalogWrite!
  int nomPosInt = nomPosMapped * 1000;
  int outputsignal = map(nomPosInt, 0, 33000, 0, 256); //Range analogWrite von 0-256, Umrechnen der Kommawerte in der Position auf int Werte
  //Serial.print("Outputsignal ");
  //Serial.println(outputsignal);
  analogWrite(aktorpin, outputsignal);
}

void driveStart(int k)
{
  pinMode(aktorpin, OUTPUT);
  analogWriteFrequency(aktorpin, 1000);
  positionActuator(k);
}

///////////////////////////////////////////////////////////////////////

//Funktionen zur Verarbeitung des eingelesenen Positionssensors an Gelenk B/////////////
//Positionssensoren:
unsigned int readAngleRadB(int potipin)
{
  unsigned int rawData = analogRead(potipin);
  unsigned int DataMapped = rawData; //TODO: Verarbeitung der Kalibrierungskurven
  //Geradengleichung benutzen, um kalibrierte Winkelstellung aus ADC Wert zu berechnen
  return DataMapped;
}

//Wegen verschiedenen Kalibriergrade werden eigene Funktionen für jeden Sensor gebraucht!
///////////////////////////////////////////////////////////////////////

//Funktionen zur Verstellung der Aktors////////////////////////////////
//Bestimmen des Stellsignals
//float getNomPosition (float nomPhi) {
//  return PosNom;
//}

//Manuelle Verstellung des Aktors über Drehpoti
void posManual(int inputpin, int aktorpin)
{
  int intputsignal = analogRead(inputpin);
  int outputsignal = map(intputsignal, 0, 1023, 0, 333);
  Serial.println(outputsignal);
  analogWrite(aktorpin, outputsignal);
}

//////////////////////////////////////////////////////////////////////

//Zuweisen der MCU Pins ///////////////////////////////////////////////

//Positionssensorik
const int potiA = A2;
const int potiB = A1;
const int potiK = A0;

//Kraftsensorik
const int fSenB = A10; // Blau
const int fSenA = A12; //Gelb

////////////////////////////////////////////////////////////////////////

unsigned long setupTime;

//Kraftregelung
int dir = 1; //Flag für Richtung R=1 vorwärts, R=0 rückwärts
int per = 1; //Flag für aktuelle Periode 1:(0,pi) oder Periode 2:(pi,2pi)
int valSens; //Analog eingelesenen Sensorwert, soll Kraftwert simulieren

float M_upper = 0.05; //Oberer Grenzwert für Moment [Nm] -> Umkehrung von Vorwärts nach Rückwärts
float M_lower = 0.01; //Unterer Grenzwert für Moment -> Umkehrung von Rückwärts nach Vorwärts

float F_upper = 5;   //Oberer Grenzwert für Moment [Nm] -> Umkehrung von Vorwärts nach Rückwärts
float F_lower = 0.4; //Unterer Grenzwert für Moment -> Umkehrung von Rückwärts nach Vorwärts

// setzte die periode
void setPeriod()
{
  if (k < totalRuns / 2)
  {
    //Serial.print("Periode = ");
    per = 1;
    //Serial.println(per);
  }
  if (k > totalRuns / 2)
  {
    //Serial.print("Periode = 2");
    per = 2;
    //Serial.println(per);
  }
}

//Berechnung des aktuell anliegenden Antriebsmoments aus Schwingenwinkel und gemessener Aktorkraft [Verifiziert durch Vergleich mit MATLAB]
float getActuationTorque(float phi_B, float F_Aktor)
{
  float alpha_quer = pi - theta - phi_B; //Winkel zwischen X-Achse und l_s
  //Serial.print("alpha_quer= ");
  //Serial.println(alpha_quer);
  float S_x = -l_s * cosf(alpha_quer);            //x-Position der Spitze des kurzen Schwingenarms
  float S_y = l_s * sinf(alpha_quer);             //y-Position der Spitze des kurzen Schwingenarms
  float phi_F = getAngle(akt_x, akt_y, S_x, S_y); // Funktion: Berechnet aktuellen Kraftvektorwinkel im Raum
  //Serial.print("phi_F= ");
  //Serial.println(phi_F);
  float F_Akt_x = F_Aktor * cosf(phi_F); //Komponenten des Kraftvektors in Weltsystem
  float F_Akt_y = F_Aktor * sinf(phi_F);
  float M_akt = (F_Akt_x * S_y + F_Akt_y * (-S_x)); //[Nmm], Berechnen des Moments um Punkt B, pos. Richtung s. Skizze, pos. delta_x führt zu neg. Moment nach Skizze
  return fabs(M_akt);
}

//Benötigte Funktion zur Berechnung des Kraftvektorwinkels [Verifiziert durch Vergleich mit MATLAB]
float getAngle(float p1_x, float p1_y, float p2_x, float p2_y)
{
  float delta_x = p2_x - p1_x; //akt_x und akt_y konst. Größen definiert in aktorik.h
  float delta_y = p2_y - p1_y;
  float phi = 0;
  if (delta_x > 0)
  {
    phi = atanf(delta_y / delta_x);
  }
  else if (delta_x < 0)
  {
    phi = atanf(delta_y / delta_x) - pi;
  }
  else if (delta_x == 0 && delta_y > 0)
  {
    phi = pi / 2;
  }
  else if (delta_x == 0 && delta_y < 0)
  {
    phi = -pi / 2;
  }
  return phi;
}
///////////////////////////////////////////////////////////////////////

void setup()
{
  //Anfahren der Ausgangsposition des Aktors
  driveStart(start_pos);
  delay(5000);

  // map k to period
  setPeriod();

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

  getReadyToStart();
}

void getReadyToStart()
{
  Serial.print("Serial available, please type y to start:");
  int go = 0;
  while (go == 0)
  {
    // read the incoming byte:
    int incomingByte = Serial.read();

    if (incomingByte != -1)
    {
      // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingByte);
      if (incomingByte == 121)
      {
        go = 1;
      }
    }
    delay(50);
  }
}

void loop()
{

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
  //Ueberschreiben mit fixem ADC Wert für Demo
  //angleB = 2600;
  unsigned int angleK = analogRead(potiK);

  //Mappen der Sensordaten auf Winkelwerte:
  //TODO
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
  //TODO
  Serial.print("   Kraft Sensor B (vorne, Druck, Blau): ");
  Serial.print(forceB);
  Serial.print("   Kraft Sensor A (hinten, Zug, Gelb): ");
  Serial.println(forceA);

  //Serial.print("Loop Nummer ");
  //Serial.println(t);

  //Bestimmen des momentanen Schwingenwinkels
  //TODO: Lineare Kalibrierkurve fuer Poti B hinterlegen -> Vereinfachte Winkelberechung fuer Kraftregler
  float phi_B_d = 0.0533 * angleB - 9.27; //Umrechnung des ADC Werts über lineare Regression von für Positionssensorik an Gelenk B
  float phi_B = phi_B_d * (pi / 180);     //Umrechnung des Winkels in Radiant
  //Serial.print("Winkel B berechnet = ");
  //Serial.println(phi_B_d);
  /* float   phi_A_d = 0.0843*angleA - 206.2727; //Umrechnung des ADC Werts über lineare Regression von für Positionssensorik an Gelenk A
  float   phi_A = phi_A_d*(pi/180); //Umrechnung des Winkels in Radiant 
  //Serial.print("Winkel A berechnet = ");
  //Serial.println(phi_A_d);
  float   phi_K_d = 0.0721*angleK - 88.8166; //Umrechnung des ADC Werts über lineare Regression von für Positionssensorik an Gelenk K
  float   phi_K = phi_K_d*(pi/180); //Umrechnung des Winkels in Radiant 
  //Serial.print("Winkel K berechnet = ");
  //Serial.println(phi_K_d);
*/
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

  //DEMO KRAFTREGELUNG: Dummywert nutzen (Encoder)
  //M_akt = analogRead(sensorpin); //Kraftwert einlesen
  //Serial.print("Sensorwert F= ");
  //Serial.println(valSens);

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

  setPeriod();

  //Schreiben der erhobenen Daten auf die SD Karte als 32Byte String
  print2dataFile(ms, angleB, angleA, angleK, forceB, forceA);

  //Prüfen der Abbruchbedingung: Beenden der Messungen nach Anzahl totalRuns Loopdurchläufen
  if (t++ > totalRuns)
  {
    t = 0;
    num_runs += 1;

    if (num_runs == max_runs)
    {
      closeDataFile();
      Serial.println("Messungen abgeschlossen, Programm beendet");
      delay(5000);
      positionActuator(start_pos);
      while (1)
        ;
    }

  } //Durchgang

  //Prüfen, ob gesetzte Soll-Loopzeit verstrichen ist - wenn nicht, warten bis neuer Loop beginnen kann
  while (millis() < breakTime)
  {
    //waiting for loop time to finish - do nothing
  } //Loopdauer

} //Loop Funktion
