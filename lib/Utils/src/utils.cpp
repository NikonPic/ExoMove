#include "utils.h"
#include <math.h>

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
    unsigned int DataMapped = rawData;
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

// setzte die periode
int setPeriod(int k)
{
    int per;
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
    return per;
}

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
