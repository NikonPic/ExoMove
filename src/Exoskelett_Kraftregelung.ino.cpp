# 1 "C:\\Users\\Niko\\AppData\\Local\\Temp\\tmpl5ef1ef6"
#include <Arduino.h>
# 1 "C:/Users/Niko/Documents/GitHub/ExoMove/src/Exoskelett_Kraftregelung.ino"
#include <SD.h>
#include <math.h>







File dataFile;
void initCard();
bool openFile_write(const char *filename);
float getActuationSignal(float phi);
float getActuationTorque(float phi_B, float F_Aktor);
float getAngle(float p1_x, float p1_y, float p2_x, float p2_y);
float getNomPosition(int k);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float mapActuatorInput(float nomPosition);
void positionActuator(int k);
unsigned int readAngleRadB(int potipin);
void posManual(int inputpin, int aktorpin);
void setup();
void loop();
#line 13 "C:/Users/Niko/Documents/GitHub/ExoMove/src/Exoskelett_Kraftregelung.ino"
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


bool openFile_write(const char *filename)
{
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.flush();
  return (dataFile == true);
}




unsigned int loopTime = 10;
unsigned long breakTime;


int totalRuns = 22000 / loopTime;
int t = 0;
int k = 0;
int num_runs = 0;



const float l_akt = 139.3f;
const float d_b = 161.062f;
const float akt_x = -152.12f;
const float akt_y = -21.79f;
const float alpha_const = 0.0612f;
const float theta = 0.6109f;
const float l_s = 20.0f;

const float phi_B_max = 2.2340f;

const float phi_B_min = 0.9235f;
const float delta_phi_max = 108.000f;
const float val_count_phi_B_max = 5.0f;
const float pi = 3.1416f;
const float multip = (2 * pi) / totalRuns;




float getActuationSignal(float phi)
{
  float alpha = pi - theta - phi + alpha_const;


  float l = sqrtf((powf(l_s, 2) + powf(d_b, 2) - 2 * l_s * d_b * cosf(alpha)));


  float s_nom = l - l_akt;
  return s_nom;
}


float getActuationTorque(float phi_B, float F_Aktor)
{
  float alpha_quer = pi - theta - phi_B;


  float S_x = -l_s * cosf(alpha_quer);
  float S_y = l_s * sinf(alpha_quer);
  float phi_F = getAngle(akt_x, akt_y, S_x, S_y);


  float F_Akt_x = F_Aktor * cosf(phi_F);
  float F_Akt_y = F_Aktor * sinf(phi_F);
  float M_akt = (F_Akt_x * S_y + F_Akt_y * (-S_x));
  return fabs(M_akt);
}


float getAngle(float p1_x, float p1_y, float p2_x, float p2_y)
{
  float delta_x = p2_x - p1_x;
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





const int aktorpin = 29;


float getNomPosition(int k)
{

  float sinsig = k * multip;


  float phi_raw = sinf(sinsig + 0.5 * pi);


  float phi_d = (phi_raw * delta_phi_max / 2);


  phi_d = phi_d + delta_phi_max / 2 + (phi_B_min / pi) * 180;


  float phi_r = (phi_d / 180) * pi;



  float nomPosition = getActuationSignal(phi_r);
  return nomPosition;
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  float val_mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return val_mapped;
}


float mapActuatorInput(float nomPosition)
{

  float nomPositionMapped = mapf(nomPosition, 0, 34.01, 0, 33);
  return nomPositionMapped;
}


void positionActuator(int k)
{

  float nomPos = getNomPosition(k);

  float nomPosMapped = mapActuatorInput(nomPos);

  int nomPosInt = nomPosMapped * 1000;
  int outputsignal = map(nomPosInt, 0, 33000, 0, 256);


  analogWrite(aktorpin, outputsignal);
}





unsigned int readAngleRadB(int potipin)
{
  unsigned int rawData = analogRead(potipin);
  unsigned int DataMapped = rawData;

  return DataMapped;
}
# 205 "C:/Users/Niko/Documents/GitHub/ExoMove/src/Exoskelett_Kraftregelung.ino"
void posManual(int inputpin, int aktorpin)
{
  int intputsignal = analogRead(inputpin);
  int outputsignal = map(intputsignal, 0, 1023, 0, 333);
  Serial.println(outputsignal);
  analogWrite(aktorpin, outputsignal);
}






const int potiA = A2;
const int potiB = A1;
const int potiK = A0;


const int fSenB = A10;
const int fSenA = A12;



unsigned long setupTime;


int dir = 1;
int per = 1;
int valSens;

float M_upper = 0.05;
float M_lower = 0.01;

float F_upper = 5;
float F_lower = 0.4;


void setup()
{

  analogWriteFrequency(aktorpin, 1000);

  Serial.begin(9600);

  delay(2000);

  pinMode(aktorpin, OUTPUT);


  analogReadResolution(12);
  analogReadAveraging(32);


  initCard();
  openFile_write("log_1.txt");


  positionActuator(0);

  setupTime = millis();
}

void loop()
{


  positionActuator(k);


  unsigned long ms = millis() - setupTime;


  breakTime = millis() + loopTime;



  unsigned int angleA = analogRead(potiA);
  unsigned int angleB = analogRead(potiB);


  unsigned int angleK = analogRead(potiK);



  Serial.print("   Winkel B: ");
  Serial.print(angleB);
  Serial.print("   Winkel A: ");
  Serial.print(angleA);
  Serial.print("   Winkel K: ");
  Serial.println(angleK);


  unsigned int forceB = analogRead(fSenB);
  unsigned int forceA = analogRead(fSenA);



  Serial.print("   Kraft Sensor B (vorne, Druck, Blau): ");
  Serial.print(forceB);
  Serial.print("   Kraft Sensor A (hinten, Zug, Gelb): ");
  Serial.println(forceA);






  float phi_B_d = 0.0533 * angleB - 9.27;
  float phi_B = phi_B_d * (pi / 180);
# 326 "C:/Users/Niko/Documents/GitHub/ExoMove/src/Exoskelett_Kraftregelung.ino"
  float F_back = 0.00477458 * forceA - 6.32508;
  float F_front = 0.0049497 * forceB - 7.17406;


  float F_akt = F_front - F_back;
  float M_akt = getActuationTorque(phi_B, F_akt) / 1000;

  float F_reg = abs(F_akt);
  Serial.print("F_reg = ");
  Serial.println(F_reg);




  Serial.print("M_akt = ");
  Serial.println(M_akt);
# 350 "C:/Users/Niko/Documents/GitHub/ExoMove/src/Exoskelett_Kraftregelung.ino"
  switch (dir)
  {
  case 1:

    if (F_reg < F_upper)
    {


      k++;
    }
    else
    {

      dir = 0;
      Serial.println("oberer Grenzwert überschritten, Wechsel in Rückwärtsbewegung");
    }
    break;
  case 0:

    if (F_reg > F_lower)
    {







      switch (per)
      {
      case 1:
        if (k > 0)
        {
          k--;
        }
        else
        {

        }
        break;
      case 2:
        if (k > totalRuns / 2)
        {
          k--;
        }
        else
        {

        }
        break;
      }
    }

    else
    {

      dir = 1;
      Serial.println("unterer Grenzwert unterschritten, wechsel in Vorwärtsbewegung");
    }
    break;
  }




  if (k < totalRuns / 2)
  {

    per = 1;

  }
  if (k > totalRuns / 2)
  {

    per = 2;

  }


  dataFile.printf("%5u;%4u;%4u;%4u;%4u;%4u\r\n", ms, angleB, angleA, angleK, forceB, forceA);


  if (t++ > totalRuns)
  {
    t = 0;
    num_runs += 1;

    if (num_runs == 2) {
      dataFile.close();
      Serial.println("Messungen abgeschlossen, Programm beendet");
      delay(5000);
      positionActuator(0);
      while (1);
    }


  }


  while (millis() < breakTime)
  {

  }

}