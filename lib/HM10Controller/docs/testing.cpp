
/*
 Hardware: DSD TECH HM-13 Bluetooth Dual-Mode-Module and Teensy 3.6
 HM-13:
 http://www.dsdtech-global.com/2018/06/dsd-tech-hm-12-hm-13.html
 Datasheet and AT Commands:
 https://drive.google.com/file/d/1QqB3Aonx9Dx4eIzqxxeJjIxL_ox9Ub5V/view

Pins
  Teensy 3.6  - Bluetooth HM-13 BLE module
  18650 Li-Ion 3.7V   VCC pin
  GND pin             GND pin
  Digital RX 34       RX pin Teensy 3.6 = 3.3V (no need voltage divider)
  Digital TX 33       TX pin Teensy 3.6 = 3.3V (no need voltage divider)
 
 Software: 
 Copied from: http://acoptex.com/project/302/basics-project-028d-bluetooth-40-hm-10-ble-module-at-acoptexcom/
 but Serial 5 used as Serial Port for BT communication and BT-boudrate updated to 115200
*/

/*
  Project: Bluetooth HM-10 BLE module AT comms using software serial library
  Function: Uses hardware serial to talk to the host computer and software serial
          for communication with the Bluetooth module. Intended for Bluetooth
          devices that require line end characters "\r\n". When a command is entered
          in the serial monitor on the computer the Arduino will relay it to the
          bluetooth module and display the result.
  Pins
  Arduino  - Bluetooth HM-10 BLE module
  3.3V (5V) pin     VCC pin
  GND pin    GND pin
  Digital 3  RX pin through a voltage divider
  Digital 2  TX pin (no need voltage divider)
*/

//*********************************************************************************
#include <Arduino.h>
#include <SoftwareSerial.h> //include library code
//*********************************************************************************
//Teensy 3.6
//#define BTserial Serial1 //1,0
//https://www.pjrc.com/teensy/td_uart.html
//Teensy 3.6

//Arduino
//SoftwareSerial BTserial(2, 3); // RX | TX
// Connect the HM-10 BLE TX to the Arduino RX on pin 2.
// Connect the HM-10 BLE RX to the Arduino TX on pin 3 through a voltage divider.
//Arduino
//*********************************************************************************
SoftwareSerial BTserial(0, 1);

char c = ' ';
boolean NL = true;
void setup()
{
    Serial.begin(9600);     //Baudrate for Serial Monitor: initialize serial communication at 9600 bps
    BTserial.begin(115200); //Baudrate for BT-Module: It can also be configured to have different baud rates:2=9600,3=19200,4=38400,5=57600,6=115200 etc.
                            //By default the baud rate is 115200. If 115200 doesn't work try different baud rate
}
void loop()
{
    // Read from the Bluetooth module and send to the Arduino Serial Monitor
    if (BTserial.available())
    {
        c = BTserial.read();
        Serial.write(c);
    }

    // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available())
    {
        c = Serial.read();

        // do not send line end characters to the HM-10
        if (c != 10 & c != 13)
        {
            BTserial.write(c);
        }

        // Echo the user input to the main window.
        // If there is a new line print the ">" character.
        if (NL)
        {
            Serial.print("\r\n>");
            NL = false;
        }
        Serial.write(c);
        if (c == 10)
        {
            NL = true;
        }
    }
}