/*
 * Test connection from Unity to Arduino
 *
 * Author: Rik Teerling (2014)
 * Adapted by: Gidi van Liempd (gidi@geedesign.com)
 *
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

#define GIVE_INPUT     'a'  // the byte signalling that the computer wants input
#define ISSUE_COMMAND  'b'  // the byte signalling that the computer issues a command
#define SET_LED_ON     'x'
#define SET_LED_OFF    'y'

// Pin 13: Arduino and Teensy 3.* has a LED connected on pin 13
// Pin 11: Teensy 2.0 has the LED on pin 11
// Pin 6: Teensy++ 2.0 has the LED on pin 6
#define LED_PIN  13

void setup()
{
  Serial.begin(115200);

  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
  
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  //status = bmp.begin();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop()
{
  
  if(Serial.available()) {
    char inByte = Serial.read();

    if (inByte == GIVE_INPUT) {
        // pass input values to computer
        passValuesToComputer();
    }
    else if (inByte == ISSUE_COMMAND) {
        // parse commands from computer
        parseCommands();
    }  // else, ignore this byte
  }
}

void passValuesToComputer() {
    // construct string from read values
    String sOutput = "";
    sensors_event_t pressure_event;
    bmp_pressure->getEvent(&pressure_event);
    // add value analog A0  
    int adc0 = pressure_event.pressure;
    sOutput = sOutput + adc0;
                
    Serial.println(sOutput);
}

// parse commands received from the computer
void parseCommands() {
  // read next byte as command
  delay(1);  // a small delay seems necessary to get Serial.available working again?
  if(Serial.available()) {
  char inByte = Serial.read();
        if (inByte == SET_LED_ON) {
           digitalWrite(LED_PIN, HIGH);   // set the LED on
  }
        else if (inByte == SET_LED_OFF) {
           digitalWrite(LED_PIN, LOW);   // set the LED off
        }  // else, ignore this byte
  }
}
