
/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Keyboard.h>
#include "Keyboard.h"

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

int TheValue = 1000;
int Value1 = 2000;
int Value2 = 2000;
int Value3 = 2000;
int oldValue = 2000;
int calcValue = 3000;

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // initialize control over the keyboard:
  Keyboard.begin();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  int calcValue = Value1 + Value2 + Value3;
  int oldValue = calcValue / 3;
  if(oldValue < TheValue){
    //rising
    Keyboard.press('f');
    Serial.print("Rising");
    Serial.print("--");
    Serial.print(oldValue);
    Serial.print("-");
    Serial.print(TheValue);
    
  }
  else
  {
    Keyboard.releaseAll();
    Serial.print("Faling");
    Serial.print("--");
    Serial.print(oldValue);
    Serial.print("-");
    Serial.print(TheValue);
    Serial.print("-");
    Serial.print(Value1);
    Serial.print("-");
    Serial.print(Value2);
    Serial.print("-");
    Serial.print(Value3);
  }
  Value1 = TheValue;
  
  
  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  TheValue = pressure_event.pressure;
  Value2 = Value1;
  Value3 = Value2;
  Serial.println("");
  delay(500);
}
