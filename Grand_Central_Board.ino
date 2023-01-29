// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
#include <math.h>
#include <MultiSerial.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/dtostrf.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>


// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define RFMSerial Serial2
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


Servo ESC;

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  //Serial2.begin(9600);
  RFMSerial.begin(9600);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

   // Attach the ESC on pin 9
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.writeMicroseconds(900); // send “stop” signal to ESC.
  delay(4000); // delay to allow the ESC to recognize the stopped signal

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); 

}

void loop() // run over and over again
{

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false                 //raw data
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 4); Serial.println();
    Serial.print("Angle: "); Serial.println(GPS.angle);
  

        char buff[7];
        Serial2.write("Lat: ");
        Serial2.write(dtostrf(GPS.latitudeDegrees, 4, 8, buff));   //WARNING! Cross reffereance with GPS modules raw data to make sure parsing is accurate!
       
  }

    ESC.write(0);    // Send the signal to the ESC

    pwm.setPWM(0, 0, 127 + (int)(GPS.angle));
  
}