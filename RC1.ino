// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <Servo.h>
#include<string>

#define VCC2  9 // define pin 5 or any other digial pin here as VCC2
#define GND2  6 // define pin 2 or any other digital pin as Ground 2

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//Servo setup
Servo servo1;

//joystick setup
int joyX = 0;
int joyY = 1;
int servoVal;

int servoValY;

//joystick 2 setup
int joy2X = 3;
int joy2Y = 4;
int servoVal2;



// Rotary Encoder Inputs
#define inputCLK 10
#define inputDT 11

//Rotary encoder min value (1000 = stop motor)
int counter = 0;
//Rotary encoder corrent and previus values
int currentStateCLK;
int previousStateCLK; 
 
String encdir ="";

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//set up function sets up all the necesary drivers and functions for the hardware
void setup() 
{

   pinMode(VCC2,OUTPUT);//define a digital pin as output
  digitalWrite(VCC2, HIGH);// set the above pin as HIGH so it acts as 5V

  pinMode(GND2,OUTPUT);//define a digital pin as output
  digitalWrite(GND2, LOW);// set the above pin as LOW so it acts as Ground 
  
  pinMode(10,OUTPUT);//define a digital pin as output
  digitalWrite(10, HIGH);// set the above pin as HIGH so it acts as 5V

  pinMode(11,INPUT);//define a digital pin as output
  digitalWrite(11, LOW);// set the above pin as LOW so it acts as Ground 

   pinMode(12,OUTPUT);//define a digital pin as output
  digitalWrite(12, HIGH);// set the above pin as HIGH so it acts as 5V

  pinMode(13,OUTPUT);//define a digital pin as output
  digitalWrite(13, LOW);// set the above pin as LOW so it acts as Ground 


  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
 /*
  while (!Serial) {
    delay(1);
  }
*/
  delay(10);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  //Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

   // Set encoder pins as inputs  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);

  
}

int16_t packetnum = 1;  // packet counter, we increment per xmission


//This method takes char array and transmits it to the LoRa module on board
void sendPacket(char*  radiopacket)
{
  //delay(10); // Wait .01 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server
  
  itoa(packetnum++, radiopacket+13, 10);
  //Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending...");
  //delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete..."); 
  //delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  //listen for a response
  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }



}

/*
char intToChar(int n) {
   
   char[10] str = {0};
  count = 0;
  for (int a = 10; a < n; a *=10)
  {
    str[count] = n%a
  }

}
*/

int motorVal = 0;

//this continuous loop gets the values from the joystick, 
//converts all the controller values to text 
//and pushes to the plane board over the radio 
void loop()
{
  //tail (y axis actually)
  servoVal = analogRead(joyX);
  servoVal = map(servoVal, 0, 1023, 197, 383);
  //servo1.write(servoVal);
  //wings (x axis actually)
  servoVal2 = analogRead(joyY);
  servoVal2 = map(servoVal2, 0, 1023, 197, 413);
/*
  servoValY = analogRead(joyY);
  servoValY = map(servoValY, 0, 1023, 127, 483);
  //servo2.write(servoVal);
  //delay(15); 
*/
  //map the joystick position to propper values
  motorVal = analogRead(2);
  motorVal = map(motorVal, 0, 1023, 400, 195);
  
  
  //motor speed value holder
  counter = motorVal;
  Serial.println(counter);
 
  //convert the joystick X axis mapped value to text
  String joy =  String(servoVal+30);
  char tab[1024];
  strncpy(tab, joy.c_str(), sizeof(tab));
  tab[sizeof(tab) - 1] = 0;
  
   //Serial.print(tab);
 
  //convert the motor speed axis mapped value to text
  String rot = String(counter);
  char tab2[1024];
  strncpy(tab2, rot.c_str(), sizeof(tab2));
  tab2[sizeof(tab2) - 1] = 0;
  
  //concatinate new value to the string that will be sent via radio
  strcat(tab, tab2);

  //convert the joystick X axis mapped value to text (I KNOW IT SAYS X, IT SHOULD BE Y!!!)
  String joyx2 =  String(servoVal2+30);
  char tab3[1024];
  strncpy(tab3, joyx2.c_str(), sizeof(tab3));
  tab3[sizeof(tab3) - 1] = 0;
 
  //concatinate new value to the string that will be sent via radio
  strcat(tab, tab3);

  //print the text to be sent over radio and push it. 
  Serial.println(tab);
  sendPacket(tab);


}
