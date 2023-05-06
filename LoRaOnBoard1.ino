//ECS: Digital 9
//Knob CLK: Digital 10, DT: Digital 11
//Joystick VRX A0, VRY A1
//Servo 1: digital 6


#include <Servo.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#define VCC2  9 // define pin 5 or any other digial pin here as VCC2
#define GND2  12 // define pin 2 or any other digital pin as Ground 2


#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13
  
Servo servo1;
Servo servo2;
Servo ESC;
int joyX = 0;
int joyY = 1;

//set up the pulse width modulation driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

 // Rotary Encoder Inputs
 #define inputCLK 10
 #define inputDT 11
 
 
 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK; 
 
 String encdir ="";

int servoVal;
int servoVal2;

void setup() {


    pinMode(9,OUTPUT);//define a digital pin as output
  digitalWrite(9, HIGH);// set the above pin as HIGH so it acts as 5V
  
  pinMode(10,OUTPUT);//define a digital pin as output
  digitalWrite(10, LOW);// set the above pin as LOW so it acts as Ground

  pinMode(12,OUTPUT);//define a digital pin as output
  digitalWrite(12, LOW);// set the above pin as LOW so it acts as Ground
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
  pwm.setPWM(0, 0, 200);
  delay(4000);
    //------------------------------------------------------------------------LoRa STUFF------------------------------------------------------------------------
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  /*
  while (!Serial) {
    delay(1);
  }
  delay(100);
  */
  Serial1.begin(9600);
  Serial.println("Feather LoRa RX Test!");

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
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);


  //------------------------------------------------------------------------SERVO STUFF------------------------------------------------------------------------


  
  servo1.attach(6);
   // Set encoder pins as inputs  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);


}

int EscVal = 200;

void loop() {
  //radio stuff
   if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

     //get the text from the remote controller
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      //this characters in the received text are the elevator value
      char sv1[3]  = {(char)buf[0], (char)buf[1], (char)buf[2]};
      servoVal = atoi(sv1) -30;
      pwm.setPWM(3, 0, servoVal);
      /*
      servoVal = map(servoVal, 0, 1023, 127, 483);
      
      Serial.println("New ServoVal:.........");
      */
      Serial.print(servoVal);
      
      //this characters in the received text are the aeleron servo actuator value
      char sv2[3]  = {(char)buf[6], (char)buf[7], (char)buf[8]};
      servoVal2 = atoi(sv2) -30;
      pwm.setPWM(1, 0, servoVal2);
      pwm.setPWM(2, 0, servoVal2);      

      //this characters in the received text are the motor speed value
      char esc1[3]  = {(char)buf[3], (char)buf[4], (char)buf[5]};
      
      //convert the ESC (motor speed) value to integer
      EscVal = atoi(esc1);
      EscVal = map(EscVal, 0, 1023, 0, 1000);
      //ESC.write(1100);
      
      //print motor value in serial monitor
      Serial.println("Motor Value: ");
      Serial.println(EscVal);
      
      // default reply, should be replaced with GPS coordinates
      aint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);

    
      
      

      
    }

else
    {
      Serial.println("Receive failed");
      
    }
    
  }
  
      //set the motor speed
      pwm.setPWM(0, 0, EscVal);
      //delay(100);
}
