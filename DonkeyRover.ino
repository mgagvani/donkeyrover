//Code by Manav and Papa Gagvani
// Code for Mega 2560
// 06/27/2015

// Added encoder support 08/31/15

#include <NewPing.h>
#include <SoftwareSerial.h>

// Arduino pin #10 to TX Bluetooth module
// Arduino pin #11 to RX Bluetooth module

#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

SoftwareSerial mySerial(10,11);                           // BlueTooth module.  Mega can only receive on pins 10,11
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "xxxx";      // message to Android device
int joyX, joyY, lastY;                     // Joystick X,Y position

// Define ultrasonic library
#define TRIGGER_PIN 7
#define ECHO_PIN 30
#define TRIGGER_BACK_PIN 9
#define ECHO_BACK_PIN 31
#define MAX_DISTANCE 256
#define STOP_DISTANCE 25

int distanceInCm = MAX_DISTANCE;
int distanceInCmBack = MAX_DISTANCE;

// Initialize the Ultrasonic library
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarback(TRIGGER_BACK_PIN, ECHO_BACK_PIN, MAX_DISTANCE);



// Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.

// Motor FR
int fwFR = 24;
int rwFR = 25;
int speedFR = 2; // Needs to be a PWM pin to be able to control motor speed

// Motor FL
int fwFL = 23;
int rwFL = 22;
int speedFL = 3; // Needs to be a PWM pin to be able to control motor speed

// Motor BR
int fwBR = 27;
int rwBR = 26;
int speedBR = 4; // Needs to be a PWM pin to be able to control motor speed

// Motor BL
int fwBL = 28;
int rwBL = 29;
int speedBL = 5; // Needs to be a PWM pin to be able to control motor speed

// Optical encoder pulse counters
volatile int FLpulses = 0;
volatile int FRpulses = 0;
volatile int BLpulses = 0;
volatile int BRpulses = 0;

// Circumference of the wheel in meters
float wheelCircumference = 0.221;

// Global microseconds since last speed measurement
unsigned long timeMicros=0;

// Motor control mode - speed = 0, rotations = 1, distance = 2
byte motorMode = 0;

// Set the max PWM setting based on battery voltage.
float batteryVoltage = 7.4;
byte maxPWM = (int) (255 * 6.0)/batteryVoltage;


void setup() {  // Setup runs once per reset
  // initialize serial communication @ 9600 baud:
  Serial.begin(9600);
  mySerial.begin(9600);  // Bluetooth comms
  //Define L298N Dual H-Bridge Motor Controller Pins

  pinMode(fwFR,OUTPUT);
  pinMode(rwFR,OUTPUT);
  pinMode(speedFR,OUTPUT);
  pinMode(fwFL,OUTPUT);
  pinMode(rwFL,OUTPUT);
  pinMode(speedFL,OUTPUT);
  pinMode(fwBR,OUTPUT);
  pinMode(rwBR,OUTPUT);
  pinMode(speedBR,OUTPUT);
  pinMode(fwBL,OUTPUT);
  pinMode(rwBL,OUTPUT);
  pinMode(speedBL,OUTPUT);
  
  // Define sonar pins
  
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_BACK_PIN, OUTPUT);
  pinMode(ECHO_BACK_PIN, INPUT);
  
  // Set up interrupts for optical encoders
  /*
   Most Arduino boards have two external interrupts: numbers 0 (on digital pin 2) 
   and 1 (on digital pin 3). The Arduino Mega has an additional four: 
   numbers 2 (pin 21), 3 (pin 20), 4 (pin 19), and 5 (pin 18). 
   
   pin 2 was giving spurious readings, so changed to pin 18 for FR encoder
  */
  
  attachInterrupt(5, FRChange, CHANGE);  // pin 18
  attachInterrupt(2, FLChange, CHANGE);  // pin 21
  attachInterrupt(3, BRChange, CHANGE);  // pin 20
  attachInterrupt(4, BLChange, CHANGE);  // pin 19
  
  // Measure time
  
  timeMicros = micros();
  
  while(mySerial.available())  mySerial.read();         // empty RX buffer
  Serial.print("Max PWM = ");
  Serial.println(maxPWM);
  
}
  

void loop() {  
   int uS = sonar.ping();
  int currentDistance = uS/US_ROUNDTRIP_CM;
  distanceInCm =  (distanceInCm + currentDistance)/2;  
  int uSBack = sonarback.ping();
  int currentDistanceBack = uSBack/US_ROUNDTRIP_CM;
  distanceInCmBack =  (distanceInCmBack + currentDistanceBack)/2;
  // Serial.println(distanceInCm);        // Debug only
  // Serial.println(distanceInCmBack);  // Debug only
  delay(2);
  if(mySerial.available())  {                           // data received from tablet
    delay(2);
    cmd[0] =  mySerial.read();
    // Serial.println("Received data");  // Debug only
    if(cmd[0] == STX)  {
      int i=1;      
      while(mySerial.available())
      {
        delay(1);
        cmd[i] = mySerial.read();
        if(cmd[i]>127 || i>7)                 break;     // Communication error
        if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
        i++;        
      }
      if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  } 
  else moveMotors();
    
  // sendBlueToothData();
  
}

void sendBlueToothData()  {
  static long previousMillis = 0;                             
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
    previousMillis = currentMillis; 

// Data frame transmitted back from Arduino to Android device:
// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    mySerial.print((char)STX);                                             // Start of Transmission
    mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
    mySerial.print(GetdataInt1());            mySerial.print((char)0x4);   // datafield #1
    mySerial.print(GetdataFloat2());          mySerial.print((char)0x5);   // datafield #2
    mySerial.print(displayStatus);                                         // datafield #3
    mySerial.print((char)ETX);                                             // End of Transmission
  }  
}


String getButtonStatusString()  {
  String bStatus = "";
  for(int i=0; i<6; i++)  {
    if(buttonStatus & (B100000 >>i))      bStatus += "1";
    else                                  bStatus += "0";
  }
  return bStatus;
}

int GetdataInt1()  {              // Data dummy values sent to Android device for demo purpose
  static int i= -30;              // Replace with your own code
  i ++;
  if(i >0)    i = -30;
  return i;  
}

float GetdataFloat2()  {           // Data dummy values sent to Android device for demo purpose
  static float i=50;               // Replace with your own code
  i-=.5;
  if(i <-50)    i = 50;
  return i;  
}

void getJoystickState(byte data[8])    {
  joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
  joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  
  if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
  
// Your code here ...
  // Serial.print("Joystick position:  ");
  //  Serial.print(joyX);  
  //  Serial.print(", ");  
  //  Serial.println(joyY);
} 

void moveMotors() {
   switch (motorMode)
   {
     case 0: motorSpeedControl();
     break;
     case 1: motorRotationControl();
     break;
     case 2: motorDistanceControl();
     break;
     default: motorSpeedControl();
     break;
   }
}

void motorSpeedControl() {
    //  Make the motor faster the closer y is to 100, but stop if too close
    if (joyY > 10 && distanceInCm > STOP_DISTANCE) 
    {
       int motorSpeed = maxPWM*joyY/100;
       // Reduce speed if distance less than 2 times STOP_DISTANCE
       float multiplier = 1.0;
       if (distanceInCm < STOP_DISTANCE*2) 
       {
          multiplier = (distanceInCm - STOP_DISTANCE)/STOP_DISTANCE;
       }
       motorSpeed = motorSpeed * multiplier;
       // Change motor speed 
       if (motorSpeed > 0)       forward(motorSpeed);
       Serial.println("Going fwd");
    }
    else if (distanceInCm < STOP_DISTANCE && joyY > 70)  // spin back if too close
    {
       int motorSpeed = maxPWM*(STOP_DISTANCE-distanceInCm)/STOP_DISTANCE;
       if (motorSpeed > 0) reverse(motorSpeed);
       Serial.print("Distance is ");
       Serial.print(distanceInCm);
       Serial.println(" Backing up");
    }      
    else if (joyY < -10 && distanceInCmBack > STOP_DISTANCE)
    {
       int yVal = -1 * joyY;
       int motorSpeed = maxPWM*yVal/100;
       // Reduce speed if distance less than 2 times STOP_DISTANCE
       float multiplier = 1.0;
       if (distanceInCmBack < STOP_DISTANCE*2) 
       {
          multiplier = (distanceInCmBack - STOP_DISTANCE)/STOP_DISTANCE;
       }
       motorSpeed = motorSpeed * multiplier;
       // Change motor speed 
       if (motorSpeed > 0) reverse(motorSpeed);
       Serial.println("Going reverse");
    }
    else if (distanceInCmBack < STOP_DISTANCE && joyY < -70) // spin forward if too close
    {
       int motorSpeed = maxPWM*(STOP_DISTANCE-distanceInCmBack)/STOP_DISTANCE;
       if (motorSpeed > 0) forward(motorSpeed);
       Serial.println("Reverse - pushing fwd");
    }     
    else if (joyX > 20 && joyY < 30 && joyY > -30)
    {
        int motorSpeed = maxPWM * joyX/100;
        turnright(motorSpeed);
    }
    else if (joyX < -20 && joyY < 30 && joyY > -30) 
    {
        int xVal = -1 * joyX;
        int motorSpeed = maxPWM * xVal/100;
        turnleft(motorSpeed);
    }
    else
    {
        halt();
    }
}

void motorRotationControl()
{
  static int lastY;
  if (lastY == joyY)
    return;  // move only once till joyY changes
  lastY = joyY;
  float rotations = (float) joyY/20;
  if (joyY > 10)
  {
    moveRotations(1, rotations);
  }
  else if (joyY < -10)
  {
    moveRotations(3, -1*rotations);
  }
}

void motorDistanceControl()
{
  if (lastY == joyY)
    return;  // move only once till joyY changes
  lastY = joyY;
  float distance = (float) joyY/10; // joystick at 100 is 10 ft 
  float rotations = (float) distance/(3.2808399*wheelCircumference);
  if (distance == 0) return;
  Serial.print("Moving Distance = ");
  Serial.print(distance);
  Serial.print(" ft.  Moving Rotations = ");
  Serial.println(rotations);
  if (joyY > 5)
  {
    moveRotations(1, rotations);
  }
  else if (joyY < -5)
  {
    moveRotations(3, -1*rotations);
  }
  delay(20);
}

// Move a specified number of feet
void moveDistance(float distance)
{
  float rotations = (float) distance/(3.2808399*wheelCircumference);
  if (distance == 0) return;
  Serial.print("Moving Distance = ");
  Serial.print(distance);
  Serial.print(" ft.  Moving Rotations = ");
  Serial.println(rotations);
  if (distance > 0.1)
  {
    moveRotations(1, rotations);
  }
  else if (distance < -0.1)
  {
    moveRotations(3, -1*rotations);
  }
  delay(20);
  halt();
}


void getButtonState(int bStatus)  {
  switch (bStatus) {
// -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      Serial.println("\n** Rotation Mode **");
      motorMode = 1;   
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF
      Serial.println("\n** Speed Mode **");
      motorMode = 0;
      break;

// -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      Serial.println("\n** Distance Mode **");
      motorMode = 2;
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      Serial.println("\n** Speed Mode **");
      motorMode = 0;
      break;

// -----------------  BUTTON #3  -----------------------
    case 'E':
      buttonStatus |= B000100;        // ON
      Serial.println("\n** Button_3: ON **");
      // motor forward 6 feet      
      moveDistance(6.0);
      break;
    case 'F':
      buttonStatus &= B111011;      // OFF
      Serial.println("\n** Button_3: OFF **");
      // motor reverse 6 ft
      moveDistance(-6.0);  
      break;

// -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // ON
      Serial.println("\n** Button_4: ON **");
      // your code...      
      displayStatus = "Datafield update <FAST>";
      Serial.println(displayStatus);
      sendInterval = FAST;
      break;
    case 'H':
      buttonStatus &= B110111;    // OFF
      Serial.println("\n** Button_4: OFF **");
      // your code...      
      displayStatus = "Datafield update <SLOW>";
      Serial.println(displayStatus);
      sendInterval = SLOW;
     break;

// -----------------  BUTTON #5  -----------------------
    case 'I':           // configured as momentary button
//      buttonStatus |= B010000;        // ON
      Serial.println("\n** Button_5: ++ pushed ++ **");
      // your code...      
      displayStatus = "Button5: <pushed>";
      break;
//   case 'J':
//     buttonStatus &= B101111;        // OFF
//     // your code...      
//     break;

// -----------------  BUTTON #6  -----------------------
    case 'K':
      buttonStatus |= B100000;        // ON
      Serial.println("\n** Button_6: ON **");
      // your code...      
       displayStatus = "Button6 <ON>"; // Demo text message
     break;
    case 'L':
      buttonStatus &= B011111;        // OFF
      Serial.println("\n** Button_6: OFF **");
      // your code...      
      displayStatus = "Button6 <OFF>";
      break;
  }
// ---------------------------------------------------------------
}

void forward(int speed) {
  // Move all motors forward at the specified speed
  
  moveFL(1, speed);
  moveFR(1, speed);
  moveBL(1, speed);
  moveBR(1, speed);
}

void reverse(int speed) {
  // Move all motors forward at the specified speed
  
  moveFL(3, speed);
  moveFR(3, speed);
  moveBL(3, speed);
  moveBR(3, speed);
}

void halt() {
  // Stop all motors
  int speed = 0;
  
  moveFL(2, speed);
  moveFR(2, speed);
  moveBL(2, speed);
  moveBR(2, speed);
}

void turnleft(int speed) {
  // Move all motors forward at the specified speed
  
  moveFL(3, speed);
  moveFR(1, speed);
  moveBL(3, speed);
  moveBR(1, speed);
}

void turnright(int speed) {
  // Move all motors forward at the specified speed
  
  moveFL(1, speed);
  moveFR(3, speed);
  moveBL(1, speed);
  moveBR(3, speed);
}

void moveRotations(int dir, float rotations) {
  // Move forward a specific number of rotations
  // dir is 1 for forward, 3 for reverse
  
  /*
  Use three different power settings depending on the number of remaining rotations.
  Use 30%, 70% and 100%
  Measure every 200 ms.  
  */
  float low = 0.3;
  float medium =0.7;
  float high = 1.0;
  
  //  How often should we measure at various speeds
  
  int highDelay = 200;
  int mediumDelay = 80;
  int lowDelay = 50;
  
  float remainingRotations = (float) rotations;
  float FRRotations = 0;
  float FLRotations = 0;
  float BRRotations = 0;
  float BLRotations = 0;
  
  if (!(dir == 1 || dir == 3)) return;
  
  while (remainingRotations > 2)
  {
    int spd = (int) (high * maxPWM);
    if (dir == 1)  forward (spd);
    else if (dir == 3) reverse(spd);
    delay(highDelay);
    measureRotations(FRRotations, FLRotations, BRRotations, BLRotations);
    float avgRotations = (FRRotations + FLRotations + BRRotations + BLRotations)/4.0;
    remainingRotations = remainingRotations - avgRotations;
    Serial.print("Remaining rotations = ");
    Serial.println(remainingRotations);
  }
  while (remainingRotations > 1)
  {
    int spd = (int) (medium * maxPWM);
    if (dir == 1)  forward (spd);
    else if (dir == 3) reverse(spd);
    delay(mediumDelay);
    measureRotations(FRRotations, FLRotations, BRRotations, BLRotations);
    float avgRotations = (FRRotations + FLRotations + BRRotations + BLRotations)/4.0;
    remainingRotations = remainingRotations - avgRotations;
    Serial.print("Remaining rotations = ");
    Serial.println(remainingRotations);
  }
  while (remainingRotations > 0.15)
  {
    int spd = (int) (low * maxPWM);
    if (dir == 1)  forward (spd);
    else if (dir == 3) reverse(spd);
    delay(lowDelay);
    measureRotations(FRRotations, FLRotations, BRRotations, BLRotations);
    float avgRotations = (FRRotations + FLRotations + BRRotations + BLRotations)/4.0;
    remainingRotations = remainingRotations - avgRotations;
    Serial.print("Remaining rotations = ");
    Serial.println(remainingRotations);
  }
  halt();
}
  
void moveFL(int mode, int speed) {
  switch (mode) {
    case 1:
      analogWrite(speedFL, speed);//Sets speed variable via PWM 
      digitalWrite(rwFL, LOW);
      digitalWrite(fwFL, HIGH);
      // Serial.println("Motor FL Forward"); 
      // Serial.println("   "); 
      break;
    case 2: // Motor FL Stop (Freespin)
      analogWrite(speedFL, 0);
      digitalWrite(rwFL, LOW);
      digitalWrite(fwFL, HIGH);
      // Serial.println("Motor FL Stop");
      // Serial.println("   ");
      break;
    case 3: // Motor FL Reverse
      analogWrite(speedFL, speed);
      digitalWrite(rwFL, HIGH);
      digitalWrite(fwFL, LOW);
      // Serial.println("Motor FL Reverse");
      // Serial.println("   ");
      break;
  }
}

void moveFR(int mode, int speed) {
  switch (mode) {
    case 1:
      analogWrite(speedFR, speed);//Sets speed variable via PWM 
      digitalWrite(rwFR, LOW);
      digitalWrite(fwFR, HIGH);
      // Serial.println("Motor FR Forward"); 
      // Serial.println("   "); 
      break;
    case 2: // Motor FR Stop (Freespin)
      analogWrite(speedFR, 0);
      digitalWrite(rwFR, LOW);
      digitalWrite(fwFR, HIGH);
      // Serial.println("Motor FR Stop");
      // Serial.println("   ");
      break;
    case 3: // Motor FR Reverse
      analogWrite(speedFR, speed);
      digitalWrite(rwFR, HIGH);
      digitalWrite(fwFR, LOW);
      // Serial.println("Motor FR Reverse");
      // Serial.println("   ");
      break;
  }
}

void moveBL(int mode, int speed) {
  switch (mode) {
    case 1:
      analogWrite(speedBL, speed);//Sets speed variable via PWM 
      digitalWrite(rwBL, LOW);
      digitalWrite(fwBL, HIGH);
      // Serial.println("Motor BL Forward"); 
      // Serial.println("   "); 
      break;
    case 2: // Motor BL Stop (BLeespin)
      analogWrite(speedBL, 0);
      digitalWrite(rwBL, LOW);
      digitalWrite(fwBL, HIGH);
      // Serial.println("Motor BL Stop");
      // Serial.println("   ");
      break;
    case 3: // Motor BL Reverse
      analogWrite(speedBL, speed);
      digitalWrite(rwBL, HIGH);
      digitalWrite(fwBL, LOW);
      // Serial.println("Motor BL Reverse");
      // Serial.println("   ");
      break;
  }
}

void moveBR(int mode, int speed) {
  switch (mode) {
    case 1:
      analogWrite(speedBR, speed);//Sets speed variable via PWM 
      digitalWrite(rwBR, LOW);
      digitalWrite(fwBR, HIGH);
      // Serial.println("Motor BR Forward"); 
      // Serial.println("   "); 
      break;
    case 2: // Motor BR Stop (freespin)
      analogWrite(speedBR, 0);
      digitalWrite(rwBR, LOW);
      digitalWrite(fwBR, HIGH);
      // Serial.println("Motor BR Stop");
      // Serial.println("   ");
      break;
    case 3: // Motor BR Reverse
      analogWrite(speedBR, speed);
      digitalWrite(rwBR, HIGH);
      digitalWrite(fwBR, LOW);
      // Serial.println("Motor BR Reverse");
      // Serial.println("   ");
      break;
  }
}

void FRChange()
{
  FRpulses = FRpulses + 1;
}

void FLChange()
{
  FLpulses = FLpulses + 1;
}

void BRChange()
{
  BRpulses = BRpulses + 1;
}

void BLChange()
{
  BLpulses = BLpulses + 1;
}

void measureRotations(float &numFRRotations, float &numFLRotations, float &numBRRotations, float &numBLRotations)
{
  unsigned long int nowTime, elapsedTime;
  
  nowTime = micros();
  if (nowTime < timeMicros) // if you run for 70 minutes and the micros counter overflows and resets
  {
    FRpulses = FLpulses = BRpulses = BLpulses = 0;
    timeMicros = micros();
    return;
  }
  elapsedTime = nowTime - timeMicros;
  if (elapsedTime < 100000) return;
  timeMicros = nowTime;
  
  numFRRotations = (float) FRpulses/40.0;
  numFLRotations = (float) FLpulses/40.0;
  numBRRotations = (float) BRpulses/40.0;
  numBLRotations = (float) BLpulses/40.0;
  
  /* 
  
  Serial.print("microsecs: ");
  Serial.println(elapsedTime);
  Serial.print("FR rotationss = ");
  Serial.print(numFRRotations);
  Serial.print(" || FL rotations = ");
  Serial.println(numFLRotations);
  Serial.print("BR rotations = ");
  Serial.print(numBRRotations);
  Serial.print(" || BL rotations = ");
  Serial.println(numBLRotations);
 */
  
  FRpulses = FLpulses = BRpulses = BLpulses = 0;
}
  

void test() {
  
// Initialize the Serial interface:

if (Serial.available() > 0) {
int inByte = Serial.read();
int speed; // Local variable

switch (inByte) {

case '1': // Motor FL Forward
analogWrite(speedFL, 136);//Sets speed variable via PWM 
digitalWrite(rwFL, LOW);
digitalWrite(fwFL, HIGH);
Serial.println("Motor FL Forward"); 
Serial.println("   "); 
break;

case '2': // Motor FL Stop (freespin)
analogWrite(speedFL, 0);
digitalWrite(rwFL, LOW);
digitalWrite(fwFL, HIGH);
Serial.println("Motor FL Stop");
Serial.println("   ");
break;

case '3': // Motor FL Reverse
analogWrite(speedFL, 136);
digitalWrite(rwFL, HIGH);
digitalWrite(fwFL, LOW);
Serial.println("Motor FL Reverse");
Serial.println("   ");
break;

case '4': // Motor FR Forward
analogWrite(speedFR, 136);//Sets speed variable via PWM 
digitalWrite(rwFR, LOW);
digitalWrite(fwFR, HIGH);
Serial.println("Motor FR Forward"); 
Serial.println("   "); 
break;

case '5': // Motor FR Stop (Freespin)
analogWrite(speedFR, 0);
digitalWrite(rwFR, LOW);
digitalWrite(fwFR, HIGH);
Serial.println("Motor FR Stop");
Serial.println("   ");
break;

case '6': // Motor FR Reverse
analogWrite(speedFR, 136);
digitalWrite(rwFR, HIGH);
digitalWrite(fwFR, LOW);
Serial.println("Motor FR Reverse");
Serial.println("   ");
break;

case '7': // Motor BL Forward
analogWrite(speedBL, 136);//Sets speed variable via PWM 
digitalWrite(rwBL, LOW);
digitalWrite(fwBL, HIGH);
Serial.println("Motor BL Forward"); 
Serial.println("   "); 
break;

case '8': // Motor BL Stop (BLeespin)
analogWrite(speedBL, 0);
digitalWrite(rwBL, LOW);
digitalWrite(fwBL, HIGH);
Serial.println("Motor BL Stop");
Serial.println("   ");
break;

case '9': // Motor BL Reverse
analogWrite(speedBL, 136);
digitalWrite(rwBL, HIGH);
digitalWrite(fwBL, LOW);
Serial.println("Motor BL Reverse");
Serial.println("   ");
break;

case 'a': // Motor BR Forward
analogWrite(speedBR, 136);//Sets speed variable via PWM 
digitalWrite(rwBR, LOW);
digitalWrite(fwBR, HIGH);
Serial.println("Motor BR Forward"); 
Serial.println("   "); 
break;

case 'b': // Motor BR Stop (BReespin)
analogWrite(speedBR, 0);
digitalWrite(rwBR, LOW);
digitalWrite(fwBR, HIGH);
Serial.println("Motor BR Stop");
Serial.println("   ");
break;

case 'c': // Motor BR Reverse
analogWrite(speedBR, 136);
digitalWrite(rwBR, HIGH);
digitalWrite(fwBR, LOW);
Serial.println("Motor BR Reverse");
Serial.println("   ");
break;


default:
// turn all the connections off if an unmapped key is pressed:
for (int thisPin = 2; thisPin < 6; thisPin++) {
digitalWrite(thisPin, LOW);
}
  }
    }
      }
