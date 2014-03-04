//******************************sensorDemo***********************************
/* This is the program for the roadkill of Team Brover's ENGR3392 project,
a miniature "Mars Rover." It is based on the Arbotix-M controller for Robot-
side and a base-side program using serial commands; we used LabView.
The communication between the controller and the PC is done via XBEE devices.
They are set to 38400 baud, 8, N, 1, N and have a preset protocol.

The PC initiates all communication and expects a return confirming operation.

There is no ESTOP Button right now. That's probably bad. 
TODO: ESTOP, Serial Send, Serial Receive, Loop Timing, Color Sensor Lighting. */
//***************************************************************************
/* Physical Pinout of the Arbotix-M Board
   Digital: 0: OUT LED
            1: OUT Color Sensor LED
            2: IN  Encoder 1
            3: IN  Encoder 2
            4: IN  Bumper Switch 1
            5: IN  Bumper Switch 2
            ...
            12:    Motor Speed 1
            13:    Motor Speed 2
            14:    Servo 1
            15:    Servo 2
   Analog:  0: IN  Lidar 1
            1: IN  Lidar 2
            2: IN  IR
            3: IN  [Battery Voltage?]
            4: IN  Encoder 1 [Sonar?]
            5: IN  Color (Blue)
            6: IN  Color (Green)
            7: IN  Color (Red)
*/
#include <Servo.h> // Package to control servos and motors
// Global Variables:
unsigned long loopStart; // time at start of loop()
unsigned long loopDuration = 100; // time for loop() in ms
int pcStatus = 1; // 1 = OK; 100 == ESTOP
const int bufferSize = 40; // Length of string for serial communication
char readbuffer[bufferSize]; // Buffer for serial communication

// Inputs (Sensors)
const int numDig = 2; // Number of Digital Inputs
const int numAnal = 0; // Number of Analog Inputs
const int digSensors[numDig] = {2,3}; // Physical Pins of the digital sensors on the Arbotix
const int analSensors[numAnal] = {}; //{0,1,2,4,5,6,7}; // Physical Pins of the analog sensors on the Arbotix
int digRes[numDig] = {}; // Storage for the raw digital sensor readings
int analRaw[numAnal] = {}; // Storage for the raw analog sensor readings
float analVolts[numAnal] = {}; // Storage for the voltage values of the analog sensors

// Outputs (Actuators)
Servo rightMotor, leftMotor, servoOne, servoTwo; // Initialize servo objects for motors, servos
// Motor Pin Numbers and storage for Motor Positions (0 - 2500)
const int rm_pin = 12; int rm_pos = 1500; // Right Motor
const int lm_pin = 13; int lm_pos = 1500; // Left Motor
const int so_pin = 14; int so_pos = 1500; // Servo One
const int st_pin = 15; int st_pos = 1500; // Servo Two
// Outputs (Other)
const int heartbeatPin = 0; // User LED on Arbotix-M board
boolean ledOn = false; // Stores Heartbeat User LED status
const int colorPin = 1; // Illumination LED on Color Sensor

//*************************************************
void setup() { // Runs once as soon as the Arbotix-M is powered
  Serial.begin(38400); // Begins serial communication via XBEE at 38400 bps
  delay(1000); // Wait for connection to establish
  pinMode(heartbeatPin, OUTPUT); // Initialize heartbeat LED pin
  pinMode(colorPin, OUTPUT); // Initialize on-chip color sensor LED pin
  digitalWrite(colorPin,HIGH); // Turns on the color sensor LED for the whole routine
  for (int i=0; i<numDig; i++){
    pinMode(digSensors[i],INPUT); // initialize each of the digital inputs
  }
  // Connect Servo objects with physical pin locations and make contact
  rightMotor.attach(rm_pin);
  rightMotor.writeMicroseconds(rm_pos);
  leftMotor.attach(lm_pin);
  leftMotor.writeMicroseconds(lm_pos);
  servoOne.attach(so_pin);
  servoOne.writeMicroseconds(so_pos);
  servoTwo.attach(st_pin);
  servoTwo.writeMicroseconds(st_pos);
}

//*************************************************
void loop() { // Runs continuously after setup() concludes
  loopStart = millis(); // marks the starting time of the loop
  readSensors(); // records values from all the sensors
  int numread = 0; // stores the number of bytes in the received string
  numread = readCommandString(); // reads the XBEE serial stream
  // Should send return only happen when a command is received?
  sendReturnString(numread);
  parseCommandString();
  // AUTONOMOUS BEHAVIORS WOULD GO HERE
  moveMotors();
  moveServos();
  while (millis() < (loopStart + loopDuration)){
    // loop timing hasn't really been implemented
  }
  beatHeart();
}

//*************************************************
void readSensors(){ // Read each of the sensors; store raw/volt data
  for (int i=0; i<numDig; i++){
    digRes[i] = digitalRead(digSensors[i]); // read each of the digital inputs
  }
  for (int i=0; i<numAnal; i++){
    analRaw[i] = analogRead(analSensors[i]); // read each of the analog inputs
    analVolts[i] = (float)analRaw[i] * 5.0 / 1024.0; // convert to volts
  }
}

int readCommandString(){ // returns number of bytes
  // Reads XBEE Serial stream and extracts command string
  int num = 0; // number of bytes read
  if (Serial.available() > 0){ // Reads buffer until end character '\n'
    num = Serial.readBytesUntil('\n',readbuffer, bufferSize);
  }
  return num;
}

void sendReturnString(int numread){ // Doesn't currently buffer anything.
  Serial.print("!");
  for (int i=0; i<numDig; i++){ // How do print?
    Serial.print("D");
    Serial.print(digSensors[i]);
    Serial.print(",");
    Serial.print(digRes[i]);
    Serial.print(",");
  }
  for (int i=0; i<numAnal; i++){ // How do print?
    Serial.print("A");
    Serial.print(analSensors[i]);
    Serial.print(",");
    Serial.print(analVolts[i]);
    Serial.print(",");
  }
}

void parseCommandString(){
  //the following converts the 4 char number values into integers
  //the readbuffer is assumed to be $CS----P----R----L----C----H----'\n' 34 bytes
  char dummy[4];    
  
  //for right motor
  dummy[0] = readbuffer[3];
  dummy[1] = readbuffer[4];
  dummy[2] = readbuffer[5];
  dummy[3] = readbuffer[6];
  
  rm_pos = atoi(dummy);    //convert char/byte array into interger
  
  //for left motor    
  dummy[0] = readbuffer[8];
  dummy[1] = readbuffer[9];
  dummy[2] = readbuffer[10];
  dummy[3] = readbuffer[11];
  
  lm_pos = atoi(dummy);
  
  //for Servo One
  dummy[0] = readbuffer[13];
  dummy[1] = readbuffer[14];
  dummy[2] = readbuffer[15];
  dummy[3] = readbuffer[16];
  
  so_pos = atoi(dummy);
  
  //for Servo Two
  dummy[0] = readbuffer[18];
  dummy[1] = readbuffer[19];
  dummy[2] = readbuffer[20];
  dummy[3] = readbuffer[21];
  
  st_pos = atoi(dummy);
  
  // Buffer block 23-26 was for camera pan Dynamixel servo
  // This is not used for this robot
  
  //for status
  dummy[0] = readbuffer[28];
  dummy[1] = readbuffer[29];
  dummy[2] = readbuffer[30];
  dummy[3] = readbuffer[31];
  
  pcStatus = atoi(dummy);
}

void moveMotors(){ // Moves motors to command positions if ESTOP is off
  if (pcStatus == 100){ // checks for PC ESTOP
    rightMotor.writeMicroseconds(1500); // Stop right motor
    leftMotor.writeMicroseconds(1500); // Stop left motor
  } else {
    rightMotor.writeMicroseconds(rm_pos); // moves motor to command position
    leftMotor.writeMicroseconds(lm_pos); // moves motor to command position
  }
}

void moveServos(){ // Moves servos to command positions if ESTOP is off 
  if (pcStatus == 100){ // checks for PC ESTOP
    // Don't move anything
  } else {
    servoOne.writeMicroseconds(so_pos); // moves servo to command position
    servoTwo.writeMicroseconds(st_pos); // moves servo to command position
  }
}

void beatHeart(){ // Changes state of Heartbeat USER LED
  if (ledOn){
    ledOn = false; // Change stored state
    digitalWrite(heartbeatPin,HIGH); // Write new state
  } else {
    ledOn = true; // Change stored state
    digitalWrite(heartbeatPin,LOW); // Write new state
  }
}


