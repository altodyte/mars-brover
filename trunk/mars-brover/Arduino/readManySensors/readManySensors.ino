/*This routine will read values from a bunch of sensors
and send the values to the PC using the XBEE */

/* Digital: 0: OUT LED
            1: IN  Bumper Switch
   Analog:  0: IN  Lidar 1
            1: IN  Lidar 2
            2: IN  IR
            3: IN  Encoder (Can this be digital?)
            4:
            5: IN  Color (Blue)
            6: IN  Color (Green)
            7: IN  Color (Red)
*/

// TO BE IMPLEMENTED: BUFFER-BASED PRINTING

// SENSOR INITIALIZATION INFORMATION
const int numDig = 3;
const int numAnal = 8;
const int digSensors[numDig] = {2,3,4}; // Physical Pins of the digital sensors on the Arbotix
const int analSensors[numAnal] = {0,1,2,3,4,5,6,7}; // Physical Pins of the analog sensors on the Arbotix
int digRes[numDig] = {}; // Storage for the raw digital sensor readings
int analRaw[numAnal] = {}; // Storage for the raw analog sensor readings
float analVolts[numAnal] = {}; // Storage for the voltage values of the analog sensors
int p = 100; // Sensor Read Period (ms)

// CONTROL INITIALIZATION INFORMATION
int led = 0;  // USER LED pin for heartbeat

//*************************************************
void setup() {
  // Serial prints are sent over the XBEE radio pseudo-serial port
  Serial.begin(38400); // start serial port at 38400 bps:
  delay(1000); // wait to connect  
  pinMode(led, OUTPUT); // initialize USER LED
  pinMode(1, OUTPUT); // initialize color sensor digital
  digitalWrite(1,HIGH); // turn on color sensor LED
  for (int i=0; i<numDig; i++){
    pinMode(digSensors[i],INPUT); // initialize each of the digital inputs
  }
}

//****************************************************
void loop() {
  digitalWrite(led, HIGH); // turn LED on
  for (int i=0; i<numDig; i++){
    digRes[i] = digitalRead(digSensors[i]); // read each of the digital inputs
  }
  for (int i=0; i<numAnal; i++){
    analRaw[i] = analogRead(analSensors[i]); // read each of the analog inputs
    analVolts[i] = (float)analRaw[i] * 5.0 / 1024.0; // convert to volts
  }
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
  Serial.println("#"); // Send voltage reading
  delay(p/2);
  digitalWrite(led, LOW); // turn LED off
  delay(p/2);  
}
