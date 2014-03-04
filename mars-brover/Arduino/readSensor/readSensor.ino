/*This routine will read values from a generic sensor on pin 2 
and send the values to the PC using the XBEE */

int sensor = 2; // Physical Pin of the sensor on the Arbotix
int  raw = 0; // Storage for the raw sensor reading
float volts = 0.0; // Storage for the voltage value of the sensor
int p = 100; // Sensor Read Period (ms)
int led = 0;  // USER LED pin for heartbeat

//*************************************************
void setup() {
  // Serial prints are sent over the XBEE radio pseudo-serial port
  Serial.begin(38400); // start serial port at 38400 bps:
  delay(1000); // wait to connect  
  pinMode(led, OUTPUT); // initialize USER LED
  //pinMode(sensor, INPUT); // initialize SENSOR pin
}

//****************************************************
void loop() {
  digitalWrite(led, HIGH); // turn LED on
  raw = analogRead(sensor);    // read the input pin
  volts = (float)raw * 5.0 / 1024.0;  // convert to volts
  Serial.println(volts); // Send voltage reading
  delay(p/2);
  digitalWrite(led, LOW); // turn LED off
  delay(p/2);  
}
