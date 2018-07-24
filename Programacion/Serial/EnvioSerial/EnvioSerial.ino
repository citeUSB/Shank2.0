/*
  Multple serial test
 
  Receives from the main serial port, sends to the others. 
  Receives from serial port 1, sends to the main serial (Serial 0).
 
  The circuit: 
  * Any serial device attached to Serial port 1
  * Serial monitor open on Serial port 0:
 
  created 30 Dec. 2008
  by Tom Igoe
 
  This example code is in the public domain.
 
*/


void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
}

void loop() {
  Serial.write('c');
  delay(500);
}
