uint32_t medicion;
char buffer[4];

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  // read from port 1, send to port 0:
  /*if (Serial1.available() > 0) {
   char inByte = Serial1.read();
   Serial.println(inByte); 
  }*/
  Serial1.readBytesUntil(0xFF,buffer,4);
  medicion = ((uint32_t)buffer[0])|((uint32_t)buffer[1]<<8)|((uint32_t)buffer[2]<<16)|((uint32_t)buffer[3]<<24);
  Serial.println(medicion); 
}
