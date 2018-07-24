uint32_t medicion;
char buffer[5];

void setup() {
  medicion = 155;
  // initialize both serial ports:
  Serial.begin(115200);
  buffer[0] = medicion;
  buffer[1] = medicion>>8;
  buffer[2] = medicion>>16;
  buffer[3] = medicion>>24;
  buffer[4] = 0xFF;
}

void loop() {
  for (int i=0;i<5;i++){
    Serial.write(buffer[i]);
  }
  delay(500);
}
