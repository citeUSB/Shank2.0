// ESP8266 WiFi <-> UART Bridge
#include <ESP8266WiFi.h>

// config: ////////////////////////////////////////////////////////////

#define UART_BAUD 115200
#define packTimeout 5 // ms (if nothing more on UART, then send packet)
#define bufferSize 8192

// For AP mode:
const char *ssid = "ESP SHANK";  // You will connect your phone to this Access Point
const char *pw = "123456cite"; // and this is the password
IPAddress ip(192, 168, 0, 1); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 9876; // and this port
// You must connect the phone to this AP, then:
// menu -> connect -> Internet(TCP) -> 192.168.0.1:9876
//////////////////////////////////////////////////////////////////////////

#include <WiFiClient.h>
WiFiServer server(port);
WiFiClient client;

uint8_t buf1[bufferSize];
uint8_t i1=0;

uint8_t buf2[bufferSize];
uint8_t i2=0;

void setup() {
  Serial.begin(UART_BAUD);
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  server.begin(); // start TCP server
}


void loop() {

  if(!client.connected()) { // if client not connected
    client = server.available(); // wait for it to connect
    return;
  }

  if(client.available()) {
    while(client.available()) {
      buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
      if(i1<bufferSize-1) i1++;
    }
    // now send to UART:
    Serial.write(buf1, i1);
    i1 = 0;
  }

  if(Serial.available()) {

    // read the data until pause:
    
    while(1) {
      if(Serial.available()) {
        buf2[i2] = (char)Serial.read(); // read char from UART
        if(i2<bufferSize-1) i2++;
      } else {
        //delayMicroseconds(packTimeoutMicros);
        delay(packTimeout);
        if(!Serial.available()) {
          break;
        }
      }
    }
    
    // now send to WiFi:
    client.write((char*)buf2, i2);
    i2 = 0;  
  }
}
