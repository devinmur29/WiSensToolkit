#include <Arduino.h>
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;
CommProtocol commType = WIFI;

//Wifi constants
IPAddress host(128,31,36,181);
const uint16_t port = 7000; //TCP port for sending and receiving. Can be any as long as it's currently unused.
const char* ssid = "StataCenter";
const char* pass = "";
WiFiConfig wifiConfig{ssid,pass,&host,port};
CommConfig commConfig;
const int adcPin = A2;
int rowPins[5] = {A0, A1, A5, 21, 12};
int readPins[5] = {27, 33, 15, 32, 14};

void setup() {
  // put your setup code here, to run once:
  commConfig.wifiConfig = wifiConfig;
  kit = new WiSensToolkit(32, 32, rowPins, readPins, adcPin, commType, &commConfig, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  kit->scanArray();
}