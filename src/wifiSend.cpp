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
int rowPins[5] = {A0, A1, A5, 21, 12};
int readPins[5] = {27, 33, 15, 32, 14};
ReadoutConfig readoutConfig {
  .numGroundWires = 16,
  .numReadWires = 16,
  .groundPins = rowPins,
  .readPins = readPins,
  .adcPin = A2,
  .resistance = 250
};

void setup() {
  // put your setup code here, to run once:
  commConfig.wifiConfig = wifiConfig;
  kit = new WiSensToolkit(&readoutConfig, commType, &commConfig, 5);
}

void loop() {
  // put your main code here, to run repeatedly:
  kit->scanArray(120,false);
  //kit->readNode(0,0);
}