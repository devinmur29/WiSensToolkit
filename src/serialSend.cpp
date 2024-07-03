#include <Arduino.h>
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;
CommProtocol commType = SERIALCOMM;
CommConfig commConfig;
int rowPins[5] = {A0, A1, A5, 21, 12};
int readPins[5] = {27, 33, 15, 32, 14};
ReadoutConfig readoutConfig {
  .numGroundPins = 5,
  .numReadPins = 5,
  .groundPins = rowPins,
  .readPins = readPins,
  .adcPin = A2,
  .resistance = 9000};

int startCoord[2] = {0,0};
int endCoord[2] = {31,31};

void setup() {
  // put your setup code here, to run once:
  kit = new WiSensToolkit(&readoutConfig, commType, &commConfig, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  kit->scanArray(120, startCoord, endCoord, false);
  //kit->readNode(0,0);
}