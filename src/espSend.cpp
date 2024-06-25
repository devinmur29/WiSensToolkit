#include <Arduino.h>
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;
CommProtocol commType = ESPNOW;
EspNowConfig espConfig{0x70, 0xB8, 0xF6, 0x84, 0x5A, 0xD8};
CommConfig commConfig;
const int adcPin = A2;
int rowPins[5] = {A0, A1, A5, 21, 12};
int readPins[5] = {27, 33, 15, 32, 14};

void setup() {
  // put your setup code here, to run once:
  commConfig.espNowConfig = espConfig;
  kit = new WiSensToolkit(32, 32, rowPins, readPins, adcPin, commType, &commConfig, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  kit->scanArray();
}