#include <Arduino.h>
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;

void setup()
{
    Serial.begin(250000);
    kit = createKit(false);
    kit->calibrate(30000, 1);
}

void loop()
{
    kit->scanArray();
}