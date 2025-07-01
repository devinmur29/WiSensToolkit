#include <Arduino.h>
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;

void setup()
{
    Serial.begin(250000);
    kit = createKit(true);
    while (!kit->toolkitInit)
    {
        StaticJsonDocument<4096> doc;
        Serial.println("Serializing with new config string");
        DeserializationError error = deserializeJson(doc, kit->configString);
        EepromStream eepromStream(0, EEPROM_SIZE);
        serializeJson(doc, eepromStream);
        eepromStream.flush();
        Serial.println("New config saved to EEPROM.");
        kit->stopComms();
        delete kit;
        kit = nullptr;
        kit = createKit(true);
    }
    setupI2S();
}

void loop()
{
    if (Serial.available() > 0)
    {
        String receivedMessage = Serial.readStringUntil('\n');
        receivedMessage.trim();

        if (receivedMessage.equals("calibrate"))
        {
            if (kit != nullptr)
            {
                // kit->calibrate();
            }
        }

        else if (receivedMessage.equals("noise"))
        {
            if (kit != nullptr)
            {
                kit->calibrateNoise();
            }
        }
        else
        {
            StaticJsonDocument<4096> doc;
            DeserializationError error = deserializeJson(doc, receivedMessage);

            if (!error)
            {
                Serial.println("Received new config. Reinitializing kit.");

                if (kit != nullptr)
                {
                    kit->stopComms(); // You’ll implement this in your class
                    delete kit;
                    kit = nullptr;
                }

                EepromStream eepromStream(0, EEPROM_SIZE);
                serializeJson(doc, eepromStream);
                eepromStream.flush();
                Serial.println("New config saved to EEPROM.");

                kit = createKit(true);
                Serial.println("Kit reinitialized.");
            }
            else
            {
                Serial.print("Invalid JSON config: ");
                Serial.println(error.c_str());
            }
        }
    }
    // Serial.println("Scanning array");
    kit->scanArray();
    // Serial.println(micros());
}