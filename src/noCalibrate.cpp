#include <Arduino.h>
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;

void setup()
{
    Serial.begin(250000);
    kit = createKit(true);
    while (!kit->toolkitInit)
    {
        StaticJsonDocument<1024> doc;
        Serial.println("Serializing with new config string");
        DeserializationError error = deserializeJson(doc, kit->configString);
        EepromStream eepromStream(0, 512);
        serializeJson(doc, eepromStream);
        eepromStream.flush();
        Serial.println("New config saved to EEPROM.");
        kit->stopComms();
        delete kit;
        kit = nullptr;
        kit = createKit(true);
    }
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
                kit->calibrate();
            }
        }
        else
        {
            StaticJsonDocument<1024> doc;
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

                EepromStream eepromStream(0, 512);
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
    kit->scanArray();
    // Serial.println(micros());
}