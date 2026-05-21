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
                kit->calibrate();
            }
        }

        else if (receivedMessage.equals("noise"))
        {
            if (kit != nullptr)
            {
                kit->calibrateNoise();
            }
        }

        else if (receivedMessage.equals("mins"))
        {
            if (kit != nullptr)
            {
                kit->minCalibrate();
            }
        }
        else if (receivedMessage.equals("getPot"))
        {
            if (kit != nullptr)
            {
                kit->getPot();
            }
        }
        else if (receivedMessage.startsWith("updatePot:"))
        {
            uint8_t potStep = (uint8_t)receivedMessage.substring(10).toInt();
            if (kit != nullptr)
            {
                kit->updatePot(potStep);
            }
        }
        else if (receivedMessage.startsWith("updateMacAddress:"))
        {
            if (kit != nullptr)
            {
                String macStr = receivedMessage.substring(17);
                StaticJsonDocument<64> macDoc;
                DeserializationError error = deserializeJson(macDoc, macStr);
                if (!error && macDoc.is<JsonArray>() && macDoc.as<JsonArray>().size() == 6)
                {
                    uint8_t peer[6];
                    JsonArray arr = macDoc.as<JsonArray>();
                    for (int i = 0; i < 6; i++)
                        peer[i] = arr[i];
                    kit->updateMacAddress(peer);
                }
                else
                {
                    Serial.println("Invalid MAC: expected JSON array of 6 bytes, e.g. [170,187,204,221,238,255]");
                }
            }
        }
        else if (receivedMessage.equals("toggleMode"))
        {
            if (kit != nullptr)
            {
                kit->toggleMode();
            }
        }
        else if (receivedMessage.equals("getConfig"))
        {
            if (kit != nullptr)
            {
                kit->getConfig();
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