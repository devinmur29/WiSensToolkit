/*
    WiSensToolkit.cpp - Library for wireless tactile sensing
*/

#include <WiSensToolkit.h>
#include "Arduino.h"

#define CHANNEL 1
#define CONVERSIONS_PER_PIN 1

/**
 * @brief Clears a section of EEPROM by writing 0xFF to each byte.
 *
 * This function writes 0xFF (which is the default erased state for EEPROM)
 * to a range of addresses starting from `startAddress` for the given `length`.
 * It can be used to effectively "clear" a region of EEPROM memory.
 *
 * @param startAddress The starting address in EEPROM to begin clearing.
 * @param length The number of bytes to clear starting from `startAddress`.
 */
void clearEEPROM(int startAddress, int length)
{
    for (int i = 0; i < length; i++)
    {
        EEPROM.write(startAddress + i, 0xFF); // Write 0xFF to each byte
    }
}

void saveUint16ArrayToEEPROM(const uint16_t *data, size_t length)
{
    // Get JSONDoc from EEPROM
    JsonDocument doc;
    DeserializationError error;
    EepromStream eepromStream(0, EEPROM_SIZE);
    error = deserializeJson(doc, eepromStream);

    // Add offsets array to the doc
    JsonArray offsets = doc.createNestedArray("offsets"); // replaces existing value if present
    for (size_t i = 0; i < length; ++i)
    {
        offsets.add(data[i]);
    }
    // Put the JSONDoc back in EEPROM
    serializeJson(doc, eepromStream);
    eepromStream.flush();
}

bool loadUint16ArrayFromEEPROM(uint16_t *outputArray, size_t maxLength)
{

    EepromStream eepromStream(0, EEPROM_SIZE);
    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, eepromStream);
    if (error)
    {
        Serial.print("Failed to deserialize JSON from EEPROM: ");
        Serial.println(error.c_str());
        return false;
    }

    JsonArray offsets = doc["offsets"];
    if (!offsets.isNull())
    {
        int actualLength = 0;
        for (JsonVariant v : offsets)
        {
            outputArray[actualLength++] = v.as<uint16_t>();
        }
        return true;
    }
    else
    {
        Serial.println("No 'offsets' array found in JSON.");
        return false;
    }
}

#define SAMPLE_RATE 150000
#define I2S_DMA_BUF_LEN 8
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34

// ========== I2S INIT ==========
void setupI2S()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0};

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT, ADC_CHANNEL);
    i2s_adc_enable(I2S_NUM_0);
}

int16_t readI2S()
{
    const int totalSamples = 18;
    const int avgSamples = 4;

    uint16_t buffer[totalSamples];
    size_t bytes_read;

    // Bulk read 16 samples (each is 2 bytes)
    i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

    // Average the last 8 samples
    uint32_t sum = 0;
    for (int i = totalSamples - avgSamples; i < totalSamples; i++)
    {
        sum += buffer[i] & 0x0FFF; // Apply 12-bit ADC mask
    }

    return static_cast<int16_t>(sum / avgSamples);
}

/**
 * @brief Creates a WiSensToolkit object from user configuration
 *
 * This function creates a WiSensToolkit object that is configured
 * using a user's custom JSON configuration, provided either over
 * serial or saved on the device.
 *
 * @param useSaved bool that is true if configuration should be loaded from memory, false if we should wait for a configuration over serial
 */
WiSensToolkit *createKit(bool useSaved)
{
    const int address = 0;
    const int TIMEOUT = 300000; // 5 minutes timeout in milliseconds

    if (!Serial)
    {
        Serial.begin(250000);
    }

    Serial.println("Building kit");

    unsigned long startMillis = millis();
    Serial.println("Waiting for serial input...");

    JsonDocument doc;
    DeserializationError error;

    bool programmed = false;
    bool saved = false;
    EEPROM.begin(EEPROM_SIZE); // Initialize EEPROM

    if (useSaved)
    {
        EepromStream eepromStream(address, EEPROM_SIZE);
        error = deserializeJson(doc, eepromStream);
        // If this fails, we should wait for a new config
        if (error)
        {
            while (millis() - startMillis < TIMEOUT && !programmed)
            {
                if (Serial.available())
                {
                    String inputString = Serial.readStringUntil('\n');

                    if (inputString.length() > 0)
                    {
                        error = deserializeJson(doc, inputString);
                        if (error)
                        {
                            Serial.print("deserializeJson() failed: ");
                            Serial.println(error.c_str());
                            return nullptr;
                        }
                        else
                        {
                            EepromStream eepromStream(address, EEPROM_SIZE);
                            serializeJson(doc, eepromStream);
                            eepromStream.flush();
                            Serial.println("JSON String saved to EEPROM");
                            programmed = true;
                        }
                    }
                }
            }
        }
    }
    else
    {
        while (millis() - startMillis < TIMEOUT && !programmed)
        {
            if (Serial.available())
            {
                String inputString = Serial.readStringUntil('\n');

                if (inputString.length() > 0)
                {
                    error = deserializeJson(doc, inputString);
                    if (error)
                    {
                        Serial.print("deserializeJson() failed: ");
                        Serial.println(error.c_str());
                        return nullptr;
                    }
                    else
                    {
                        EepromStream eepromStream(address, EEPROM_SIZE);
                        serializeJson(doc, eepromStream);
                        eepromStream.flush();
                        Serial.println("JSON String saved to EEPROM");
                        programmed = true;
                    }
                }
            }
        }
    }

    // Get parameters which user MUST have provided in order to perform readout
    int id = doc["id"];
    const char *protocol = doc["protocol"];
    int numNodes = doc["numNodes"];

    JsonObject intermittent;
    bool intermittent_enabled = false;
    int intermittent_p = 41;
    int intermittent_d = 26;

    if (doc.containsKey("intermittent"))
    {
        intermittent = doc["intermittent"];
        intermittent_enabled = intermittent["enabled"];
        intermittent_p = intermittent["p"];
        intermittent_d = intermittent["d"];
    }

    JsonArray groundPinsArray = doc["groundPins"];
    int numGroundPins = groundPinsArray.size();
    int *groundPins = new int[numGroundPins];
    for (int i = 0; i < numGroundPins; i++)
    {
        groundPins[i] = groundPinsArray[i];
    }

    JsonArray readPinsArray = doc["readPins"];
    int numReadPins = readPinsArray.size();
    int *readPins = new int[numReadPins];
    for (int i = 0; i < numReadPins; i++)
    {
        readPins[i] = readPinsArray[i];
    }

    int adcPin = doc["adcPin"];
    int resistance = doc["resistance"];

    int startCoord[2] = {doc["startCoord"][0], doc["startCoord"][1]};
    int endCoord[2] = {doc["endCoord"][0], doc["endCoord"][1]};

    ReadoutConfig readoutConfig(numGroundPins, numReadPins, groundPins, readPins, startCoord, endCoord, adcPin, resistance);

    CommProtocol commType;
    CommConfig *commConfig = nullptr;
    int delay = 0;
    if (doc.containsKey("delay"))
    {
        delay = doc["delay"];
    }

    if (strcmp(protocol, "ble") == 0)
    {
        const char *deviceName = doc["deviceName"];
        commType = CommProtocol::BLUETOOTH;
        BluetoothConfig bleConfig(deviceName);
        if (delay == 0)
        {
            delay = 12;
        }
        commConfig = new CommConfig(commType, numNodes, delay, bleConfig);
    }
    else if (strcmp(protocol, "wifi") == 0)
    {
        commType = CommProtocol::WIFI;
        const char *hostString = doc["tcp_ip"];
        const uint16_t port = doc["port"];
        const char *ssid = doc["ssid"];
        const char *pass = doc["password"];
        WiFiConfig wiFiConfig(ssid, pass, hostString, port);
        commConfig = new CommConfig(commType, numNodes, delay, wiFiConfig);
    }
    else if (strcmp(protocol, "espnow") == 0)
    {
        commType = CommProtocol::ESPNOW;
        JsonArray peerAddress = doc["macAddress"];
        if (delay == 0)
        {
            delay = 12;
        }
        uint8_t peer[6];
        for (int i = 0; i < 6; i++)
        {
            peer[i] = peerAddress[i];
        }
        EspNowConfig espConfig(peer);
        commConfig = new CommConfig(commType, numNodes, delay, espConfig);
    }
    else
    {
        commType = CommProtocol::SERIALCOMM;
        commConfig = new CommConfig(commType, numNodes, delay);
    }

    WiSensToolkit *kit = new WiSensToolkit(&readoutConfig, commType, commConfig, id);

    kit->p = intermittent_p;
    kit->d = intermittent_d;
    kit->intermittent = intermittent_enabled;
    kit->saturatedPercentage = doc["saturatedPercentage"];
    kit->duration = doc["duration"];

    return kit;
}

/**
 * @brief WiSensToolkit constructor
 *
 * This function takes necessary configurations and initializes different modules
 * within the toolkit for readout and wireless communication
 *
 * @param readoutConfig Readout configuration
 * @param commType Type of communication protocol
 * @param commConfig Communication protocol configuration
 * @param sendId Id of the sending device
 */
WiSensToolkit::WiSensToolkit(ReadoutConfig *readoutConfig, CommProtocol commType, CommConfig *commConfig, int sendId)
{
    if (serial_communication)
    {
        if (!Serial)
        {
            Serial.begin(250000);
        }

        Serial.println("Initializing Toolkit");
    }

    thisReadoutConfig = new ReadoutConfig(*readoutConfig);
    thisCommConfig = commConfig;
    Serial.println(thisCommConfig->numNodes);
    Serial.println("Initiating comm protocol");
    String protocolInit;
    protocolInit = initCommProtocol(commType, commConfig);
    if (!protocolInit.equals("init"))
    {
        configString = protocolInit;
    }
    else
    {
        thisSendId = sendId;
        Serial.println("Comm protocol initiated");
        tactile_data = new struct_message(thisCommConfig->numNodes);
        tactile_data->sendId = thisSendId;
        Serial.println("Tactile struct initiated");
        bufferSize = sizeof(tactile_data->sendId) + sizeof(tactile_data->startIdx) + sizeof(uint16_t) * tactile_data->PressureArraySize + sizeof(uint32_t);
        buffer = new uint8_t[bufferSize];
        Serial.println("Buffers Initiated");
        int totalrows = (thisReadoutConfig->endCoord[1] - thisReadoutConfig->startCoord[1]) + 1;
        int totalcols = (thisReadoutConfig->endCoord[0] - thisReadoutConfig->startCoord[0]) + 1;
        currNodes = new TwoDArray(totalrows, totalcols);
        pastNodes = new TwoDArray(totalrows, totalcols);

        JsonDocument doc;
        DeserializationError error;
        EepromStream eepromStream(0, EEPROM_SIZE);
        error = deserializeJson(doc, eepromStream);
        totalNodes = totalrows * totalcols;
        offsets = new uint16_t[totalNodes]();

        if (doc.containsKey("offsets"))
        {
            loadUint16ArrayFromEEPROM(offsets, totalNodes);
        }

        totalError = 0;
        intermittentInit = false;
        firstPass = false;
        Serial.println("Intermittent Vars Initiated");
        ADCSetup();
        initDigiPot(thisReadoutConfig->resistance);
        toolkitInit = true;
    }
}

/**
 * @brief Sets up digitial potentiometer
 *
 * @param potStep Value to set potentiometer to
 */
void WiSensToolkit::initDigiPot(uint8_t potStep)
{
    MCP = new MCP4018;
    while (!(*MCP).begin())
    {
        Serial.print("\nMCP4018 could not be found...");
        delay(200);
    }
    (*MCP).reset();
    (*MCP).setWiperByte(potStep);
    Serial.print("Set Resistance to ");
    Serial.println(calculateResistance(potStep, 50000));
}

/**
 * @brief Initiates Communication Protocol
 *
 * @param commType type of the communication protocol to initiate
 * @param commConfig communication protocol specific configurations
 */
String WiSensToolkit::initCommProtocol(CommProtocol commType, CommConfig *commConfig)
{
    currentCommType = commType;
    thisCommConfig = commConfig;
    Serial.println("Comm config set");
    String commInit = "init";
    switch (commType)
    {
    case CommProtocol::ESPNOW:
        EspNowSetup();
        break;
    case CommProtocol::WIFI:
        commInit = WiFiSetup();
        break;
    case CommProtocol::BLUETOOTH:
        BluetoothSetup();
        break;
    default:
        Serial.println("Serial case");
        break;
    }

    return commInit;
}
/**
 * @brief Configures the device for BLE communications
 */
void WiSensToolkit::BluetoothSetup()
{
    BLEDevice::init(std::get<BluetoothConfig>(thisCommConfig->config).deviceName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(&deviceConnected));
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    pService->start();

    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("Started Advertising");
}

/**
 * @brief Configures the device for ESP-NOW communication
 */
void WiSensToolkit::EspNowSetup()
{
    WiFi.mode(WIFI_STA);
    // This is the mac address of the Master in Station Mode
    Serial.print("STA MAC: ");
    Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
    }
    // Register peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    for (int i = 0; i < 6; i++)
    {
        Serial.println(std::get<EspNowConfig>(thisCommConfig->config).peerAddress[i]);
    }
    memcpy(peerInfo.peer_addr, std::get<EspNowConfig>(thisCommConfig->config).peerAddress, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    // Add peer
    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus == ESP_OK)
    {
        // Pair success
        Serial.println("Pair success");
    }
    else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
    {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
    }
    else if (addStatus == ESP_ERR_ESPNOW_ARG)
    {
        Serial.println("Invalid Argument");
    }
    else if (addStatus == ESP_ERR_ESPNOW_FULL)
    {
        Serial.println("Peer list full");
    }
    else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
    {
        Serial.println("Out of memory");
    }
    else if (addStatus == ESP_ERR_ESPNOW_EXIST)
    {
        Serial.println("Peer Exists");
    }
    else
    {
        Serial.println("Not sure what happened");
    }
}

/**
 * @brief Configures the device for WiFi Communication
 */
String WiSensToolkit::WiFiSetup()
{
    // Wifi Setup setup
    tactileClient = new WiFiClient;
    thisIp = new IPAddress();
    thisIp->fromString(std::get<WiFiConfig>(thisCommConfig->config).host);
    WiFi.mode(WIFI_STA);

    WiFi.begin(std::get<WiFiConfig>(thisCommConfig->config).ssid, std::get<WiFiConfig>(thisCommConfig->config).password);

    while (WiFi.status() != WL_CONNECTED)
    {

        delay(500);
        if (Serial.available() > 0)
        {
            String receivedMessage = Serial.readStringUntil('\n');
            receivedMessage.trim();
            return receivedMessage;
        }
    }

    while (!tactileClient->connect(*thisIp, std::get<WiFiConfig>(thisCommConfig->config).port))
    {
        delay(500);
        if (Serial.available() > 0)
        {
            String receivedMessage = Serial.readStringUntil('\n');
            receivedMessage.trim();
            return receivedMessage;
        }
    }

    Serial.println("Connected to TCP Port!");
    return "init";
}

/**
 * @brief Sets up the Analog to Digital Converter
 */
void WiSensToolkit::ADCSetup()
{
    Serial.println("Setting up ADC");
    for (int i = 0; i < 5; i++)
    {
        pinMode(thisReadoutConfig->groundPins[i], OUTPUT);
        pinMode(thisReadoutConfig->readPins[i], OUTPUT);
        digitalWrite(thisReadoutConfig->groundPins[i], LOW);
        digitalWrite(thisReadoutConfig->readPins[i], LOW);
    }
    // pinMode(thisReadoutConfig->adcPin, INPUT);
}

// ISR Function that will be triggered when ADC conversion is done
// void ARDUINO_ISR_ATTR adcComplete()
// {
//     adc_conversion_done = true;
// }

/**
 * @brief Callback function which controls the start of calibration
 *
 * Calibration will start when this method returns true
 */
boolean sensitvitycalibrationCallback()
{
    if (Serial.available() > 0)
    {
        String message = Serial.readStringUntil('\n');
        if (message == "sense")
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Callback function which controls the start of calibration
 *
 * Calibration will start when this method returns true
 */
boolean noisecalibrationCallback()
{
    if (Serial.available() > 0)
    {
        String message = Serial.readStringUntil('\n');
        if (message == "noise")
        {
            return true;
        }
    }
    return false;
}

void WiSensToolkit::calibrateNoise()
{
    int *startCoord = thisReadoutConfig->startCoord;
    int *endCoord = thisReadoutConfig->endCoord;
    uint16_t nodeIdx;
    uint16_t reading;
    int totalrows = (thisReadoutConfig->endCoord[1] - thisReadoutConfig->startCoord[1]) + 1;
    int totalcols = (thisReadoutConfig->endCoord[0] - thisReadoutConfig->startCoord[0]) + 1;
    int numCycles = 100;
    Serial.println("Starting noise calibration");

    // Caclulate average across nodes for 10 cycles
    // Set Max reading - average as the offset to add for each node
    for (int y = startCoord[1]; y <= endCoord[1]; y++)
    {
        selectMuxPin(y, true);
        for (int x = startCoord[0]; x <= endCoord[0]; x++)
        {
            int fixedX = x - startCoord[0];
            int fixedY = y - startCoord[1];
            nodeIdx = (endCoord[0] - startCoord[0] + 1) * (y - startCoord[1]) + (x - startCoord[0]);
            if (packetCount == 0)
            {
                tactile_data->startIdx = nodeIdx;
            }
            selectMuxPin(x, false);

            int32_t sum = 0;
            for (int i = 0; i < numCycles; i++)
            {
                sum += readI2S();
            }
            offsets[nodeIdx] = 3072 - sum / numCycles;
        }
    }

    saveUint16ArrayToEEPROM(offsets, totalrows * totalcols);
    Serial.println("Finished noise calibration");
}

/**
 * @brief Calibrate the device for a particular pressure sensing application
 *
 * This method finds the value of the resistance for the digital potentiometer which maximizes
 * ADC range while avoiding saturation
 *
 */
// void WiSensToolkit::calibrate()
// {

//     uint8_t calResistance = 126; // Resistance to run calibration at (about 393 ohms to capture maximum possible sensitivity)
//     uint8_t packetCount = 0;
//     uint16_t saturateThreshold = totalNodes * saturatedPercentage;
//     Serial.print("Threshold is ");
//     Serial.println(saturateThreshold);
//     MinValuesTracker tracker(saturateThreshold);
//     uint16_t reading;
//     uint16_t nodeIdx;
//     int *startCoord = thisReadoutConfig->startCoord;
//     int *endCoord = thisReadoutConfig->endCoord;
//     (*MCP).setWiperByte(calResistance);

//     // for (size_t i = 0; i < totalNodes; i++)
//     // {
//     //     offsets[i] = 2235;
//     // }

//     // while (!noisecalibrationCallback())
//     // {
//     //     Serial.print('.');
//     //     delay(500);
//     // }

//     // Serial.println("Waiting to start calibration");
//     // while (!sensitvitycalibrationCallback())
//     // {
//     //     Serial.print('.');
//     //     delay(500);
//     // }
//     // Serial.println("Starting Calibration");
//     long startTime = millis();
//     long elapsedTime = 0;
//     while (elapsedTime < duration)
//     {
//         for (int y = startCoord[1]; y <= endCoord[1]; y++)
//         {
//             selectMuxPin(y, true);
//             for (int x = startCoord[0]; x <= endCoord[0]; x++)
//             {
//                 nodeIdx = (endCoord[0] - startCoord[0] + 1) * (y - startCoord[1]) + (x - startCoord[0]);
//                 if (packetCount == 0)
//                 {
//                     tactile_data->startIdx = nodeIdx;
//                 }
//                 selectMuxPin(x, false);
//                 reading = analogRead(thisReadoutConfig->adcPin);
//                 // if (reading < 2235 - offsets[nodeIdx])
//                 // {
//                 //     reading += offsets[nodeIdx];
//                 // }
//                 tactile_data->TruePressure[packetCount] = reading;
//                 if (packetCount == thisCommConfig->numNodes - 1)
//                 {
//                     createBuffer(tactile_data);
//                     sendResult();
//                     packetCount = 0;
//                 }
//                 else
//                 {
//                     packetCount += 1;
//                 }

//                 // track the adc readout using the min tracker
//                 tracker.addNode(nodeIdx, reading);

//                 // increment elapsed time
//                 elapsedTime = millis() - startTime;
//             }
//         }
//     }

//     double vout2 = tracker.avgMinValues();
//     double resOhms = calculateResistance(calResistance, 50000);

//     // Solve for approx voltage after stage 1 opamp
//     double vout1 = (220 / resOhms) * (1.8 - (vout2 / 4096) * 3.3) + 1.8;

//     // Solve for Rpot which will make minimum voltage reading = 0
//     double rpot = (1.8 * 220) / (vout1 - 1.8);

//     uint8_t potValue = calculatePotValue(rpot, 50000);

//     Serial.print("Resistance :");
//     Serial.println(potValue);
//     initDigiPot(potValue);
// }

/**
 * @brief converts from a discrete pot step to a resistance in Ohms
 *
 * @param potValue Integer representing digipot step
 * @param maxResistance maximum resistance of the digipot (in ohms)
 */
double WiSensToolkit::calculateResistance(uint8_t potValue, int maxResistance)
{
    return ((127.0 - potValue) / 127.0) * maxResistance;
}

/**
 * @brief converts from a resistance in ohms to a discrete pot step
 *
 * @param resistance resistance to convert from (in ohms)
 * @param maxResistance maximum resistance of the digipot (in ohms)
 */
uint8_t WiSensToolkit::calculatePotValue(double resistance, int maxResistance)
{
    return static_cast<uint8_t>(127.0 - ((resistance * 127.0 / maxResistance) + 0.999));
}

/**
 * @brief read the tactile sensing array set by the user
 *
 * Reads and sends the voltages from
 * the tactile sensing array configured by the bounding box (startCoord, endCoord)
 * where startCoord and endCoord are of the form (readWire, groundWire). Sends after numNodes readings.
 */
void WiSensToolkit::scanArray()
{
    // Serial.println("Scanning array");
    int *startCoord = thisReadoutConfig->startCoord;
    int *endCoord = thisReadoutConfig->endCoord;
    uint16_t i2s_read_buff; // 2 bytes per sample
    size_t bytes_read;
    // int start = micros();
    for (int y = startCoord[1]; y <= endCoord[1]; y++)
    {
        selectMuxPin(y, true);
        for (int x = startCoord[0]; x <= endCoord[0]; x++)
        {
            int fixedX = x - startCoord[0];
            int fixedY = y - startCoord[1];
            int nodeIdx = (endCoord[0] - startCoord[0] + 1) * fixedY + fixedX;
            if (packetCount == 0)
            {
                tactile_data->startIdx = (endCoord[0] - startCoord[0] + 1) * (fixedY) + (fixedX);
            }
            selectMuxPin(x, false);

            // Read actual sample
            // i2s_read(I2S_NUM_0, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
            // uint16_t raw = i2s_read_buff & 0x0FFF; // Mask to 12-bit
            int16_t trueReading = readI2S() + offsets[nodeIdx];
            if (trueReading > 3072)
            {
                trueReading = 3072;
            }

            // int16_t trueReading = analogRead(thisReadoutConfig->adcPin);
            // Serial.println(trueReading);
            // int16_t trueReading = 0;
            if (intermittent)
            {
                if (intermittentInit)
                {
                    int16_t pred;
                    if ((1 / p) * (*pastNodes)(fixedY, fixedX) < (1 + (1 / p)) * (*currNodes)(fixedY, fixedX))
                    {
                        pred = (1 + (1 / p)) * (*currNodes)(fixedY, fixedX) - (1 / p) * (*pastNodes)(fixedY, fixedX);
                    }
                    else
                    {
                        pred = 0;
                    }
                    tactile_data->PredPressure[packetCount] = pred;
                    totalError = totalError + abs(pred - trueReading);
                }
                else
                {
                    if (!firstPass && x == endCoord[0] && y == endCoord[1])
                    {
                        firstPass = true;
                    }

                    if (firstPass && x == endCoord[0] && y == endCoord[1])
                    {
                        intermittentInit = true;
                    }
                }
                (*pastNodes)(fixedY, fixedX) = (*currNodes)(fixedY, fixedX);
                (*currNodes)(fixedY, fixedX) = trueReading;
            }
            tactile_data->TruePressure[packetCount] = trueReading;
            // Serial.println(packetCount);
            if (packetCount == thisCommConfig->numNodes - 1)
            {
                // Serial.println(micros() - start);
                float avgError = totalError / thisCommConfig->numNodes;
                createBuffer(tactile_data);
                if (!intermittent || avgError > d || !intermittentInit)
                {
                    sendResult();
                }
                else
                {
                    // Don't send, use prediction data instead
                    (*currNodes).copyInto(tactile_data->startIdx, tactile_data->PredPressure, thisCommConfig->numNodes);
                }
                packetCount = 0;
                totalError = 0;
            }
            else
            {
                packetCount += 1;
            }
        }
    }
}

/**
 * @brief read the voltage at a particular node
 *
 * Reads the voltage at the node (readWire, groundWire)
 * Sends the read voltages after numNodes readings
 */

// void WiSensToolkit::readNode(int readWire, int groundWire)
// {
//     tactile_data->startIdx = 20000;
//     selectMuxPin(groundWire, true);
//     selectMuxPin(readWire, false);
//     int *startCoord = thisReadoutConfig->startCoord;
//     int *endCoord = thisReadoutConfig->endCoord;
//     int fixedX = readWire - startCoord[0];
//     int fixedY = groundWire - startCoord[1];
//     int16_t trueReading = analogRead(thisReadoutConfig->adcPin);
//     tactile_data->TruePressure[packetCount] = (endCoord[0] - startCoord[0] + 1) * (fixedY) + (fixedX);
//     tactile_data->TruePressure[packetCount + 1] = trueReading;
//     if (packetCount == (thisCommConfig->numNodes) - 2)
//     {
//         createBuffer(tactile_data);
//         sendResult();
//         packetCount = 0;
//     }
//     else
//     {
//         packetCount += 2;
//     }
// }

/**
 * @brief Toggle the digital pins to select grounding and reading wires
 *
 * @param pin Byte representing # of wire to select
 * @param ground True if the pin is for a grounding wire
 */

void WiSensToolkit::selectMuxPin(int pin, bool ground)
{
    if (ground)
    {
        for (int i = 0; i < thisReadoutConfig->numGroundPins; i++)
        {
            if (pin & (1 << i))
            {
                digitalWrite(thisReadoutConfig->groundPins[i], HIGH);
            }
            else
            {
                digitalWrite(thisReadoutConfig->groundPins[i], LOW);
            }
        }
    }
    else
    {
        for (int i = 0; i < thisReadoutConfig->numReadPins; i++)
        {
            if (pin & (1 << i))
            {
                digitalWrite(thisReadoutConfig->readPins[i], HIGH);
            }
            else
            {
                digitalWrite(thisReadoutConfig->readPins[i], LOW);
            }
        }
    }
}
/**
 * @brief Send the current buffer using the configured communication protocol
 */
void WiSensToolkit::sendResult()
{
    switch (currentCommType)
    {
    case CommProtocol::ESPNOW:
        sendEspNow();
        break;
    case CommProtocol::WIFI:
        sendWiFi();
        break;
    case CommProtocol::BLUETOOTH:
        sendBluetooth();
        break;
    default:
        sendSerial();
    }
}

void WiSensToolkit::stopComms()
{
    switch (currentCommType)
    {
    case CommProtocol::WIFI:
        tactileClient->stop();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        break;
    case CommProtocol::BLUETOOTH:
        if (pAdvertising)
        {
            pAdvertising->stop();
            pAdvertising = nullptr;
        }

        if (pServer)
        {
            pServer->removeService(pService);
            pServer = nullptr;
            pService = nullptr;
        }

        if (pCharacteristic)
        {
            pCharacteristic = nullptr;
        }

        BLEDevice::deinit(true); // Fully releases memory used by NimBLE stack
        deviceConnected = false;
        break;
    default:
        break;
    }
}

/**
 *@brief Compose the packet and increase the packetNumber
 */
void WiSensToolkit::createBuffer(struct_message *tactile_data)
{
    // Copy the data into the buffer
    size_t offset = 0;
    memcpy(buffer + offset, &tactile_data->sendId, sizeof(tactile_data->sendId));
    offset += sizeof(tactile_data->sendId);
    memcpy(buffer + offset, &tactile_data->startIdx, sizeof(tactile_data->startIdx));
    offset += sizeof(tactile_data->startIdx);
    memcpy(buffer + offset, tactile_data->TruePressure, sizeof(uint16_t) * tactile_data->PressureArraySize);
    offset += sizeof(uint16_t) * tactile_data->PressureArraySize;
    memcpy(buffer + offset, &packetNumber, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    packetNumber += 1;
}

/**
 * @brief Send packet via ESP-NOW protocol
 */
void WiSensToolkit::sendEspNow()
{
    // Send the buffer
    esp_err_t result = esp_now_send(std::get<EspNowConfig>(thisCommConfig->config).peerAddress, buffer, bufferSize);
    if (result == ESP_OK)
    {
        // Serial.println("Sent with Success");
    }
    else
    {
        Serial.println("Error sending the data");
    }
    if (thisCommConfig->delay != 0)
    {
        delay(thisCommConfig->delay);
    }
}

/**
 * @brief Send packet via TCP connection
 */
void WiSensToolkit::sendWiFi()
{
    if (tactileClient->connected())
    {
        tactileClient->write(buffer, bufferSize);
        if (thisCommConfig->delay != 0)
        {
            delay(thisCommConfig->delay);
        }
    }
    else
    {
        Serial.println("Client not connected!");

        while (!tactileClient->connect(*thisIp, std::get<WiFiConfig>(thisCommConfig->config).port))
        {
            Serial.println("Reconnecting");
            delay(500);
        }
    }
}

WiSensToolkit::MyServerCallbacks::MyServerCallbacks(bool *_bptr)
{
    connection_ptr = _bptr;
    BLEServerCallbacks();
}

void WiSensToolkit::MyServerCallbacks::onConnect(BLEServer *pServer)
{
    *connection_ptr = true;
}

void WiSensToolkit::MyServerCallbacks::onDisconnect(BLEServer *pServer)
{
    *connection_ptr = false;
}

/**
 * @brief Send packet via bluetooth connection
 */
void WiSensToolkit::sendBluetooth()
{
    if (deviceConnected)
    {
        pCharacteristic->setValue(buffer, bufferSize);
        pCharacteristic->notify();
        if (thisCommConfig->delay != 0)
        {
            delay(thisCommConfig->delay);
        }
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);
        pServer->startAdvertising();
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        oldDeviceConnected = deviceConnected;
    }
}

/**
 * @brief Send packet via serial connection
 */
void WiSensToolkit::sendSerial()
{
    Serial.write((uint8_t *)buffer, bufferSize);
    Serial.write("wr");
}