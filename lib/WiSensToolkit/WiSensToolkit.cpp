/*
    WiSensToolkit.cpp - Library for wireless tactile sensing
*/

#include <Arduino.h>
#include <WiSensToolkit.h>
#define CHANNEL 1

WiSensToolkit::WiSensToolkit(int numRows, int numCols, int* rowPins, int* readPins, int adcPin,  CommProtocol commType, CommConfig* commConfig, int sendId)
:tactile_data(120){
    if (serial_communication) {
        Serial.begin(250000);
        Serial.println("Initializing Toolkit");
    }

    initCommProtocol(commType, commConfig, sendId);
    _rowPins = rowPins;
    _readPins = readPins;
    _numRowPins = calcNumPins(numRows-1);
    _numColPins = calcNumPins(numCols-1);
    Serial.println("Number of pins");
    Serial.println(_numRowPins);
    Serial.println(_numColPins);
    ADCSetup(); 
    //initDigiPot(2000);

    //_numPins = sizeof(rowPins) / sizeof(rowPins[0]);
    _numRows = numRows;
    _numCols = numCols;
    zInput = adcPin;
    //bufferSize = sizeof(tactile_data.sendId) + sizeof(tactile_data.PacketIndex) + sizeof(uint16_t) * tactile_data.PressureArraySize;
    bufferSize = sizeof(tactile_data.sendId) + sizeof(tactile_data.startIdx)  + sizeof(uint16_t) * tactile_data.PressureArraySize + sizeof(uint32_t);
    buffer = new uint8_t[bufferSize];
}

void WiSensToolkit::initDigiPot(int res){
    MCP = new MCP4018;
    while (!(*MCP).begin()){
        Serial.print("\nMCP4018 could not be found...");
        delay(200);
    }
    (*MCP).reset();
    (*MCP).setWiperResistance(res);
    Serial.print("Set Resistance to ");
    Serial.print(res);
    Serial.println();
}

void WiSensToolkit::initCommProtocol(CommProtocol commType, CommConfig* commConfig, int sendId){
    currentCommType = commType;
    tactile_data.sendId = sendId;
    currentCommConfig = commConfig;
    switch (commType)
    {
    case ESPNOW:
        EspNowSetup();
        break;
    case WIFI:
        WiFiSetup();
        break;
    case BLUETOOTH:
        BluetoothSetup();
        break;
    default:
        break;
    }
}

void WiSensToolkit::BluetoothSetup(){
    Serial.println("Initiating Bluetooth");
    BLEDevice::init(currentCommConfig->bluetoothConfig.deviceName);
    pServer = BLEDevice::createServer();
    Serial.println("Created Server");
    pServer->setCallbacks(new MyServerCallbacks(&deviceConnected));
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE|
        NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();
    Serial.println("Started Service");

    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Started Advertising");
}


void WiSensToolkit::EspNowSetup(){
    WiFi.mode(WIFI_STA);
    // This is the mac address of the Master in Station Mode
    Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
    }
    // Register peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, currentCommConfig->espNowConfig.peerAddress, 6);
    peerInfo.channel = CHANNEL;  
    peerInfo.encrypt = 0;
    // Add peer        
    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
    } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
    } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
    } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
    } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
    } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
    } else {
        Serial.println("Not sure what happened");
    }
}



void WiSensToolkit::WiFiSetup(){
     // Wifi Setup setup
    tactileClient = new WiFiClient;
    WiFi.mode(WIFI_STA);

    WiFi.begin(currentCommConfig->wifiConfig.ssid, currentCommConfig->wifiConfig.password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());

    while (!tactileClient->connect(*currentCommConfig->wifiConfig.host, currentCommConfig->wifiConfig.port)) {
        Serial.println("Initial connection failed");
        delay(500);
    }

    Serial.println("Connected to TCP Port!");
}
void WiSensToolkit::ADCSetup(){
    for (int i = 0; i < _numRowPins; i++) {
    Serial.println("Setting pin mode");
    pinMode(_rowPins[i], OUTPUT);
    pinMode(_readPins[i], OUTPUT);
    digitalWrite(_rowPins[i], LOW);
    digitalWrite(_readPins[i], LOW);
  }
  pinMode(zInput, INPUT);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);
}

void WiSensToolkit::scanArray() {
  for (int y = 0; y <= _numRows-1; y++) {
    //tactile_data.PacketIndex = y;
    selectMuxPin(y, true);
      for(int x = 0; x <= _numCols-1; x++) {
        if (packetCount == 0) {
            tactile_data.startIdx = _numRows*y+x;
        }
        
        selectMuxPin(x, false);
        //tactile_data.PressureArray[packetCount] = analogRead(zInput);
        tactile_data.PressureArray[x] = random(1024);
        if (packetCount == 119) {
            sendResult();
            packetCount = 0;
        } else {
            packetCount += 1;
        }
        

        
      }
    // Send resut through desired means
    //sendResult();
  }  
}

void WiSensToolkit::readNode(int row, int col){
    selectMuxPin(row, true);
    selectMuxPin(col,false);
    Serial.println(analogRead(zInput));
}
void WiSensToolkit::selectMuxPin(byte pin, bool row) {
    for (int i=0; i<_numRowPins; i++)
        {
            if (pin & (1<<i)) {
                if (row) {
                    digitalWrite(_rowPins[i], HIGH);
                }
                else {
                    digitalWrite(_readPins[i],HIGH);
                }
                
            }
            else {
                if (row) {
                    digitalWrite(_rowPins[i], LOW);
                }
                else {
                    digitalWrite(_readPins[i], LOW);
                }
            }
        }
}

void WiSensToolkit::sendResult() {
    switch (currentCommType)
    {
    case ESPNOW:
        sendEspNow();
        break;
    case WIFI:
        sendWiFi();
        break;
    case BLUETOOTH:
        sendBluetooth();
        break;
    default:
        sendSerial();
    }
     
}

// void WiSensToolkit::createBuffer(){
//     // Copy the data into the buffer
//     size_t offset = 0;
//     memcpy(buffer + offset, &tactile_data.sendId, sizeof(tactile_data.sendId));
//     offset += sizeof(tactile_data.sendId);
//     memcpy(buffer + offset, &tactile_data.PacketIndex, sizeof(tactile_data.PacketIndex));
//     offset += sizeof(tactile_data.PacketIndex);
//     memcpy(buffer + offset, tactile_data.PressureArray, sizeof(uint16_t) * tactile_data.PressureArraySize);
// }

void WiSensToolkit::createBufferEsp(){
    // Copy the data into the buffer
    size_t offset = 0;
    memcpy(buffer + offset, &tactile_data.sendId, sizeof(tactile_data.sendId));
    offset += sizeof(tactile_data.sendId);
    memcpy(buffer + offset, &tactile_data.startIdx, sizeof(tactile_data.startIdx));
    offset += sizeof(tactile_data.startIdx);
    memcpy(buffer + offset, tactile_data.PressureArray, sizeof(uint16_t) * tactile_data.PressureArraySize);
    offset += sizeof(uint16_t)*tactile_data.PressureArraySize;
    memcpy(buffer+offset, &packetNumber, sizeof(uint32_t));
    packetNumber +=1;
}

void WiSensToolkit::sendEspNow(){
    //Update buffer with tactile data
    createBufferEsp();
    // Send the buffer
    esp_err_t result = esp_now_send(currentCommConfig->espNowConfig.peerAddress, buffer, bufferSize);
    if (result == ESP_OK) {
        Serial.println("Sent with Success");
    } else {
        Serial.println("Error sending the data");
    }
    delay(4);
}

void WiSensToolkit::sendWiFi(){
    if (tactileClient->connected()){
        //Update buffer with tactile data
        createBufferEsp();
        tactileClient->write(buffer, bufferSize);
    }
    else {
        Serial.println("Client not connected!");
        while (!tactileClient->connect(*currentCommConfig->wifiConfig.host, currentCommConfig->wifiConfig.port)) {
            Serial.println("Reconnecting");
            delay(500);
        }
    }
}

WiSensToolkit::MyServerCallbacks::MyServerCallbacks(bool *_bptr) {
    connection_ptr = _bptr;
    BLEServerCallbacks();
}

void WiSensToolkit::MyServerCallbacks::onConnect(BLEServer* pServer){
    *connection_ptr = true; 
}

void WiSensToolkit::MyServerCallbacks::onDisconnect(BLEServer* pServer){
    *connection_ptr = false; 
}

void WiSensToolkit::sendBluetooth(){
    if (deviceConnected) {
        createBufferEsp();
        pCharacteristic->setValue(buffer, bufferSize);
        pCharacteristic->notify();
        delay(2); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

void WiSensToolkit::sendSerial(){
    createBufferEsp();
    Serial.write((byte*)buffer, bufferSize);
    Serial.write("wr");
}

int WiSensToolkit::calcNumPins(unsigned int n) {
    if (n == 0) return 1; // Special case for zero
    int numBits = 0;
    while (n > 0) {
        n >>= 1;
        numBits++;
    }
    return numBits;
}