/*
  WiSensToolkit.h - Library for wireless sensing using tactile snenso
  Created by Devin Murphy, May 21, 2024
*/
#ifndef WiSensToolkit_h
#define WiSensToolkit_h

#include <Arduino.h>

#ifdef ESP32
    #include <esp_now.h>
    #include <driver/adc.h>
    #include <WiFi.h>
    #include <NimBLEDevice.h>
    #include <DS_MCP4018.h>
#endif

    

// Define Bluetooth UUID
#define SERVICE_UUID        "dc643f2d-a2f8-4a00-a964-e138b4d82fdb"
#define CHARACTERISTIC_UUID "1766324e-8b30-4d23-bff2-e5209c3d986f"  

struct struct_message {
    int8_t sendId;
    //int8_t PacketIndex;
    uint16_t startIdx;
    uint16_t* PressureArray;
    size_t PressureArraySize;

    // Constructor
    struct_message(size_t size)
        : PressureArraySize(size), PressureArray(new uint16_t[size]) {}

    // Destructor
    ~struct_message() {
        delete[] PressureArray;
    }

    // Disable copy constructor and copy assignment operator
    struct_message(const struct_message&) = delete;
    struct_message& operator=(const struct_message&) = delete;

    // Move constructor
    struct_message(struct_message&& other) noexcept
        : sendId(other.sendId), startIdx(other.startIdx),
          PressureArray(other.PressureArray), PressureArraySize(other.PressureArraySize) {
        other.PressureArray = nullptr;
    }

    // Move assignment operator
    struct_message& operator=(struct_message&& other) noexcept {
        if (this != &other) {
            delete[] PressureArray;
            sendId = other.sendId;
            startIdx = other.startIdx;
            PressureArray = other.PressureArray;
            PressureArraySize = other.PressureArraySize;
            other.PressureArray = nullptr;
        }
        return *this;
    }
};

typedef enum {WIFI, BLUETOOTH, ESPNOW, SERIALCOMM} CommProtocol;

typedef struct {
    const char* ssid;
    const char* password;
    IPAddress* host;
    uint16_t port;
} WiFiConfig;

typedef struct {
    const char* deviceName;
} BluetoothConfig;

typedef struct {
    uint8_t peerAddress[6];
} EspNowConfig;

typedef union {
    WiFiConfig wifiConfig;
    BluetoothConfig bluetoothConfig;
    EspNowConfig espNowConfig;
} CommConfig;

class WiSensToolkit
{
    public:
        WiSensToolkit(int numRows, int numCols, int* rowPins, int* readPins, int adcPin,  CommProtocol commType, CommConfig* commConfig, int sendId);
        void scanArray();
        struct_message tactile_data;
        bool serial_communication = true;
        void readNode(int row, int col);

    private:
        class MyServerCallbacks : public BLEServerCallbacks {
            bool *connection_ptr = nullptr;
        public:
            MyServerCallbacks(bool *_bptr);
            void onConnect(BLEServer* pServer);
            void onDisconnect(BLEServer* pServer);
        };
        MCP4018* MCP;
        BLEService *pService = nullptr;
        BLECharacteristic *pCharacteristic = nullptr;
        BLEServer *pServer = nullptr;
        BLEAdvertising *pAdvertising = nullptr;
        WiFiClient* tactileClient;
        void sendBluetooth();
        void BluetoothSetup();
        bool deviceConnected = false;
        bool oldDeviceConnected = false;
        bool serial_print = false; //Flag for printing debug messages to serial monitor
        int _numRowPins = 0; //Number of digital pins used for multiplexer
        int _numColPins = 0;
        int _numRows; //Number of rows to read 
        int _numCols; //Number of cols to read
        int* _rowPins;
        int* _readPins;
        int zInput = A2; //Pin number of ADC
        uint32_t packetNumber = 0;
        uint8_t packetCount = 0;
        
        uint8_t* buffer;
        size_t bufferSize;
        CommProtocol currentCommType;
        CommConfig* currentCommConfig;
        void WiFiSetup();
        void sendWiFi();
        void initCommProtocol(CommProtocol commType, CommConfig* commConfig, int sendId);
        void EspNowSetup();
        void sendEspNow();
        void ADCSetup();
        void selectMuxPin(byte pin, bool row);
        void sendResult();
        void createBuffer();
        void createBufferEsp();    
        
        void sendSerial();
        void initDigiPot(int res);
        int calcNumPins(unsigned int n);
};
#endif