#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"


void onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    const uint8_t *mac_addr = esp_now_info->src_addr; // get MAC address if needed

    // Now you can use data and data_len as before
    Serial.write(data, data_len);
    Serial.write("wr");
}

void printMacAddress(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++)
    {
        if (i > 0) Serial.print(":");
        Serial.printf("%02X", mac[i]);
    }
    Serial.println();
}

void setup()
{
    Serial.begin(250000);  // Match your PC-side baud rate
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // Example: channel 1
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
}

void loop()
{
    // Nothing to do — data is handled in callback
    // Serial.println("Heelo");
}
