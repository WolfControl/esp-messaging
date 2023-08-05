#ifndef MESSAGING_H
#define MESSAGING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_log.h"
#include <string.h>

// TODO: Add destination MAC address to message struct
typedef struct {
    char* message;
    size_t size;
} Message;

/*---------- ESP NOW Related Globals & Stubs ----------*/
esp_now_peer_info_t gatewayInfo;

// This should be provisioned later, hardcoded for now
const char* bridgeName;
const char* deviceName;
const char* networkName;

QueueHandle_t incomingMessageQueue;
QueueHandle_t outgoingMessageQueue;

void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowRecvDevice(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowRecvGateway(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

esp_err_t setupESPNow(const uint8_t *gatewayAddress, bool isGateway);

void sendMessageTask(void *pvParameter);

bool sendMessageJSON(cJSON *message);
void sendMessageString(char* topic, char* payload);
void sendMessageFloat(char* topic, float payload);

void createMessageQueues();

#ifdef __cplusplus
}
#endif

#endif