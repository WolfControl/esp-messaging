#ifndef MESSAGING_H
#define MESSAGING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"

// move somewhere else, they're getting redifined by linker
// macros for serial

#define UART_NUMBER UART_NUM_2
#define BUF_SIZE 256
#define UART_QUEUE_SIZE 0
#define TASK_STACK_SIZE 4096
#define TASK_PRIORITY 5
#define BAUD_RATE 115200
#define LISTENER_TASK_DELAY_MS 10
#define UART_READ_TIMEOUT_MS 100

bool hasSentCONN, hasReceivedCONNACK;

TaskHandle_t handshakeTaskHandle, serialListenerTaskHandle;

typedef void (*messageHandler)(const char* topic, const char* payload);

typedef struct {
    char* message;
    size_t size;
    uint8_t destinationMAC[6];
} Message;

esp_now_peer_info_t gatewayInfo;

const char* bridgeName;
const char* deviceName;
const char* networkName;

QueueHandle_t incomingMessageQueue;
QueueHandle_t outgoingMessageQueue;

// Callbacks
void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowRecvDevice(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowRecvGateway(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

// Setup functions
esp_err_t setupESPNow(const uint8_t *gatewayAddress, bool isGateway);
void setupSerial(messageHandler handler, int txPin, int rxPin);


// RTOS tasks
void sendMessageTask(void *pvParameter);
void listenSerialTask(void* pvParameters);

// Helpers
bool sendMessageJSON(cJSON *message, const uint8_t *destinationMAC);
void sendMessageSerial(const char* topic, const char* payload);

char* parseJsonField(const cJSON* jsonObject, const char* fieldName);
cJSON* parseAndValidateJson(const char* rawData);

#ifdef __cplusplus
}
#endif

#endif