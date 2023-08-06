#pragma once

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

#ifdef __cplusplus
extern "C" {
#endif

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

TaskHandle_t receiveSerialTaskHandle, receiveESPNowTaskHandle, sendESPNowTaskHandle, sendSerialTaskHandle, serialDaemonTaskHandle;

typedef struct {
    char* bodySerialized;                 // Serialized JSON
    size_t size;                // Size of serialized JSON
    uint8_t destinationMAC[5];  // MAC address of destination device or NULL for serial
} Message;

typedef void (*messageHandler)(Message* incomingMessage);

esp_now_peer_info_t gatewayInfo;

const char* bridgeName;
const char* deviceName;
const char* networkName;

QueueHandle_t incomingESPNowQueue, outgoingESPNowQueue;
QueueHandle_t incomingSerialQueue, outgoingSerialQueue;


// Callbacks
void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

// Setup functions
esp_err_t setupESPNow (messageHandler handler, const uint8_t *gatewayAddress, bool isGateway);
esp_err_t setupSerial(messageHandler handler, const int txPin, const int rxPin);

// RTOS tasks
void sendESPNowTask(void *pvParameters);
void sendSerialTask(void *pvParameters);
void receiveESPNowTask (void* pvParameters);
void receiveSerialTask(void* pvParameters);
void listenSerialDaemon(void* pvParameters);

// Helpers
bool sendMessageJSON(cJSON *body, uint8_t *destinationMAC);
Message* createMessage(cJSON *body, uint8_t *destinationMAC);

#ifdef __cplusplus
}
#endif
