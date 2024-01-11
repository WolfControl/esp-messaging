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
#include "math.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

// macros for serial
#define UART_NUMBER UART_NUM_2
#define BUF_SIZE 256
#define UART_QUEUE_SIZE 0
#define TASK_STACK_SIZE 8192
#define TASK_PRIORITY 5
#define BAUD_RATE 115200
#define LISTENER_TASK_DELAY_MS 10
#define UART_READ_TIMEOUT_MS 100



// Defining the user's message handler function to be used with setupESPNow and setupSerial
// The messageHandler can be any user defined function so long as it takes the message body as a cJSON pointer
typedef void (*messageHandler)(cJSON* incomingMessage);

// Used internally for outgoing ESP-NOW messages
typedef struct {
    char* bodyserialized;
    uint8_t destinationMAC[ESP_NOW_ETH_ALEN];
} ESPNowMessage;

esp_now_peer_info_t broadcastPeer;
TaskHandle_t receiveSerialTaskHandle, receiveESPNowTaskHandle, sendESPNowTaskHandle, sendSerialTaskHandle, serialDaemonTaskHandle;
QueueHandle_t incomingESPNowQueue, outgoingESPNowQueue, incomingSerialQueue, outgoingSerialQueue;


/*--------------------------------------*/
/*---------- Callbacks & ISRs ----------*/
/*--------------------------------------*/

// Logs ESP-NOW send status
void OnESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status);

// Interrupts, posts incoming messages to incomingESPNowQueue as uint8_t pointer
void OnESPNowRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

/*--------------------------------------*/
/*----------- Setup Functions ----------*/
/*--------------------------------------*/

/**
 * @brief Sets up components necessary for messaging via ESP-NOW.
 * 
 * @param handler User defined function to handle incoming messages. Expects a cJSON pointer.
 * 
 * @return ESP_OK if successful, ESP_FAIL if not.
 * 
 * @note Sets up wifi stack, sets callbacks, creates queues and tasks.
*/
esp_err_t setupESPNow (messageHandler handler);

/**
 * @brief Sets up components necessary for messaging via UART.
 * 
 * @param handler User defined function to handle incoming messages. Expects a cJSON pointer.
 * 
 * @param txPin GPIO pin to use for UART TX.
 * 
 * @param rxPin GPIO pin to use for UART RX.
 * 
 * @return ESP_OK if successful, ESP_FAIL if not.
 * 
 * @note Sets up UART driver, event queues, and tasks.
*/
esp_err_t setupSerial(messageHandler handler, const int txPin, const int rxPin);

/*--------------------------------------*/
/*------------- RTOS Tasks -------------*/
/*--------------------------------------*/

/**
 * @brief RTOS Task that receives outgoing messages from outgoingESPNowQueue and sends them via ESP-NOW.
 * 
 * @note Receives data as a char pointer (JSON is serialized in sendMessageESPNow)
*/
void sendESPNowTask(void *pvParameters);

/**
 * @brief RTOS Task that receives outgoing messages from outgoingSerialQueue and sends them via UART.
 * 
 * @note Receives data as char pointer (JSON is serialized in sendMessageSerial)
*/
void sendSerialTask(void *pvParameters);

/**
 * @brief RTOS Task that receives data from incomingESPNowQueue and passes to user defined handler.
 * 
 * @note receives data as uint8_t pointer and parses to cJSON object
*/
void receiveESPNowTask (void* pvParameters);

/**
 * @brief RTOS Task that receives data from incomingSerialQueue and passes to user defined handler.
 *
 * @note receives data as char pointer and parses to cJSON object
*/
void receiveSerialTask(void* pvParameters);

/**
 * @brief RTOS task that listens for incoming messages on UART and posts them to incomingSerialQueue.
 * 
 * @note Posts data as char pointer
*/
void listenSerialDaemon(void* pvParameters);

/*--------------------------------------*/
/*-------- Messaging Functions ---------*/
/*--------------------------------------*/

/**
 * @brief Re-serializes a cJSON object and posts it to outgoingSerialQueue.
 * 
 * @param body A cJSON object already parsed by receiveSerialTask or receiveESPNowTask.
 * 
 * @return ESP_OK if successful, ESP_FAIL if not.
 */
esp_err_t sendMessageSerial(cJSON* body);

/**
 * @brief Serializes a cJSON object, wraps in ESPNowMessage struct, and posts it to outgoingESPNowQueue.
 * 
 * @param body A cJSON object.
 * 
 * @param destinationMAC The MAC address to send our message to.
 * 
 * @return ESP_OK if successful, ESP_FAIL if not.
 */
esp_err_t sendMessageESPNow(cJSON* body, const uint8_t* destinationMAC);

// Hello

/*--------------------------------------*/
/*---------- Helper Functions ----------*/
/*--------------------------------------*/

/**
 * @brief Helper function for sendLog, sendReadings, and sendCommand
 * 
 * @return cJSON object body with deviceid and timestamp
*/
cJSON* createMessageBody();

/*--------------------------------------*/
/*----------- User Functions -----------*/
/*--------------------------------------*/

/**
 * @brief Sends a log message to the gateway via ESP-NOW.
 *
 * @param log String to send to gateway
 * 
 * @param destinationMAC MAC address to send to
 * 
 * @return ESP_OK if successful, error code as esp_err_t if not.
 *
 * @note adds deviceID and timestamp to JSON body
*/
esp_err_t sendLog(char* log, uint8_t* destinationMAC);

/**
 * @brief Sends an array of sensor readings to the gateway via ESP-NOW.
 * 
 * @param readings Array of sensor readings
 * 
 * @param numReadings Number of readings in array
 * 
 * @param destinationMAC MAC address to send to
 * 
 * @return ESP_OK if successful, error code as esp_err_t if not.
 * 
 * @note adds deviceID and timestamp to JSON body
 * 
*/
esp_err_t sendReadings(float* readings, int numReadings, uint8_t* destinationMAC);

/**
 * Called on MQTT gateway
 * Takes a target deviceId, node, and command, compiles a cJSON object with id, timestamp, and command
 * Sends serialized JSON to the ESP-NOW gateway via UART
*/
esp_err_t sendCommand(char* command, uint8_t* targetDeviceId);

#ifdef __cplusplus
}
#endif
