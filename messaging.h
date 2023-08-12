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

// Struct for passing messages between tasks and devices
typedef struct {
    char* bodySerialized;       // Serialized JSON
    size_t size;                // Size of serialized JSON
    uint8_t destinationMAC[5];  // MAC address of destination device or NULL for serial
} Message;

// Defining the user's message handler function
// setupESPNow and setupSerial both take a messageHandler function pointer
// The messageHandler can be any user defined function so long as it takes a Message* as its only argument
typedef void (*messageHandler)(Message* incomingMessage);

esp_now_peer_info_t gatewayInfo;

const char* bridgeName;
const char* deviceName;
const char* networkName;

QueueHandle_t incomingESPNowQueue, outgoingESPNowQueue;
QueueHandle_t incomingSerialQueue, outgoingSerialQueue;


/*----- Callback Functions -----*/
// Logs ESP-NOW send status
void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status);

// Logs ESP-NOW send status
void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status);

// Interrupts, posts incoming data to incomingESPNowQueue
void OnESPNowRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

/*----- Setup Functions -----*/
// Sets up ESP-NOW, more description TODO
esp_err_t setupESPNow (messageHandler handler, const uint8_t *gatewayAddress, bool isGateway);

// Sets up UART, more description TODO
esp_err_t setupSerial(messageHandler handler, const int txPin, const int rxPin);

/*----- RTOS Tasks -----*/
// Receive message structs from outgoingESPNowQueue and sends serialized JSON body via ESP-NOW
void sendESPNowTask(void *pvParameters);

// Receive message structs from outgoingSerialQueue and sends serialized JSON body via UART
void sendSerialTask(void *pvParameters);

// Picks up incoming raw data from incomingESPNowQueue as uint8_t pointer, parses the message body as JSON, and passes to handler
void receiveESPNowTask (void* pvParameters);

// Picks up raw data from incomingSerialQueue as char pointer, parses to a JSON body, and passes to handler
void receiveSerialTask(void* pvParameters);

// In lieu of ISR, this task posts incoming UART messages directly to the incomingSerialQueue as a void pointer to a char array
void listenSerialDaemon(void* pvParameters);

/*----- Messaging Helpers -----*/
// Takes raw json body and passes to outbound queue dependent on if MAC was provided
bool sendMessageJSON(cJSON *body, uint8_t *destinationMAC);

// Takes raw json body and passes to outgoingSerialQueue
bool sendJSONSerial(cJSON *body);

// Takes raw json body and passes to outgoingESPNowQueue
bool sendJSONESPNow(cJSON *body, uint8_t *destinationMAC);

// Takes topic and payload as strings and passes to outbound queue dependent on if MAC was provided
bool sendMessageTopicPayload(const char* topic, const char* payload, uint8_t *destinationMAC);


/*----- User Functions -----*/

// Takes an array of floats, creates a comma separated string, compiles a cJSON object with id, timestamp, and readings, sends serialized JSON to the gatewayMAC via ESP-NOW 
esp_err_t sendReadings(float* readings, size_t numReadings);

// Takes a string, compiles a cJSON object with id, timestamp, and log, sends serialized to the gatewayMAC via ESP-NOW
esp_err_t sendLog(char* log);

// What does this need to take? Where will I pick apart MQTT topic and payload to find device id and command?
esp_err_t sendCommand(char* command);




/*

RTD: 25.0 // deg C
pH: 7.0 // logaritmic scale
DO: 400 // mg/L
EC: 1.2 // mS/cm

JSON structures for messages

Readings
    Id: 12345 // Device ID
    T: 1234567890 // Unix timestamp of when reading was taken and sent from device
    t: "reading" // Type of message
    r: [25.0, 7.0, 400, 1.2]
    s: ["RTD", "pH", "DO", "EC"]


Commands
    id: 12345 // ID of device to receive command
    T: 1234567890 // Unix timestamp of when the command was first sent from the MQTT gateway
    t: "command" // Type of message
    m: "PH,cal,mid,7.0" // cmd to pass to EZO method

Logs
    id: 12345 // Device ID
    T: 1234567890 // Unix timestamp of when log was sent from device
    t: "log" // Type of message
    m: "Message" // Message

*/

#ifdef __cplusplus
}
#endif
