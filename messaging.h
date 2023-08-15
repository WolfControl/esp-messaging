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


// Defining the user's message handler function to be used with setupESPNow and setupSerial
// The messageHandler can be any user defined function so long as it takes the message body as a cJSON pointer
typedef void (*messageHandler)(cJSON* incomingMessage);

const char* networkName;
const char* zoneName;
const char* gatewayName;
const char* deviceName;
const char* nodeName;

esp_now_peer_info_t gatewayInfo;

TaskHandle_t receiveSerialTaskHandle, receiveESPNowTaskHandle, sendESPNowTaskHandle, sendSerialTaskHandle, serialDaemonTaskHandle;

QueueHandle_t incomingESPNowQueue, outgoingESPNowQueue;
QueueHandle_t incomingSerialQueue, outgoingSerialQueue;


/*--------------------------------------*/
/*---------- Callbacks & ISRs ----------*/
/*--------------------------------------*/

// Logs ESP-NOW send status
void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status);

// Logs ESP-NOW send status
void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status);

// Interrupts, posts incoming messages to incomingESPNowQueue as uint8_t pointer
void OnESPNowRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

/*--------------------------------------*/
/*----------- Setup Functions ----------*/
/*--------------------------------------*/

// Sets up ESP-NOW, more description TODO
esp_err_t setupESPNow (messageHandler handler, const uint8_t *gatewayAddress, bool isGateway);

// Sets up UART, more description TODO
esp_err_t setupSerial(messageHandler handler, const int txPin, const int rxPin);

/*--------------------------------------*/
/*------------- RTOS Tasks -------------*/
/*--------------------------------------*/

/**
 * Receive serialized JSON body from outgoingESPNowQueue
 * Sends via ESP-NOW to MAC address injected into message body by sendMessageESPNow
*/
void sendESPNowTask(void *pvParameters);

/**
 * Receive serialized JSON body from outgoingSerialQueue
 * Sends via UART
*/
void sendSerialTask(void *pvParameters);

/**
 * Receive serialized JSON body from incomingESPNowQueue as uint8_t pointer
 * Parses as JSON and passes to handler
*/
void receiveESPNowTask (void* pvParameters);

/**
 * Receive serialized JSON body from incomingSerialQueue as char pointer
 * Parses as JSON and passes to handler
*/
void receiveSerialTask(void* pvParameters);

/**
 * Listens for incoming messages on UART
 * Posts incoming messages to incomingSerialQueue as char pointer
*/
void listenSerialDaemon(void* pvParameters);

/*--------------------------------------*/
/*-------- Messaging Functions ---------*/
/*--------------------------------------*/

/** 
 * To be used in a messageHandler on ESPNow and MQTT gateways
 * Takes a cJSON object already parsed by receiveSerialTask or receiveESPNowTask (or user created in the case of sendCommand)
 * Serializes and posts char array to outgoingSerialQueue
 * Does not check or modify the message in any way
*/
esp_err_t sendMessageSerial(cJSON* body);

/**
 * To be used on devices for sending readings/logs as well as on ESPNow Gateways for sending commands
 * takes a cJSON object and MAC address to send to
 * Adds MAC to json body, serializes to a char array, and posts to outgoingESPNowQueue
 * Is there a better way to do this? Previously used a message struct that had MAC and cJSON pointer but simplifying for now
*/
esp_err_t sendMessageESPNow(cJSON* body, const uint8_t* destinationMAC);

/*--------------------------------------*/
/*---------- Helper Functions ----------*/
/*--------------------------------------*/

/**
 * Helper function for sendLog, sendReadings, and sendCommand
 * returns a cJSON object body with deviceid and timestamp
*/
cJSON* createMessageBody();

/*--------------------------------------*/
/*----------- User Functions -----------*/
/*--------------------------------------*/

/**
 * Called on device
 * Takes a string, compiles a cJSON object with id, timestamp, and log
 * Sends serialized JSON to the gatewayMAC via ESP-NOW
*/
esp_err_t sendLog(char* log);

/**
 * Called on device
 * Takes array of sensor readings and size, compiles a cJSON object with id, timestamp, and readings
 * Sends serialized JSON to the gatewayMAC via ESP-NOW
*/
esp_err_t sendReadings(float* readings, int numReadings);

/**
 * Called on MQTT gateway
 * Takes a target deviceId, node, and command, compiles a cJSON object with id, timestamp, and command
 * Sends serialized JSON to the ESP-NOW gateway via UART
*/
esp_err_t sendCommand(char* command, uint8_t* targetDeviceId);

/*--------------------------------------*/

/*
RTD: 25.0 // deg C
pH: 7.0 // logaritmic scale
DO: 400 // mg/L
EC: 1.2 // mS/cm

JSON structures for messages

Readings
    id: 12345 // Device ID
    t: 1234567890 // Unix timestamp of when reading was taken and sent from device
    r: [25.0, 7.0, 400, 1.2] // readings

Commands
    id: 12345 // ID of device to receive command
    t: 1234567890 // Unix timestamp of when the command was first sent from the MQTT gateway
    c: "PH,cal,mid,7.0" // cmd to pass to EZO method for object "PH"

Logs
    id: 12345 // Device ID
    t: 1234567890 // Unix timestamp of when log was sent from device
    l: "Pump3 ERR" // log message

*/

#ifdef __cplusplus
}
#endif
