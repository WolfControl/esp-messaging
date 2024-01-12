# Messaging Library for ESP32/ESP8266 Devices

## Introduction

ESP-messaging is a library for sending and receiving messages between ESP32/ESP8266 devices using ESP-NOW or UART. It allows a user to send JSON messages between devices, abstracting the underlying configuration, communication, and parsing of messages.

## Features

- Separate RTOS tasks for send/receive
- Message queueing for inbound/outbound messages
- Automatic configuration of ESP-NOW and UART
- Automatic parsing of incoming messages
- Pass messages to user-defined callback functions
- Use one or both communication methods at the same time

## Installation

ESP-IDF: Add the header and source files to your `components` folder.

PlatformIO: Add the url to the repository to your `lib_deps` in `platformio.ini`.

## Usage

See `messaging.h` for a full list of functions and their documentation.

```c
#include "esp-messaging.h"

void myMessageHandler(cJSON* incomingMessage) {
    // Handle incoming message
    printf("Incoming message: %s\n", cJSON_Print(incomingMessage));
}

void app_main() {
    // Setup messaging for ESP-NOW
    setupESPNow(myMessageHandler);

    // Setup messaging for UART on GPIO17 (TX) and GPIO16 (RX)
    setupSerial(myMessageHandler, 17, 16);

    // Create message
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "message", "Hello World!");

    // Send message to all devices
    sendMessageESPNow(message, "ff:ff:ff:ff:ff:ff");
}
```

## Requirements

- ESP-IDF + FreeRTOS
- cJSON
- At least 2 ESP32/ESP8266 devices
