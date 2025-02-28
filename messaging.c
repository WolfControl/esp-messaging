#include "messaging.h"

/*---------- ESPNow Interrupts: Posting to queues ----------*/

void OnESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    static const char *TAG = "OnESPNowSend";

    ESP_LOGD(TAG, "Last Packet Send Status: %d", status == ESP_NOW_SEND_SUCCESS);
}

void OnESPNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) 
{
    static const char *TAG = "OnESPNowRecv";

    const uint8_t *mac_addr = recv_info->src_addr;  // Extract MAC address from the struct

    ESP_LOGI(TAG, "Received %d bytes from %02x:%02x:%02x:%02x:%02x:%02x", len, 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    ESP_LOGD(TAG, "Allocating memory for incoming data...");
    uint8_t* incomingDataCopy = (uint8_t*)calloc(len, sizeof(uint8_t));
    if (incomingDataCopy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for data, dropping packet");
        return;
    }

    ESP_LOGD(TAG, "Copying incoming data to allocated memory...");
    memcpy(incomingDataCopy, incomingData, len);

    ESP_LOGD(TAG, "Posting pointer to incomingESPNowQueue...");
    if (xQueueSend(incomingESPNowQueue, &incomingDataCopy, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send packet to incomingESPNowQueue queue");
        free(incomingDataCopy);
    }
}



/*---------- Setup Functions ----------*/

esp_err_t setupESPNow (jsonHandler jsonhandler)
{
    static const char *TAG = "setupESPNow";
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "Initializing tcpip adapter...");
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize tcpip adapter");
        return ret;
    }

    // Confirm if this can be removed
    //ESP_LOGD(TAG, "Initializing default station...");
    //esp_netif_create_default_wifi_sta();

    ESP_LOGD(TAG, "Initializing Wi-Fi...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Wi-Fi");
        return ret;
    }

    ESP_LOGD(TAG, "Setting Wi-Fi storage and mode...");
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Wi-Fi storage");
        return ret;
    }

    ESP_LOGD(TAG, "Setting Wi-Fi mode...");
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Wi-Fi mode");
        return ret;
    }

    ESP_LOGD(TAG, "Starting Wi-Fi...");
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi");
        return ret;
    }

    ESP_LOGD(TAG, "Initializing ESP-NOW...");
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ESP-NOW");
        return ret;
    }
    
    ESP_LOGD(TAG, "Registering send callback for ESP-NOW...");
    ret = esp_now_register_send_cb(OnESPNowSend);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback");
        return ret;
    }

    ESP_LOGD(TAG, "Creating Broadcast Peer...");
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(broadcastPeer.peer_addr, broadcastAddress, ESP_NOW_ETH_ALEN);
    broadcastPeer.channel = 0;
    broadcastPeer.encrypt = false;

    ESP_LOGD(TAG, "Adding Broadcast Peer...");
    ret = esp_now_add_peer(&broadcastPeer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer");
        return ret;
    }

    ESP_LOGD(TAG, "Registering receive callback for ESP-NOW...");
    ret = esp_now_register_recv_cb(OnESPNowRecv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback");
        return ret;
    }

    ESP_LOGD(TAG, "Creating incomingESPNowQueue...");
    incomingESPNowQueue = xQueueCreate(10, sizeof(cJSON*));
    if (incomingESPNowQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create incomingESPNowQueue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Creating outgoingESPNowQueue...");
    outgoingESPNowQueue = xQueueCreate(10, sizeof(ESPNowMessage));
    if (outgoingESPNowQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create outgoingESPNowQueue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Starting receive task with abstract handler...");
    if (pdPASS != xTaskCreate(receiveESPNowTask, "listenESPNow_task", TASK_STACK_SIZE, jsonhandler, TASK_PRIORITY, &receiveESPNowTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create listener task");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Starting sender task...");
    if (pdPASS != xTaskCreate(sendESPNowTask, "sendESPNow_task", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &sendESPNowTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create sender task");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "ESPNow setup complete");
    return ESP_OK;
}

esp_err_t setupSerial(jsonHandler jsonhandler, binaryHandler binaryhandler, const int txPin, const int rxPin) {
    const char*TAG = "setupSerial";

    ESP_LOGD(TAG, "Setting UART parameters...");
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUMBER, &uart_config);

    ESP_LOGD(TAG, "Setting UART pins...");
    uart_set_pin(UART_NUMBER, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGD(TAG, "Installing UART driver...");
    uart_driver_install(UART_NUMBER, BUF_SIZE, BUF_SIZE, UART_QUEUE_SIZE, NULL, 0);

    ESP_LOGD(TAG, "Assigning UART driver to vfs...");
    esp_vfs_dev_uart_use_driver(UART_NUMBER);

    // setup message queues
    ESP_LOGD(TAG, "Creating incomingSerialQueue...");
    incomingSerialQueue = xQueueCreate(10, sizeof(SerialMessage));
    if (incomingSerialQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create incomingSerialQueue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Creating outgoingSerialQueue...");
    outgoingSerialQueue = xQueueCreate(10, sizeof(SerialMessage));
    if (outgoingSerialQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create outgoingSerialQueue");
        return ESP_FAIL;
    }    

    // setup tasks
    ESP_LOGD(TAG, "Creating listener daemon task...");
    if (pdPASS != xTaskCreate(listenSerialDaemon, "listenSerialDaemon_task", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &serialDaemonTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create task: listenSerialDaemon_task");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Starting receive task with JSON and Binary handlers...");
    // Allocate handlers struct on heap
    struct {
        jsonHandler jsonHandler;
        binaryHandler binHandler;
    } *handlers = malloc(sizeof(*handlers));

    if (handlers == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for handlers");
        return ESP_FAIL;
    }

    // Assign values
    handlers->jsonHandler = jsonhandler;
    handlers->binHandler = binaryhandler;

    if (pdPASS != xTaskCreate(receiveSerialTask, "receiveSerial_task", TASK_STACK_SIZE, handlers, TASK_PRIORITY, &receiveSerialTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create task: receiveSerial_task");
        free(handlers);  // Cleanup if task creation fails
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Starting sender task...");
    if (pdPASS != xTaskCreate(sendSerialTask, "sendSerial_task", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &sendSerialTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create task: sendSerial_task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*---------- RTOS Tasks ----------*/ 

void sendESPNowTask(void *pvParameters)
{
    static const char *TAG = "sendESPNowTask";
    ESPNowMessage outgoingMessage;

    while(1){
        // Receive the Message struct from the queue
        if (xQueueReceive(outgoingESPNowQueue, &outgoingMessage, portMAX_DELAY) == pdTRUE)
        {

            ESP_LOGD(TAG, "Received message from outgoingESPNowQueue: %s", outgoingMessage.bodyserialized);
            ESP_LOGD(TAG, "Sending to %02x:%02x:%02x:%02x:%02x:%02x", outgoingMessage.destinationMAC[0], outgoingMessage.destinationMAC[1], outgoingMessage.destinationMAC[2], outgoingMessage.destinationMAC[3], outgoingMessage.destinationMAC[4], outgoingMessage.destinationMAC[5]);

            int len = strlen(outgoingMessage.bodyserialized) + 1;

            esp_err_t result = esp_now_send(outgoingMessage.destinationMAC, (uint8_t *) outgoingMessage.bodyserialized, len); 

            if (result == ESP_OK) {
                ESP_LOGD(TAG, "Published packet to ESP-NOW: %s", outgoingMessage.bodyserialized);
            }
            else {
                ESP_LOGE(TAG, "Error: %s", esp_err_to_name(result));
            }

            ESP_LOGD(TAG, "Freeing memory...");
            free(outgoingMessage.bodyserialized);    
        }
    }
}

void sendSerialTask(void *pvParameters) {
    static const char *TAG = "sendSerialTask";
    SerialMessage *msg;

    while (1) {
        if (xQueueReceive(outgoingSerialQueue, &msg, portMAX_DELAY) == pdTRUE) {
            size_t totalSize = offsetof(SerialMessage, payload) + msg->length;

            uint8_t *buffer = (uint8_t*) malloc(totalSize);  // Added '*' here
            if (!buffer) {
                ESP_LOGE(TAG, "Failed to allocate buffer for sending");
                free(msg);
                continue;
            }

            // Serialize
            memcpy(buffer, &msg->type, sizeof(msg->type));
            memcpy(buffer + sizeof(msg->type), &msg->length, sizeof(msg->length));
            memcpy(buffer + offsetof(SerialMessage, payload), msg->payload, msg->length);

            uart_write_bytes(UART_NUMBER, buffer, totalSize);

            ESP_LOGI(TAG, "Packet sent to UART. Type: %u, Size: %lu bytes", msg->type, msg->length);

            free(buffer);
            free(msg);
        }
    }
}


void receiveESPNowTask (void* pvParameters)
{
    static const char *TAG = "receiveESPNowTask";
    jsonHandler handler = (jsonHandler)pvParameters;
    uint8_t* incomingData;
    const char* errorPtr;

    while (1) {
        ESP_LOGD(TAG, "Waiting for incoming data...");
        if (xQueueReceive(incomingESPNowQueue, &incomingData, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received message from incomingESPNowQueue: %s", incomingData);

            ESP_LOGD(TAG, "Parsing JSON...");
            cJSON* incomingJSON = cJSON_ParseWithOpts((char*) incomingData, &errorPtr, 0);

            if (incomingJSON == NULL) {
                ESP_LOGE(TAG, "Failed to parse incoming JSON: Error at %s", errorPtr);
                continue;
            }

            ESP_LOGI(TAG, "Passing data to abstract handler...");
            handler(incomingJSON);

            ESP_LOGD(TAG, "Freeing memory...");
            free(incomingData);
        }
    }
}

void receiveSerialTask(void* pvParameters) {
    static const char* TAG = "receiveSerialTask";
    
    // Extract handlers from parameter struct
    struct {
        jsonHandler jsonHandler;
        binaryHandler binHandler;
    } *handlers = pvParameters;

    SerialMessage* msg;
    ESP_LOGD(TAG, "Waiting for incoming data...");

    while (1) {
        if (xQueueReceive(incomingSerialQueue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGW(TAG, "Received message. Type: %u, Length: %lu", msg->type, msg->length);

            if (msg->type == 0x01 && handlers->jsonHandler) {  // JSON Message
                ESP_LOGD(TAG, "Parsing JSON...");
                const char* errorPtr;
                cJSON* incomingJSON = cJSON_ParseWithOpts((char*)msg->payload, &errorPtr, 0);

                if (!incomingJSON) {
                    ESP_LOGE(TAG, "Failed to parse incoming JSON: Error at %s", errorPtr);
                } else {
                    ESP_LOGI(TAG, "Passing data to JSON handler...");
                    handlers->jsonHandler(incomingJSON);
                    cJSON_Delete(incomingJSON);
                }
            } 
            else if (msg->type == 0x02 && handlers->binHandler) {  // Binary Message
                ESP_LOGI(TAG, "Passing binary data to handler...");
                handlers->binHandler(msg->payload, msg->length);
            } 
            else {
                ESP_LOGE(TAG, "Unknown or unhandled message type: %u", msg->type);
            }

            ESP_LOGD(TAG, "Freeing memory...");
            free(msg);
        }
    }
}


void listenSerialDaemon(void* pvParameters) {
    static const char* TAG = "listenSerialDaemon";
    ESP_LOGI(TAG, "Listening for incoming serial data...");
    
    // Buffer for header only
    uint8_t headerBuffer[sizeof(SerialMessage)];  // Just the fixed part of SerialMessage
    
    // Main states
    enum ReadState {
        WAITING_FOR_HEADER,
        READING_PAYLOAD
    } state = WAITING_FOR_HEADER;
    
    // Message tracking variables
    SerialMessage* currentMsg = NULL;
    uint32_t bytesRead = 0;
    uint32_t payloadLength = 0;
    uint8_t msgType = 0;
    
    // Initialize
    uart_flush_input(UART_NUMBER);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while (1) {
        size_t available;
        uart_get_buffered_data_len(UART_NUMBER, &available);
        
        if (available > 0) {
            if (state == WAITING_FOR_HEADER) {
                // Read complete header
                if (available >= sizeof(SerialMessage)) {
                    int readHeader = uart_read_bytes(UART_NUMBER, headerBuffer, 
                                                  sizeof(SerialMessage), 
                                                  UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
                    
                    if (readHeader == sizeof(SerialMessage)) {
                        // Extract header fields
                        msgType = headerBuffer[0];
                        payloadLength = *((uint32_t*)&headerBuffer[1]);
                        
                        ESP_LOGI(TAG, "Received Header - Type: %u, Length: %lu", msgType, payloadLength);
                        
                        // Simple sanity checks
                        if ((msgType != 0x01 && msgType != 0x02) || 
                            payloadLength > BUF_SIZE) {
                            
                            ESP_LOGE(TAG, "Invalid header (type=%u, length=%lu), resetting", 
                                     msgType, payloadLength);
                            uart_flush_input(UART_NUMBER);
                            continue;
                        }
                        
                        // Allocate message with extra byte for JSON null terminator if needed
                        size_t allocSize = sizeof(SerialMessage) + payloadLength + 
                                         (msgType == 0x01 ? 1 : 0);
                        
                        currentMsg = (SerialMessage*) malloc(allocSize);
                        if (!currentMsg) {
                            ESP_LOGE(TAG, "Memory allocation failed");
                            uart_flush_input(UART_NUMBER);
                            continue;
                        }
                        
                        // Copy header information
                        currentMsg->type = msgType;
                        currentMsg->length = payloadLength;
                        bytesRead = 0;
                        
                        state = READING_PAYLOAD;
                    } else {
                        ESP_LOGE(TAG, "Failed to read complete header, read %d bytes", readHeader);
                        uart_flush_input(UART_NUMBER);
                    }
                }
            }
            else if (state == READING_PAYLOAD) {
                // Read payload in smaller chunks (more robust)
                uint32_t readSize = (available < 256) ? available : 256;
                
                // Don't read more than needed
                if (readSize > (payloadLength - bytesRead)) {
                    readSize = payloadLength - bytesRead;
                }
                
                if (readSize > 0) {
                    int readBytes = uart_read_bytes(UART_NUMBER, 
                                                 &currentMsg->payload[bytesRead], 
                                                 readSize,
                                                 UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
                    
                    if (readBytes > 0) {
                        bytesRead += readBytes;
                        ESP_LOGD(TAG, "Read %d payload bytes, total %lu/%lu", 
                                readBytes, bytesRead, payloadLength);
                    }
                    
                    // Check for completion
                    if (bytesRead >= payloadLength) {
                        // Add null terminator for JSON
                        if (msgType == 0x01) {
                            currentMsg->payload[payloadLength] = '\0';
                        }
                        
                        ESP_LOGI(TAG, "Payload complete, dispatching message");
                        
                        // Send to queue for processing
                        SerialMessage* msgToSend = currentMsg;
                        if (xQueueSend(incomingSerialQueue, &msgToSend, pdMS_TO_TICKS(100)) != pdTRUE) {
                            ESP_LOGE(TAG, "Failed to send to queue");
                            free(currentMsg);
                        }
                        
                        // Reset for next message
                        currentMsg = NULL;
                        bytesRead = 0;
                        state = WAITING_FOR_HEADER;
                    }
                }
                
                // Handle stalled reads
                static TickType_t lastProgress = 0;
                static uint32_t lastBytesRead = 0;
                
                if (bytesRead == lastBytesRead) {
                    if (lastProgress == 0) {
                        lastProgress = xTaskGetTickCount();
                    }
                    else if ((xTaskGetTickCount() - lastProgress) > pdMS_TO_TICKS(3000)) {
                        ESP_LOGE(TAG, "Read stalled at %lu/%lu bytes, abandoning", 
                                bytesRead, payloadLength);
                        free(currentMsg);
                        currentMsg = NULL;
                        bytesRead = 0;
                        state = WAITING_FOR_HEADER;
                        lastProgress = 0;
                        uart_flush_input(UART_NUMBER);
                    }
                }
                else {
                    lastBytesRead = bytesRead;
                    lastProgress = xTaskGetTickCount();
                }
            }
        }
        
        // Prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/*---------- Messaging Functions ----------*/

esp_err_t sendMessageSerial(cJSON* body)
{
    static const char* TAG = "sendMessageSerial";
    char* bodySerialized;

    ESP_LOGD(TAG, "Serializing JSON...");
    bodySerialized = cJSON_PrintUnformatted(body);
    if (bodySerialized == NULL) {
        ESP_LOGE(TAG, "Failed to serialize JSON");
        cJSON_Delete(body);
        return ESP_FAIL;
    }

    // Calculate message size
    size_t bodyLength = strlen(bodySerialized);
    size_t msgSize = sizeof(SerialMessage) + bodyLength + 1; // +1 for null terminator

    // Allocate memory for the SerialMessage struct
    SerialMessage* msg = (SerialMessage*) malloc(msgSize);
    if (msg == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for SerialMessage");
        free(bodySerialized);
        return ESP_FAIL;
    }

    // Populate the struct
    msg->type = 0x01;  // JSON type
    msg->length = bodyLength;
    memcpy(msg->payload, bodySerialized, bodyLength + 1); // Copy JSON data

    ESP_LOGD(TAG, "Posting to outgoingSerialQueue...");
    if (xQueueSend(outgoingSerialQueue, &msg, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send message to outgoingSerialQueue");
        free(msg);
        free(bodySerialized);
        return ESP_FAIL;
    }

    // Clean up
    free(bodySerialized);
    return ESP_OK;
}

esp_err_t sendMessageESPNow(cJSON* body, const uint8_t* destinationMAC)
{
    static const char* TAG = "sendMessageESPNow";
    ESPNowMessage outgoingMessage;

    ESP_LOGD(TAG, "Creating ESPNowMessage...");
    outgoingMessage.bodyserialized = cJSON_PrintUnformatted(body);

    ESP_LOGD(TAG, "Copying destination MAC address to ESPNowMessage struct...");
    memcpy(outgoingMessage.destinationMAC, destinationMAC, ESP_NOW_ETH_ALEN);

    ESP_LOGD(TAG, "Posting to outgoingESPNowQueue...");
    if (xQueueSend(outgoingESPNowQueue, &outgoingMessage, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send struct to outgoingESPNowQueue queue");
        cJSON_Delete(body);
        return ESP_FAIL;
    }

    cJSON_Delete(body);
    return ESP_OK;
}


// EoF
