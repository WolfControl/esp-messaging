#include "messaging.h"

/*---------- ESPNow Interrupts: Posting to queues ----------*/

void OnESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    static const char *TAG = "OnESPNowSend";

    ESP_LOGD(TAG, "Last Packet Send Status: %d", status == ESP_NOW_SEND_SUCCESS);
}

void OnESPNowRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
    static const char *TAG = "OnESPNowRecv";

    ESP_LOGI(TAG, "Received %d bytes from %02x:%02x:%02x:%02x:%02x:%02x", len, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    ESP_LOGD(TAG, "Allocating memory for incoming data...");
    // use calloc to zero out memory
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

esp_err_t setupESPNow (messageHandler handler)
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
    if (pdPASS != xTaskCreate(receiveESPNowTask, "listenESPNow_task", TASK_STACK_SIZE, handler, TASK_PRIORITY, &receiveESPNowTaskHandle)) {
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

esp_err_t setupSerial(messageHandler handler, const int txPin, const int rxPin) {
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
    incomingSerialQueue = xQueueCreate(10, sizeof(cJSON*));
    if (incomingSerialQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create incomingSerialQueue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Creating outgoingSerialQueue...");
    outgoingSerialQueue = xQueueCreate(10, sizeof(char*));
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

    ESP_LOGD(TAG, "Starting receive task with abstract handler...");
    if (pdPASS != xTaskCreate(receiveSerialTask, "receiveSerial_task", TASK_STACK_SIZE, (void *) handler, TASK_PRIORITY, &receiveSerialTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create task: receiveSerial_task");
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

void sendSerialTask(void *pvParameters)
{
    static const char *TAG = "sendSerialTask";
    char* bodySerialized;

    while(1) {
        ESP_LOGD(TAG, "Waiting for outgoing message...");

        if (xQueueReceive(outgoingSerialQueue, &bodySerialized, portMAX_DELAY) == pdTRUE)
        {

            ESP_LOGD(TAG, "Received message from outgoingSerialQueue: %s", bodySerialized);
            int len = strlen(bodySerialized) + 1;

            if (len > BUF_SIZE) {
                ESP_LOGE(TAG, "Serialized JSON string exceeds buffer size");
                free(bodySerialized);
                continue;
            }

            ESP_LOGI(TAG, "Sending packet to UART...");
            uart_write_bytes(UART_NUMBER, bodySerialized, len);
            //uart_write_bytes(UART_NUMBER, &newline, 1);

            ESP_LOGD(TAG, "Freeing memory...");
            free(bodySerialized);    
        }
    }
}

void receiveESPNowTask (void* pvParameters)
{
    static const char *TAG = "receiveESPNowTask";
    messageHandler handler = (messageHandler)pvParameters;
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

void receiveSerialTask(void* pvParameters)
{
    static const char* TAG = "receiveSerialTask";
    messageHandler handler = (messageHandler)pvParameters;
    char* incomingData;
    const char* errorPtr;

    ESP_LOGD(TAG, "Waiting for incoming data...");

// test
    while (1) {
        if (xQueueReceive(incomingSerialQueue, &incomingData, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received data from incomingSerialQueue: %s", incomingData);

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

void listenSerialDaemon(void* pvParameters)
{
    static const char* TAG = "listenSerialDaemon";

    ESP_LOGI(TAG, "Waiting for incoming data...");

    while (1) {
        size_t len = 0;
        uart_get_buffered_data_len(UART_NUMBER, &len);

        if (len > 0) {
            ESP_LOGI(TAG, "Reading %d bytes from UART...", len);
            uint8_t incomingData[BUF_SIZE];

            // Read bytes into incomingData without reserving space for null terminator
            int received_msg_length = uart_read_bytes(UART_NUMBER, incomingData, BUF_SIZE, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
            
            if (received_msg_length > 0) {
                // Allocate memory to copy received binary data
                uint8_t* incomingDataCopy = (uint8_t*)malloc(received_msg_length * sizeof(uint8_t));
                if (incomingDataCopy == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for data, dropping packet");
                    uart_flush(UART_NUMBER);
                    continue;
                }

                // Copy the received binary data to dynamically allocated memory
                memcpy(incomingDataCopy, incomingData, received_msg_length);

                ESP_LOGI(TAG, "Received %d bytes of binary data.", received_msg_length);

                // Send the binary data to the queue
                if (xQueueSend(incomingSerialQueue, &incomingDataCopy, 0) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send packet to incomingSerial queue");
                    free(incomingDataCopy);
                }

            } else if (received_msg_length == BUF_SIZE) {
                ESP_LOGE(TAG, "Received message too long for buffer: %d bytes", received_msg_length);
                uart_flush(UART_NUMBER);
            } else if (received_msg_length == -1) {
                ESP_LOGE(TAG, "Internal error in UART driver");
                uart_flush(UART_NUMBER);
            } else {
                ESP_LOGE(TAG, "Received incomplete or malformed message");
                uart_flush(UART_NUMBER);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);  // delay to allow other tasks to run
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
        free(bodySerialized);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Posting to outgoingSerialQueue...");
    if (xQueueSend(outgoingSerialQueue, &bodySerialized, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send struct to outgoingSerialQueue queue");
        cJSON_Delete(body);
        free(bodySerialized);
        return ESP_FAIL;
    }

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
