#include "messaging.h"

/*---------- ESPNow Interrupts: Posting to queues ----------*/

void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    static const char *TAG = "OnESPNowSendDevice";

    ESP_LOGD(TAG, "Last Packet Send Status: %d", status == ESP_NOW_SEND_SUCCESS);
}

void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    static const char *TAG = "OnESPNowSendGateway";

    ESP_LOGD(TAG, "Last Packet Send Status: %d", status == ESP_NOW_SEND_SUCCESS);
}

void OnESPNowRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
    static const char *TAG = "OnESPNowRecv";

    ESP_LOGD(TAG, "Received data from %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Received data: %s", incomingData);

    ESP_LOGD(TAG, "Posting %d bytes to incomingESPNowQueue", len);
    if (xQueueSend(incomingESPNowQueue, (void *) incomingData, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send packet to incomingESPNowQueue queue");
    }
}

/*---------- Setup Functions ----------*/

esp_err_t setupESPNow (messageHandler handler, const uint8_t *gatewayAddress, bool isGateway)
{
    static const char *TAG = "setupESPNow";
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "Initializing tcpip adapter...");
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize tcpip adapter");
        return ret;
    }

    if(!isGateway) {
        ESP_LOGD(TAG, "Initializing default station...");
        esp_netif_create_default_wifi_sta();
    }

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

    // Is this needed?
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // think about how mac needs set on devices for future
    if(isGateway) {
        ESP_LOGD(TAG, "Setting MAC address...");
        ret = esp_wifi_set_mac(ESP_IF_WIFI_STA, gatewayAddress);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set MAC address");
            return ret;
        }
    }

    ESP_LOGD(TAG, "Initializing ESP-NOW...");
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ESP-NOW");
        return ret;
    }
    
    ESP_LOGD(TAG, "Registering send callback for ESP-NOW...");
    if (isGateway) {
        ret = esp_now_register_send_cb(OnESPNowSendGateway);
    } else {
        ret = esp_now_register_send_cb(OnESPNowSendDevice);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback");
        return ret;
    }

    // how will gateway handle peers?
    if(!isGateway) {
        ESP_LOGD(TAG, "Creating peer for gateway...");
        memcpy(gatewayInfo.peer_addr, gatewayAddress, ESP_NOW_ETH_ALEN);
        gatewayInfo.channel = 0;
        gatewayInfo.encrypt = false;

        ret = esp_now_add_peer(&gatewayInfo);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add peer");
            return ret;
        }
    }

    ESP_LOGD(TAG, "Registering receive callback for ESP-NOW...");
    ret = esp_now_register_recv_cb(OnESPNowRecv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback");
        return ret;
    }

    // setup message queues
    ESP_LOGD(TAG, "Creating incomingESPNowQueue...");
    incomingESPNowQueue = xQueueCreate(10, sizeof(cJSON*));
    if (incomingESPNowQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create incomingESPNowQueue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Creating outgoingESPNowQueue...");
    outgoingESPNowQueue = xQueueCreate(10, sizeof(ESPNowMessage*));
    if (outgoingESPNowQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create outgoingESPNowQueue");
        return ESP_FAIL;
    }

    // setup tasks
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
    outgoingSerialQueue = xQueueCreate(10, sizeof(cJSON*));
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
            ESP_LOGI(TAG, "Received message from outgoingESPNowQueue: %s", outgoingMessage.bodyserialized);

            int len = strlen(outgoingMessage.bodyserialized) + 1;

            esp_err_t result = esp_now_send(outgoingMessage.destinationMAC, (uint8_t *) outgoingMessage.bodyserialized, len); 

            if (result == ESP_OK) {
                ESP_LOGI(TAG, "Published packet to ESP-NOW: %s", outgoingMessage.bodyserialized);
            }
            else {
                ESP_LOGE(TAG, "Error: %s", esp_err_to_name(result));
            }

            // Free the packet string
            free(outgoingMessage.bodyserialized);    
        }
    }
}

void sendSerialTask(void *pvParameters)
{
    static const char *TAG = "sendSerialTask";
    char* outgoingData;
    static const char newline = '\0';
    int len;

    while(1) {
        ESP_LOGD(TAG, "Waiting for outgoing message...");

        if (xQueueReceive(outgoingSerialQueue, &outgoingData, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received message from outgoingSerialQueue: %s", outgoingData);
            len = strlen(outgoingData) + 1;

            if (len > BUF_SIZE) {
                ESP_LOGE(TAG, "Serialized JSON string exceeds buffer size");
                free(outgoingData);
                continue;
            }

            ESP_LOGD(TAG, "Sending packet to UART...");
            uart_write_bytes(UART_NUMBER, outgoingData, len);
            uart_write_bytes(UART_NUMBER, &newline, 1);

            // Free the packet string
            free(outgoingData);    
        }
    }
}

void receiveESPNowTask (void* pvParameters)
{
    static const char *TAG = "receiveESPNowTask";
    messageHandler handler = (messageHandler)pvParameters;
    uint8_t* incomingData;
    cJSON* incomingJSON;

    while (1) {
        ESP_LOGD(TAG, "Waiting for incoming data...");
        if (xQueueReceive(incomingESPNowQueue, &incomingData, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Received message from incomingESPNowQueue: %s", incomingData);

            ESP_LOGD(TAG, "Parsing JSON...");
            char* incomingDataChar = (char*)incomingData;
            incomingJSON = cJSON_Parse(incomingDataChar);

            ESP_LOGD(TAG, "Passing data to handler...");
            // Gateway handler: forward over serial
            // Device handler: process incoming command
        }
    }

}

void receiveSerialTask(void* pvParameters)
{
    static const char* TAG = "receiveSerialTask";
    messageHandler handler = (messageHandler)pvParameters;
    char* incomingData;
    cJSON* incomingJSON;

    while (1) {
        ESP_LOGD(TAG, "Waiting for incoming data...");
        if (xQueueReceive(incomingSerialQueue, &incomingData, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received data from incomingSerialQueue: %s", incomingData);

            ESP_LOGD(TAG, "Parsing JSON...");
            incomingJSON = cJSON_Parse(incomingData);

            ESP_LOGD(TAG, "Passing data to handler...");
        }
    }
}

void listenSerialDaemon(void* pvParameters)
{
    static const char* TAG = "listenSerialDaemon";

    while (1) {
        ESP_LOGD(TAG, "Waiting for incoming data...");        
        size_t len = 0;
        uart_get_buffered_data_len(UART_NUMBER, &len);

        if (len > 0) {
            ESP_LOGD(TAG, "Reading %d bytes from UART...", len);
            char incomingData[BUF_SIZE];
            int received_msg_length = uart_read_bytes(UART_NUMBER, (uint8_t*)incomingData, BUF_SIZE - 1, UART_READ_TIMEOUT_MS / portTICK_RATE_MS); // leaving 1 byte for null terminator
            
            ESP_LOGD(TAG, "Null terminating string...");
            incomingData[received_msg_length] = '\0';

            if (received_msg_length > 0 && incomingData[received_msg_length - 1] == '\0') {

                ESP_LOGD(TAG, "Posting to incomingSerialQueue...");
                if (xQueueSend(incomingSerialQueue, (void *) incomingData, 0) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send packet to incomingSerial queue");
                }

            } else if (received_msg_length == BUF_SIZE - 1) {
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
    char* outgoingData;
    int len;

    ESP_LOGD(TAG, "Serializing JSON...");
    outgoingData = cJSON_PrintUnformatted(body);

    ESP_LOGD(TAG, "Posting to outgoingSerialQueue...");
    if (xQueueSend(outgoingSerialQueue, &outgoingData, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send packet to outgoingSerialQueue queue");
        free(outgoingData);
        cJSON_Delete(body);
        return ESP_FAIL;
    }

    cJSON_Delete(body);
    return ESP_OK;

}

esp_err_t sendMessageESPNow(cJSON* body, const uint8_t* destinationMAC)
{
    static const char* TAG = "sendMessageESPNow";
    char* bodyserialized;
    ESPNowMessage outgoingMessage;

    ESP_LOGD(TAG, "Serializing JSON...");
    bodyserialized = cJSON_PrintUnformatted(body);

    ESP_LOGD(TAG, "Creating ESPNowMessage...");
    outgoingMessage.bodyserialized = bodyserialized;
    memcpy(outgoingMessage.destinationMAC, destinationMAC, ESP_NOW_ETH_ALEN);

    ESP_LOGD(TAG, "Posting to outgoingESPNowQueue...");
    if (xQueueSend(outgoingESPNowQueue, &outgoingMessage, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send struct to outgoingESPNowQueue queue");
        free(bodyserialized);
        cJSON_Delete(body);
        return ESP_FAIL;
    }

    cJSON_Delete(body);
    return ESP_OK;

}

/*---------- Helper Functions ----------*/

cJSON* createMessageBody()
{
    cJSON* body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "d", deviceName);
    cJSON_AddNumberToObject(body, "t", esp_timer_get_time());

    return body;
}

/*---------- User Functions ----------*/

// TODO: Refactor these to take MAC address either as a param or move the gateway mac from main.h to messaging.h

esp_err_t sendLog(char* logMessage, uint8_t* destinationMAC)
{
    static const char* TAG = "sendLog";
    esp_err_t res;
    cJSON* body = createMessageBody();

    ESP_LOGD(TAG, "Adding log message to body...");
    cJSON_AddStringToObject(body, "l", logMessage);

    // if on ESPNow gateway, return sendMessageSerial?
    res = sendMessageESPNow(body, destinationMAC);
    return res;

}

esp_err_t sendReadings(float* readings, int numReadings, uint8_t* destinationMAC)
{
    static const char* TAG = "sendReadings";
    esp_err_t res;
    cJSON* body = createMessageBody();

    ESP_LOGI(TAG, "Adding readings to body...");
    cJSON* readingsArray = cJSON_CreateArray();
    for (int i = 0; i < numReadings; i++) {
        cJSON_AddItemToArray(readingsArray, cJSON_CreateNumber(readings[i]));
    }

    cJSON_AddItemToObject(body, "r", readingsArray);

    res = sendMessageESPNow(body, destinationMAC);
    return res;

}

esp_err_t sendCommand(char* command, uint8_t* targetDeviceId)
{
    static const char* TAG = "sendCommand";
    esp_err_t res;
    cJSON* body = createMessageBody();

    ESP_LOGD(TAG, "Adding command to body...");
    cJSON_AddStringToObject(body, "c", command);

    res = sendMessageSerial(body);
    return res;
    
}

// EoF
