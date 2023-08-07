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
    incomingESPNowQueue = xQueueCreate(10, sizeof(Message*));
    if (incomingESPNowQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create incomingESPNowQueue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Creating outgoingESPNowQueue...");
    outgoingESPNowQueue = xQueueCreate(10, sizeof(Message*));
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

    ESP_LOGD(TAG, "Sensor setup complete");
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
    Message* outgoingMsg;

    while(1){
        // Receive the Message struct from the queue
        if (xQueueReceive(outgoingESPNowQueue, &outgoingMsg, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGD(TAG, "Received message from outgoingESPNowQueue: %s", outgoingMsg->bodySerialized);
        
            // TODO: Parsing

            ESP_LOGD(TAG, "Sending packet to destination MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                outgoingMsg->destinationMAC[0], 
                outgoingMsg->destinationMAC[1], 
                outgoingMsg->destinationMAC[2], 
                outgoingMsg->destinationMAC[3], 
                outgoingMsg->destinationMAC[4], 
                outgoingMsg->destinationMAC[5]);
            
            esp_err_t result = esp_now_send(outgoingMsg->destinationMAC, (uint8_t *) outgoingMsg->bodySerialized, outgoingMsg->size); 

            if (result == ESP_OK) {
                ESP_LOGD(TAG, "Published packet to ESP-NOW: %s", outgoingMsg->bodySerialized);
            }
            else {
                ESP_LOGE(TAG, "Error: %s", esp_err_to_name(result));
            }

            // Free the Message struct and packet string
            free(outgoingMsg->bodySerialized);    
            free(outgoingMsg); 
        }
    }
}

void sendSerialTask(void *pvParameters)
{
    static const char *TAG = "sendSerialTask";
    Message* outgoingMsg;
    static const char newline = '\0';

    while(1) {
        // Receive the Message struct from the queue
        if (xQueueReceive(outgoingSerialQueue, &outgoingMsg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received message from outgoingSerialQueue: %s", outgoingMsg->bodySerialized);

            // TODO: Parsing

            // Check that the serialized JSON string doesn't exceed the buffer size
            if (outgoingMsg->size > BUF_SIZE) {
                ESP_LOGE(TAG, "Serialized JSON string exceeds buffer size");
                free(outgoingMsg->bodySerialized);
                free(outgoingMsg);
                continue;
            }

            ESP_LOGD(TAG, "Sending packet to UART...");
            uart_write_bytes(UART_NUMBER, outgoingMsg->bodySerialized, outgoingMsg->size);
            uart_write_bytes(UART_NUMBER, &newline, 1);

            // Free the Message struct and packet string
            free(outgoingMsg->bodySerialized);    
            free(outgoingMsg); 
        }
    }
}

void receiveESPNowTask (void* pvParameters)
{
    static const char *TAG = "receiveESPNowTask";
    messageHandler handler = (messageHandler)pvParameters;
    Message* incomingMsg;

    while (1) {
        ESP_LOGD(TAG, "Waiting for incoming data...");

        if (xQueueReceive(incomingESPNowQueue, &incomingMsg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Received message from incomingESPNowQueue: %s", incomingMsg->bodySerialized);

            // TODO: Parsing

            // Gateway handler: forward over serial
            // Device handler: process incoming command

            ESP_LOGD(TAG, "Passing data to handler...");
            //onESPNowReceive(incomingMsg, handler);

        }
    }

}

void receiveSerialTask(void* pvParameters)
{
    static const char* TAG = "receiveSerialTask";
    messageHandler handler = (messageHandler)pvParameters;
    Message* incomingMsg;

    while (1) {
        if (xQueueReceive(incomingSerialQueue, &incomingMsg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received data from incomingSerialQueue: %s", incomingMsg->bodySerialized);

            // TODO: Parsing

            ESP_LOGD(TAG, "Passing data to handler...");
            //onSerialReceive(incomingMsg, handler);
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

bool sendMessageJSON(cJSON *body, uint8_t *destinationMAC) {
    
    Message* outgoingMsg = (Message*) malloc(sizeof(Message));
    outgoingMsg->bodySerialized = cJSON_PrintUnformatted(body);
    outgoingMsg->size = strlen(outgoingMsg->bodySerialized) + 1;
    memcpy(outgoingMsg->destinationMAC, destinationMAC, 6);

    QueueHandle_t queue = destinationMAC ? outgoingESPNowQueue : outgoingSerialQueue;
    const char* TAG = destinationMAC ? "sendMessageESPNow" : "sendMessageSerial";

    if (xQueueSend(queue, &outgoingMsg, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send packet to queue: %s", TAG);
        free(outgoingMsg->bodySerialized);
        free(outgoingMsg);
        cJSON_Delete(body);
        return false;
    }

    cJSON_Delete(body);
    return true;
}

bool sendMessageTopicPayload(const char* topic, const char* payload, uint8_t *destinationMAC) {
    cJSON *body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "topic", topic);
    cJSON_AddStringToObject(body, "payload", payload);
    return sendMessageJSON(body, destinationMAC);
}