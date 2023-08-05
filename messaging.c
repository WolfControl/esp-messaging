#include "messaging.h"

/*---------- Handlers: ESPNow  ----------*/
void OnESPNowSendDevice(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    static const char *TAG = "OnESPNowSendDevice";

    ESP_LOGI(TAG, "Last Packet Send Status: %d", status == ESP_NOW_SEND_SUCCESS);

}

void OnESPNowRecvDevice(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
    static const char *TAG = "OnESPNowRecvDevice";

    ESP_LOGD(TAG, "Posting packet from gateway to incomingMessageQueue, length: %d", len);
    if (xQueueSend(incomingMessageQueue, (void *) incomingData, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send packet to incomingMessage queue");
        sendMessageString("device/error", "MessageReceivedFailed!");
    }
}

void OnESPNowSendGateway(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    static const char *TAG = "OnESPNowSendGateway";

    ESP_LOGI(TAG, "Last Packet Send Status: %d", status == ESP_NOW_SEND_SUCCESS);
}

void OnESPNowRecvGateway(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
    static const char *TAG = "OnESPNowRecvGateway";

    // Print the address of the sender and the message
    ESP_LOGI(TAG, "Received data from %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Received %d bytes", len);
    ESP_LOGI(TAG, "Received data: %s", incomingData);

    // todo: call function to check for and register peer

    cJSON* json = parseAndValidateJson((const char*)incomingData);
    if(json != NULL) {
        const char* topic = parseJsonField(json, "topic");
        const char* payload = parseJsonField(json, "payload");

        if(topic && payload) {
            ESP_LOGI(TAG, "Forwarding incoming ESP-NOW message to serial bridge");
            // prepend topic with "hydronet/gatewayid/deviceid" where deviceID is mapped to mac_addr of sender
            sendMessage(topic, payload);
        } else {
            ESP_LOGE(TAG, "Could not parse topic or payload from incoming data");
        }
        
        // Free the memory after using the json object
        cJSON_Delete(json);
    } else {
        ESP_LOGE(TAG, "Could not parse incoming data as JSON");
    }
}

/*---------- Handlers: Serial  ----------*/
// Used for handshake in setupSerial. Not intended to be used by the user
void setupHandler(const char* topic, const char* payload)
{
    const char*TAG = "setupHandler";

    if (strcmp(topic, "handshake") == 0) {
        if (strcmp(payload, "CONN") == 0) {
            ESP_LOGI(TAG, "Received CONN from the other side! Responding with CONNACK...");
            sendMessage("handshake", "CONNACK");
        } else if (strcmp(payload, "CONNACK") == 0) {
            ESP_LOGI(TAG, "Received CONNACK from the other side!");
            hasReceivedCONNACK = true;
        }
    }
}

// Receives a string, parses and validates it as a JSON object, then passes the data to an abstracted handler (expects parameters const char* topic, const char* payload)
void onSerialReceive(const char* data, messageHandler handler) {
    static const char* TAG = "onSerialReceive";
    ESP_LOGD(TAG, "Parsing and validating JSON...");
    cJSON* jsonObject = parseAndValidateJson(data);

    if (jsonObject == NULL) {
        ESP_LOGE(TAG, "Failed to parse or validate JSON");
        return;
    }

    const char* topicValue = parseJsonField(jsonObject, "topic");
    const char* payloadValue = parseJsonField(jsonObject, "payload");

    if (strcmp(topicValue, "handshake") == 0 && strcmp(payloadValue, "CONN") == 0) {
        ESP_LOGI(TAG, "Received handshake message, peer crashed! Reshaking hands...");
        setupHandler(topicValue, payloadValue);
    } else {
        ESP_LOGI(TAG, "Passing data to handler...");
        handler(topicValue, payloadValue);
    }

    ESP_LOGD(TAG, "Releasing JSON object...");
    cJSON_Delete(jsonObject);
}


/*---------- Setup functions ----------*/
esp_err_t setupESPNow (const uint8_t *gatewayAddress, bool isGateway)
{
    static const char *TAG = "setupESPNowCommon";
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
    if (isGateway) {
        ret = esp_now_register_recv_cb(OnESPNowRecvGateway);
    } else {
        ret = esp_now_register_recv_cb(OnESPNowRecvDevice);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback");
        return ret;
    }

    // setup message queues
    ESP_LOGD(TAG, "Creating incomingMessage queue...");
    incomingMessageQueue = xQueueCreate(10, sizeof(Message*));
    if (incomingMessageQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create incomingMessage queue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Creating outgoingMessage queue...");
    outgoingMessageQueue = xQueueCreate(10, sizeof(Message*));
    if (outgoingMessageQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create outgoingMessage queue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sensor setup complete");
    return ESP_OK;
}

void setupSerial(messageHandler handler, int txPin, int rxPin)
{
    const char*TAG = "setupSerial";

    // TODO: replace magic numbers with defines
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // TODO: error handling
    ESP_LOGD(TAG, "Setting UART pins and installing driver...");
    uart_param_config(UART_NUMBER, &uart_config);
    uart_set_pin(UART_NUMBER, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUMBER, BUF_SIZE, BUF_SIZE, UART_QUEUE_SIZE, NULL, 0);
    esp_vfs_dev_uart_use_driver(UART_NUMBER);

    hasSentCONN = false;
    hasReceivedCONNACK = false;

     ESP_LOGI(TAG, "Starting handshake task...");
    if (pdPASS != xTaskCreate(listenSerialTask, "handshake_task", TASK_STACK_SIZE, (void *) setupHandler, TASK_PRIORITY, &handshakeTaskHandle)) {
        ESP_LOGE(TAG, "Failed to create task: handshake_task");
        return;
    }

    // Retry with backoff
    int delayMs = 100;  // Start with 100ms delay
    while(!hasReceivedCONNACK) {
        if(!hasSentCONN) {
            ESP_LOGI(TAG, "Sending CONN message...");
            sendMessage("handshake", "CONN");
            hasSentCONN = pdTRUE;
        }

        vTaskDelay(pdMS_TO_TICKS(delayMs)); // wait before checking the flag

        if (hasReceivedCONNACK == pdFALSE) {
            delayMs *= 2;  // Double the delay
            if (delayMs > 800) delayMs = 100; // Avoid too long delay, reset after reaching 800ms
            ESP_LOGI(TAG, "Resending CONN message...");
            sendMessage("handshake", "CONN");
        }
    }

    // sleep for 500ms to allow the other side to finish setup
    vTaskDelay(500 / portTICK_PERIOD_MS);

    vTaskDelete(handshakeTaskHandle);
    handshakeTaskHandle = NULL;

    ESP_LOGI(TAG, "Bridge Setup Complete!");

    // Start the main listener task with the user-provided handler
    ESP_LOGI(TAG, "Starting listener task with user provided handler");
    xTaskCreate(listenSerialTask, "listenSerial_task", TASK_STACK_SIZE, (void *) handler, TASK_PRIORITY, &serialListenerTaskHandle);
}

/*---------- RTOS tasks ----------*/ 

// Receives messages from the outgoingMessageQueue and sends them to destination passed as parameter.
// TODO: pull destination from message struct. Currently sends all messages to gateway
void sendMessageTask(void *pvParameters)
{

    static const char *TAG = "sendMessageTask";
    Message* outgoingMsg;

    const uint8_t *destinationAddress = (uint8_t *)pvParameters;

    while(1){
        // Receive the Message struct from the queue
        if (xQueueReceive(outgoingMessageQueue, &outgoingMsg, portMAX_DELAY) == pdTRUE)
        {
            // Send the JSON string over ESP-NOW
            esp_err_t result = esp_now_send(destinationAddress, (uint8_t *) outgoingMsg->message, outgoingMsg->size); 

            if (result == ESP_OK) {
                ESP_LOGI(TAG, "Published packet to ESP-NOW: %s", outgoingMsg->message);
            }
            else {
                ESP_LOGE(TAG, "Error: %s", esp_err_to_name(result));
            }

            free(outgoingMsg->message);    // Free the packet string
            free(outgoingMsg); // Free the Message struct
        }
    }
}

void listenSerialTask(void* pvParameters)
{
    messageHandler handler = (messageHandler)pvParameters;
    const char* TAG = "listenSerialTask";

    while (true) {        
        size_t len = 0;
        uart_get_buffered_data_len(UART_NUMBER, &len);
        if (len > 0) {
            char incomingData[BUF_SIZE];
            int received_msg_length = uart_read_bytes(UART_NUMBER, (uint8_t*)incomingData, BUF_SIZE - 1, UART_READ_TIMEOUT_MS / portTICK_RATE_MS); // leaving 1 byte for null terminator
            incomingData[received_msg_length] = '\0';  // Null-terminate the string

            if (received_msg_length > 0 && incomingData[received_msg_length - 1] == '\0') {                
                ESP_LOGI(TAG, "Received %d bytes: %s", received_msg_length, incomingData);
                onSerialReceive(incomingData, handler);
            } else if (received_msg_length == BUF_SIZE - 1) {
                ESP_LOGE(TAG, "Received message too long for buffer: %d bytes", received_msg_length);
                uart_flush(UART_NUMBER);
            } else if (received_msg_length == -1) {
                ESP_LOGE(TAG, "Internal error in UART driver");
                uart_flush(UART_NUMBER);
            } else {
                ESP_LOGE(TAG, "Received incomplete or malformed message");
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);  // delay to allow other tasks to run
    }
}


/*---------- Helper functions ----------*/

// TODO: refactor to sendESPNowJSON, string, etc
// takes a cJSON object, serializes, wraps in Message struct, posts to outgoingMessage queue. Does not parse input.
bool sendMessageJSON(cJSON *message) {
    char* packetString = cJSON_PrintUnformatted(message);

    Message* outgoingMsg = (Message*) malloc(sizeof(Message));
    outgoingMsg->message = packetString;
    outgoingMsg->size = strlen(packetString) + 1;

    if (xQueueSend(outgoingMessageQueue, &outgoingMsg, 0) != pdTRUE) {
        ESP_LOGE("sendMessageJSON", "Failed to send packet to outgoingMessage queue");
        free(outgoingMsg->message);
        free(outgoingMsg);
        cJSON_Delete(message);
        return false;
    }

    cJSON_Delete(message);
    return true;
}

void sendMessageString(const char* topic,const  char* payload) {
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "topic", topic);
    cJSON_AddStringToObject(message, "payload", payload);

    sendMessageJSON(message);
}

void sendMessageFloat(const char* topic, float payload) {
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "topic", topic);
    cJSON_AddNumberToObject(message, "payload", payload);

    sendMessageJSON(message);
}

// TOP PRIO: Refactor on espnow gateway to rename to sendSerialMessage
// Creates a JSON message from the provided topic and payload, and transmits it over UART
void sendMessage(const char* topic, const char* payload) {
    static const char* TAG = "sendMessage";
    static const char newline = '\0';

    // Create the JSON object
    ESP_LOGD(TAG, "Creating JSON object...");
    cJSON* jsonObject = cJSON_CreateObject();
    cJSON_AddStringToObject(jsonObject, "topic", topic);
    cJSON_AddStringToObject(jsonObject, "payload", payload);

    // Serialize the JSON object to a string
    ESP_LOGD(TAG, "Serializing JSON object...");
    const char* jsonStr = cJSON_PrintUnformatted(jsonObject);

    // Check that the serialized JSON string doesn't exceed the buffer size
    if (strlen(jsonStr) > BUF_SIZE) {
        ESP_LOGE(TAG, "Serialized JSON string exceeds buffer size");
        cJSON_Delete(jsonObject);
        free((void*)jsonStr);
        return;
    }

    // Transmit the JSON string over UART
    ESP_LOGI(TAG, "Transmitting %d bytes: %s", strlen(jsonStr), jsonStr);
    uart_write_bytes(UART_NUMBER, jsonStr, strlen(jsonStr));
    uart_write_bytes(UART_NUMBER, &newline, 1);

    // Clean up
    cJSON_Delete(jsonObject);
    free((void*)jsonStr);
}

// Helper function to parse cJSON fields, returns the string value of a given field
char* parseJsonField(const cJSON* jsonObject, const char* fieldName) {
    static const char* TAG = "parseJsonField";
    cJSON* fieldObject = cJSON_GetObjectItemCaseSensitive(jsonObject, fieldName);
    if (fieldObject == NULL || !cJSON_IsString(fieldObject)) {
        ESP_LOGE(TAG, "No '%s' field in JSON object or '%s' is not a string", fieldName, fieldName);
        return NULL;
    }
    return fieldObject->valuestring;
}

// Helper function to parse and validate incoming JSON data
cJSON* parseAndValidateJson(const char* rawData) {
    static const char* TAG = "parseAndValidateJson";

    ESP_LOGD(TAG, "Parsing incoming data as JSON...");
    cJSON* jsonObject = cJSON_Parse(rawData);
    if (jsonObject == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return NULL;
    }

    ESP_LOGD(TAG, "Confirming JSON object contains expected fields...");
    if (!parseJsonField(jsonObject, "topic") || !parseJsonField(jsonObject, "payload")) {
        cJSON_Delete(jsonObject);
        return NULL;
    }

    return jsonObject;
}