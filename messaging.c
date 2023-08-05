#include "messaging.h"

// TODO: Move to separate file
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
}




// Takes a destionation address and a message queue. Sends messages from the queue to the destination address.
// TODO: Parse destination from message or second parameter
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

void sendMessageString(char* topic, char* payload) {
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "topic", topic);
    cJSON_AddStringToObject(message, "payload", payload);

    sendMessageJSON(message);
}

void sendMessageFloat(char* topic, float payload) {
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "topic", topic);
    cJSON_AddNumberToObject(message, "payload", payload);

    sendMessageJSON(message);
}

void createMessageQueues() {
    static const char *TAG = "createMessageQueues";

    ESP_LOGD(TAG, "Creating incomingMessage queue...");
    incomingMessageQueue = xQueueCreate(10, sizeof(Message*));

    ESP_LOGD(TAG, "Creating outgoingMessage queue...");
    outgoingMessageQueue = xQueueCreate(10, sizeof(Message*));
}
