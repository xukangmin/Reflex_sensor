/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave(s)

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

#include <esp_now.h>
#include <WiFi.h>

#define CONFIG_ESPNOW_CHANNEL 1
#define CHANNEL 1
#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#define CONFIG_ESPNOW_LMK "lmk1234567890123"

#define NUMSLAVES 20
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int slaveCnt = 0;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

enum {
    SENSOR_DATA_REGISTER,
    SENSOR_DATA_TRIGGER,
    SENSOR_DATA_RECV_CONFIRM,
    SENSOR_DATA_RESULT,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint8_t sensor_data_type;              //Sensor data type
    uint8_t payload[25];                   //Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;


static espnow_data_t sensor_data;
static int seq_num = 0;

static uint32_t recv_diff;
// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
  
  
}


// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

 
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.println("Last Packet Recv Data: "); 
  for (int i = 0; i < data_len; i++)
  {
    Serial.print(data[i],HEX);
    Serial.print(" ");
  }
  Serial.println("");
  // add peer
  if (esp_now_is_peer_exist(mac_addr) == false)
  {


        
    slaves[slaveCnt].channel = CONFIG_ESPNOW_CHANNEL;
    slaves[slaveCnt].encrypt = true;
    memcpy(slaves[slaveCnt].lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    memcpy(slaves[slaveCnt].peer_addr, mac_addr, ESP_NOW_ETH_ALEN);

    esp_err_t addStatus = esp_now_add_peer(&slaves[slaveCnt]);
    if (addStatus == ESP_OK) {
      // Pair success
      Serial.println("Pair success");
    } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW Not Init");
    } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Add Peer - Invalid Argument");
    } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
      Serial.println("Peer list full");
    } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("Out of memory");
    } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
      Serial.println("Peer Exists");
    } else {
      Serial.println("Not sure what happened");
    }

    sensor_data.type = ESPNOW_DATA_UNICAST;
    sensor_data.state = 0;
    sensor_data.seq_num = seq_num++;
    sensor_data.crc = 0;
    sensor_data.sensor_data_type = SENSOR_DATA_REGISTER;

    memset(sensor_data.payload,0,sizeof(sensor_data.payload));
    

    esp_err_t result = esp_now_send(slaves[slaveCnt].peer_addr, (const uint8_t *)&sensor_data, sizeof(sensor_data));

    delay(500);

    
    slaveCnt++;

  }

  espnow_data_t* recv_data = (espnow_data_t*)data;

  if (recv_data->sensor_data_type == SENSOR_DATA_RESULT)
  {
    Serial.println("receive result data from slave");

    memcpy(&recv_diff,recv_data->payload,sizeof(uint32_t));

    Serial.print("received diff");

    Serial.println(recv_diff);
  }
  

}

void send_trigger_request_to_slave()
{
    sensor_data.type = ESPNOW_DATA_UNICAST;
    sensor_data.state = 0;
    sensor_data.seq_num = seq_num++;
    sensor_data.crc = 0;
    sensor_data.sensor_data_type = SENSOR_DATA_TRIGGER;

    memset(sensor_data.payload,0,sizeof(sensor_data.payload));

    esp_err_t result = esp_now_send(slaves[0].peer_addr, (const uint8_t *)&sensor_data, sizeof(sensor_data));
}

void loop() {
  // Chill
  delay(5000);

  send_trigger_request_to_slave();
}
