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
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


#include <esp_now.h>
#include <WiFi.h>
#include <RGBmatrixPanel.h>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK  15   // USE THIS ON ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   33
#define LAT 32
#define A   12
#define B   16
#define C   17
#define D  18

int running_speed = 5;
#define CONFIG_ESPNOW_CHANNEL 1
#define CHANNEL 1
#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#define CONFIG_ESPNOW_LMK "lmk1234567890123"

RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);

#define NUMSLAVES 20
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int slaveCnt = 0;

int period = 10000;
int time_now = 0;

void update_period()
{
   if(running_speed == 0)
   {
      period = 500;
   }
   else{
      period = 1000 * running_speed;
   }
}

volatile int result_received = 0;
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

static uint32_t recv_diff = 0;
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

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        if (rxValue.compare("+") == 0)
        {
            if (running_speed < 10)
            {
              running_speed++;
            }
        }
        else if (rxValue.compare("-") == 0)
        {
            if (running_speed > 0)
            {
              running_speed--;
            }
        }

        update_period();
        update_screen();
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {
  
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  //WiFi.mode(WIFI_AP);
  WiFi.mode(WIFI_STA);
  // configure device AP mode
  // configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
  
  

  BLEDevice::init("Reflex Trainer");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  matrix.begin();

  update_screen();
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

  espnow_data_t* recv_data = (espnow_data_t*)data;

  if (recv_data->type == ESPNOW_DATA_BROADCAST)
  {
      if (esp_now_is_peer_exist(mac_addr) == false)
      {
        slaves[slaveCnt].channel = CONFIG_ESPNOW_CHANNEL;
        slaves[slaveCnt].encrypt = false;
        slaves[slaveCnt].ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
        // memcpy(slaves[slaveCnt].lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
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
    
       
    
        // memset(sensor_data.payload,0,sizeof(sensor_data.payload));
    
       for(int i = 0; i < 6; i++)
       {
        Serial.print(slaves[slaveCnt].peer_addr[i],HEX);
        Serial.print(" ");
       }
       Serial.println("");


      
    
        delay(500);
    
        
        slaveCnt++;
        Serial.println("peer added");
        update_screen();
      }

      sensor_data.type = ESPNOW_DATA_UNICAST;
      sensor_data.state = 0;
      sensor_data.seq_num = seq_num++;
      sensor_data.crc = 0;
      sensor_data.sensor_data_type = SENSOR_DATA_REGISTER;
    
      esp_err_t result = esp_now_send(mac_addr, (const uint8_t *)&sensor_data, sizeof(sensor_data));
  
      if (result == ESP_OK)
      {
        Serial.println("send ok");
      }
      else{
        Serial.print("send error=");
        Serial.println(result);
      }
      
  }
    
  


  
  

  if (recv_data->sensor_data_type == SENSOR_DATA_RESULT)
  {
    Serial.println("receive result data from slave");

    memcpy(&recv_diff,recv_data->payload,sizeof(uint32_t));

    Serial.print("received diff");

    Serial.println(recv_diff);


    update_screen();
    result_received = 1;

  }
  

}

void update_screen()
{
  
    matrix.fillScreen(matrix.Color333(0, 0, 0));

    matrix.setCursor(0, 0);

    matrix.setTextSize(1);     // size 1 == 8 pixels high
    matrix.setTextWrap(false); // Don't wrap at end of line - will do ourselves

    matrix.print("m:");
    matrix.print(slaveCnt);

    matrix.print(" ");

    matrix.print("s:");
    matrix.println(running_speed);

    if (recv_diff > 0)
    {
      matrix.setCursor(2, 10);    // start at top left, with 8 pixel of spacing
      
      matrix.setTextSize(2);     // size 1 == 8 pixels high
      matrix.setTextWrap(false); // Don't wrap at end of line - will do ourselves
    
      matrix.setTextColor(matrix.Color333(4,0,7));
      matrix.println(recv_diff);
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

    int rand_slave = 0;

    if (slaveCnt > 1)
    {
      rand_slave = random(0, slaveCnt);  
    }

    Serial.print("random_index=");
    Serial.println(rand_slave);

    esp_err_t result = esp_now_send(slaves[rand_slave].peer_addr, (const uint8_t *)&sensor_data, sizeof(sensor_data));
}


void loop() {
  // Chill
     if (deviceConnected) {
//        pTxCharacteristic->setValue(&txValue, 1);
//        pTxCharacteristic->notify();
//        txValue++;
//        
     // delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    
   if (millis() > time_now + period)
  {
    // start laser
    time_now = millis();

    if (slaveCnt > 0)
    {
      send_trigger_request_to_slave();
    }
    
  }

  
}
