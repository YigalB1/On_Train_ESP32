// from:" https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/"

//#include <Arduino.h>
#include <esp_now.h>
#include "WiFi.h"

int STOP_key= 21;
int FWD_key = 19;
int BCK_key = 18;
int BUZ_key =  5;
int buzz_time = 300; // in ms
int SPEED_key = 15;

void send_it();

// Insert the RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct struct_message {
  bool stop;
  bool fwd;
  bool bck;
  bool buzz;
  int buzz_time;
  int speed;
} struct_message;
struct_message TrainCMD;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
} // of OnDataSent()




void setup() {
  pinMode(STOP_key,INPUT_PULLUP);
  pinMode(FWD_key ,INPUT_PULLUP);
  pinMode(BCK_key ,INPUT_PULLUP);
  pinMode(BUZ_key ,INPUT_PULLUP);
  

  Serial.begin(9600);
  // getMAC address
  //WiFi.mode(WIFI_MODE_STA);
  //Serial.println(WiFi.macAddress());
  // sender: 7C:9E:BD:E3:F5:0C
 
  // Prepare the ESP now 
  WiFi.mode(WIFI_STA);   // Set device as a Wi-Fi Station

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } // of if()

  // Register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  } // of if()

} // of setup()

bool STOP_pressed = false;
bool FWD_pressed  = false;
bool BCK_pressed  = false;
bool BUZ_pressed  = false;
int speed = 0;


void loop() {
  delay(200);

  // start reading keys
  STOP_pressed = digitalRead(STOP_key);
  FWD_pressed  = digitalRead(FWD_key);
  BCK_pressed  = digitalRead(BCK_key);
  BUZ_pressed  = digitalRead(BUZ_key);
  speed = analogRead(SPEED_key);
  

  if (STOP_pressed) {
    TrainCMD.stop = true;
    TrainCMD.fwd  = false;
    TrainCMD.bck  = false;
    TrainCMD.speed = 0;
    TrainCMD.buzz = BUZ_pressed;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    return;
  } // of if()
  
  if (FWD_pressed) {
    TrainCMD.stop = false;
    TrainCMD.fwd  = true;
    TrainCMD.bck  = false;
    TrainCMD.speed = speed;
    TrainCMD.buzz = BUZ_pressed;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    return;
  } // of if()
  
  if (BCK_pressed) {
    TrainCMD.stop = false;
    TrainCMD.fwd  = false;
    TrainCMD.bck  = true;
    TrainCMD.speed = speed;
    TrainCMD.buzz = BUZ_pressed;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    return;
  } // of if()
  
  // in case BUZ was pressed without other keys
  // because BUZ can be alone or with other keys
  // all other leys are mtutual exclusive

  if (BUZ_pressed) {
    TrainCMD.buzz = true;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    return;
  } // of if()
  

} // of loop()




void send_it() {
    // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &TrainCMD, sizeof(TrainCMD));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } // of if()
  else {
    Serial.println("Error sending the data");
  } // of if()
} // of get_mac()
 