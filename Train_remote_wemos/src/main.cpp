// from:"https://randomnerdtutorials.com/esp-now-esp8266-nodemcu-arduino-ide/"
#include <ESP8266WiFi.h>
//#include "esp_now.h"
#include "ESPNowW.h"


int STOP_key= 14; // 
int FWD_key = 12; // GPIO12 is D6          GPIO13 is D7
//int FWD_key = 12; // GPIO16 is D0
int BCK_key = 13; // GPIO15 is D8
int BUZ_key =  4;
//int other = 5;
int buzz_time = 300; // in ms
int SPEED_key = 12;

int ADC_pin = A0;


bool pckg_recieved = false; // to know when a new package was recieved

void send_it();

// RECEIVER MAC Address (Main board)
uint8_t MAC_MAIN[] = {0x10, 0x52, 0x1C, 0x5B, 0x0C, 0xCC};


typedef struct struct_message {
  bool stop;
  bool fwd;
  bool bck;
  bool buzz;
  int  buzz_time;
  int  speed;
  int  temperature;
} struct_message;

struct_message TrainCMD; // the data package to be sent



// callback when data is sent
void onSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0){
    Serial.println("Delivery success");
  } // of if()
  else{
    Serial.println("Delivery fail");
  } // of else()
} // of onSent()


// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&TrainCMD, incomingData, sizeof(TrainCMD));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  //incomingTemp = incomingReadings.temp;
  //incomingHum = incomingReadings.hum;
  pckg_recieved = true;
}





void setup() {
  Serial.begin(9600);
  Serial.println("Setup starting");

  pinMode(STOP_key,INPUT_PULLUP);
  pinMode(FWD_key ,INPUT_PULLUP);
  pinMode(BCK_key ,INPUT_PULLUP);
  pinMode(BUZ_key ,INPUT_PULLUP);
  //pinMode(other ,INPUT_PULLUP);
    
  
 
  // Prepare the ESP now 
  WiFi.mode(WIFI_STA);   // Set device as a Wi-Fi Station
  // getMAC address
  Serial.println(WiFi.macAddress());
  // sender: 7C:9E:BD:E3:F5:0C

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } // of if()

   //esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
   esp_now_set_self_role(ESP_NOW_ROLE_COMBO); // to recieve and send

  // Register the peer
  //esp_now_add_peer(MAC_MAIN, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(MAC_MAIN, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Register send callback function
  esp_now_register_send_cb(onSent);
  // Register received callback function
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Setup ended");

} // of setup()

bool STOP_pressed = false;
bool FWD_pressed  = false;
bool BCK_pressed  = false;
bool BUZ_pressed  = false;
//bool other_pressed  = false;
int speed = 0;
bool msg_exist = false;

// for delay function
int period = 50;
unsigned long time_now = 0;


void loop() {

//ADC_val = analogRead(ADC_pin);  // This reads the analog in value
//  Serial.print(ADC_val);
//  return;


  time_now = millis();
  while(millis() < time_now + period){
        //wait approx. [period] ms
    }





  if (pckg_recieved) {
    pckg_recieved = false;
    // new package arrived
    Serial.print(TrainCMD.temperature);
    Serial.print("...");
  }
  

  // start reading keys
  STOP_pressed = !digitalRead(STOP_key);
  FWD_pressed  = !digitalRead(FWD_key);
  BCK_pressed  = !digitalRead(BCK_key);
  BUZ_pressed  = !digitalRead(BUZ_key);
  //other_pressed  = !digitalRead(other);
  //speed = analogRead(SPEED_key);

  msg_exist = STOP_pressed || FWD_pressed || BCK_pressed || BUZ_pressed;

  if (!msg_exist) {
    // nothing was pressed
    return;
  } // of if()

  
  //Serial.print("---->");
  //Serial.print(STOP_pressed);
  //Serial.print("....");
  //Serial.print(FWD_pressed);
  //Serial.print("....");
  //Serial.print(BCK_pressed);
  //Serial.print("....");
  //Serial.print(BUZ_pressed);
  //Serial.println("....");
  
  
  if (STOP_pressed) {
    Serial.println("STOP");
    TrainCMD.stop = true;
    TrainCMD.fwd  = false;
    TrainCMD.bck  = false;
    TrainCMD.speed = 0;
    TrainCMD.buzz = BUZ_pressed;
    TrainCMD.buzz_time = buzz_time;
    //Serial.println("sending stop");
    send_it();
    STOP_pressed = false;
    return;
  } // of if()
  
  if (FWD_pressed) {
    Serial.println("FWD");
    TrainCMD.stop = false;
    TrainCMD.fwd  = true;
    TrainCMD.bck  = false;
    TrainCMD.speed = speed;
    TrainCMD.buzz = BUZ_pressed;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    FWD_pressed  = false;
    return;
  } // of if()
  
  if (BCK_pressed) {
    Serial.println("BCK");
    TrainCMD.stop = false;
    TrainCMD.fwd  = false;
    TrainCMD.bck  = true;
    TrainCMD.speed = speed;
    TrainCMD.buzz = BUZ_pressed;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    BCK_pressed  = false;
    return;
  } // of if()
  
  // in case BUZ was pressed without other keys
  // because BUZ can be alone or with other keys
  // all other leys are mtutual exclusive

  if (BUZ_pressed) {
    Serial.println("sending BUZZ");
    TrainCMD.buzz = true;
    TrainCMD.buzz_time = buzz_time;
    send_it();
    BUZ_pressed  = false;
    return;
  } // of if()

  // None pressed, reset variables
  
  
  
  

} // of loop()




void send_it() {
    // Send message via ESP-NOW
  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &TrainCMD, sizeof(TrainCMD));
  esp_now_send(NULL, (uint8_t *) &TrainCMD, sizeof(TrainCMD));
   
  /*
  if (result == 0) {
    //Serial.println("Sent with success");
  } // of if()
  else {
    Serial.println("Error sending the data");
  } // of if()
  */
} // of send_it()
 