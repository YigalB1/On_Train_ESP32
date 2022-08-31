//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h> // for the temp sensor (one wire bus)
#include <DallasTemperature.h>

#include <classes.h>

// LED1 is power ...
int Led2_yellow = 21 ; 
int Led3_blue =   18 ; 
int Led4_green =   5 ; 
int Buzzer_pin     =  23 ; 

const int EN1_Pin  = 13 ; // 35 is inpput only ! 
const int IN1A_Pin = 32 ;
const int IN1B_Pin = 14 ;  

const int EN2_Pin  =  2 ;  
const int IN2A_Pin =  19 ;  
const int IN2B_Pin =  4 ;  

int Temperature_pin = 15; // temperature sensor

int trigPin1 = 26; 
int echoPin1 = 27;
const int trigPin2 = 33;  
const int echoPin2 = 35; 

// for PWM (driving motors)
const int freq = 5000;
const int ledChannel0 = 0;
const int ledChannel2 = 2;
const int resolution = 8;

void wait_millies(int _period);
int measure_it();

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(Temperature_pin);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

bool state = true;

typedef struct struct_message {
  bool stop;
  bool fwd;
  bool bck;
  bool buzz;
  int  buzz_time;
  int  speed;
  int  temperature;
} struct_message;

struct_message TrainCMD;  // the data package to be sent
esp_now_peer_info_t peerInfo; // the info about the peer

// to recieve on ESP_NOW 
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// for transmit on ESP_NOW:
// Remote COntrol MAC Address (Main board)
//uint8_t MAC_RC[] = {0x7C, 0x9E, 0xBD, 0xE3, 0xF5, 0x0C}; // wrong !
uint8_t MAC_RC[] = {0x40, 0x91, 0x51, 0x50, 0x31, 0xD0};

// callback when data is sent (from remote control)
void onSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0){
    Serial.println("Delivery success");
  } // of if()
  else{
    Serial.println("Delivery fail");
  } // of else()
} // of onSent()





motor_control motor_1;
motor_control motor_2;
leds led_stop;
leds led_fwd;
leds led_bck;

UltraSonic_sensor  dist_sensor_1;
UltraSonic_sensor  dist_sensor_2;

car my_car;

buzzer buzzer;


// ********************* SETUP ****************************
void setup() {
  Serial.begin(9600);
  
  pinMode(Led2_yellow, OUTPUT);
  pinMode(Led3_blue, OUTPUT);
  pinMode(Led4_green, OUTPUT);
  pinMode(Buzzer_pin, OUTPUT);

  //pinMode(EN1_Pin, OUTPUT); // changes to PWM
  pinMode(IN1A_Pin, OUTPUT);
  pinMode(IN1B_Pin, OUTPUT);
  //pinMode(EN2_Pin, OUTPUT);
  pinMode(IN2A_Pin, OUTPUT);
  pinMode(IN2B_Pin, OUTPUT);

  pinMode(trigPin1, OUTPUT); 
  pinMode(echoPin1, INPUT); 
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT); 

  dist_sensor_1._trig = trigPin1;
  dist_sensor_1._echo = echoPin1;
  dist_sensor_2._trig = trigPin2;
  dist_sensor_2._echo = echoPin2;
    
  my_car.motors_right.init(EN1_Pin,IN1A_Pin,IN1B_Pin,freq,resolution,ledChannel0);
  my_car.motors_left.init(EN2_Pin,IN2A_Pin,IN2B_Pin,freq,resolution,ledChannel2);

  my_car.f_sns.init(trigPin1,echoPin1);
  my_car.b_sns.init(trigPin2,echoPin2);

  led_stop._led = Led2_yellow;
  led_fwd._led = Led3_blue;
  led_bck._led = Led4_green;

  buzzer._buz_pin = Buzzer_pin;
 

  // setup ESPnow to recieve from remote control
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  Serial.println(WiFi.macAddress()); // // get MAC address
  // (main):            {0x10, 0x52, 0x1C, 0x5B, 0x0C, 0xCC}
  // (remote control):  {0x7C, 0x9E, 0xBD, 0xE3, 0xF5, 0x0C};

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.print("..Error init ESP-NOW.......");
    return;
  } // of if()
  
  // register for Send CB to, get status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register for RECIEVE CB to get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, MAC_RC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  } // of if()

// Start the DS18B20 sensor
  sensors.begin();

  Serial.println("end of Setup()");
} // of setup()

int time_on = 30*1000; // in ms
int time_off= 10*1000; // in ms
int tmp_speed=777;

// for delay function
int period = 50;

int ADC_val = 0;

int i;

// *************************** MAIN **********************************
void loop() {

  my_car.go_f_auto();
  return;

  my_car.go_fwd(200);
  wait_millies(3000);
  my_car.stop();
  wait_millies(1000);
  my_car.go_bck(200);
  wait_millies(3000);
  my_car.stop();
  wait_millies(1000);
  return;



  //int dist = measure_it();
  int dist1 = dist_sensor_1.measure_it();
  int dist2 = dist_sensor_2.measure_it();

  Serial.print(dist1);
  Serial.print(".......");
  Serial.println(dist2);
  
  digitalWrite(Buzzer_pin,HIGH);
//buzzer.set_buzzer_on();
  //for (int i=255;i>150;i-=20) {
    i=255;
    led_fwd.set_led_on();
    led_bck.set_led_off();
    motor_1.go_fwd(i);
    motor_2.go_fwd(i);
    Serial.print("FWD ");
    Serial.println(i);
    wait_millies(5000);
    //buzzer.set_buzzer_off();
    digitalWrite(Buzzer_pin,LOW);
  //};

  //for (int i=255;i>150;i-=20) {
    i=255;
    led_fwd.set_led_off();
    led_bck.set_led_on();
    motor_1.go_back(i);
    motor_2.go_back(i);
    Serial.print("BACK ");
    Serial.println(i);
    wait_millies(5000);
  //};


  return;
  /*



  return;

    

  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  
  Serial.print(temperatureC);
  Serial.println("ºC");
  
  TrainCMD.temperature = temperatureC;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(MAC_RC, (uint8_t *) &TrainCMD, sizeof(TrainCMD));
   
  if (result == ESP_OK) {
    //Serial.print("Sent with success...");
  }
  else {
    Serial.print("Error sending the data...");
  }
  

  if (TrainCMD.buzz) {
    TrainCMD.buzz = false;
    motor_1.make_buzz(TrainCMD.buzz_time);
  }// inner if()
  
  if (TrainCMD.stop) {
    TrainCMD.stop = false;
    //Serial.println("Stopping");
    motor_1.stop();
    return;
  }  // if()

  if (TrainCMD.fwd) {
    TrainCMD.fwd = false;
    //Serial.println("FWD");
    motor_1.go_fwd(TrainCMD.speed);
    return;
  }  // if()

    if (TrainCMD.bck) {
      TrainCMD.bck = false;
    //Serial.println("BCK");
    motor_1.go_back(TrainCMD.speed);
    return;
  }  // if()

*/

} // of main()
// *************************************************************


// *************************************************************

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // callback function that will be executed when data is received
  memcpy(&TrainCMD, incomingData, sizeof(TrainCMD));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("stop: ");
  Serial.println(TrainCMD.stop);
  Serial.print("fwd: ");
  Serial.println(TrainCMD.fwd);
  Serial.print("bck: ");
  Serial.println(TrainCMD.bck);
  Serial.print("buzz: ");
  Serial.println(TrainCMD.buzz);
  Serial.print("buzz_time: ");
  Serial.println(TrainCMD.buzz_time);
  Serial.print("speed: ");
  Serial.println(TrainCMD.speed);
  Serial.println("................................");
} // of OnDataRecv()
