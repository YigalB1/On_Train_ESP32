//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h> // for the temp sensor (one wire bus)
#include <DallasTemperature.h>


// LED1 is power ...
int Led2_yellow = 21 ; 
int Led3_blue =   18 ; 
int Led4_green =   5 ; 
int Buzzer_pin     =  23 ; 

int EN1_Pin  = 13 ; // 35 is inpput only ! 
int IN1A_Pin = 32 ;
int IN1B_Pin = 14 ;  

int EN2_Pin  =  2 ;  
int IN2A_Pin =  19 ;  
int IN2B_Pin =  4 ;  

int Temperature_pin = 15; // temperature sensor

const int trigPin = 26; 
const int echoPin = 27;


// for PWM (driving motors)
const int freq = 5000;
const int ledChannel = 0;
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



class motor_control {
  public:
  int en;
  int in_1;
  int in_2;
  // each instance of class will have different motor
  // in this case we have one motor

void stop() {
    digitalWrite(in_1, LOW); // 
    digitalWrite(in_2, LOW); //
    //digitalWrite(en,LOW); 
        ledcWrite(ledChannel, 0);

    digitalWrite(Led2_yellow, HIGH);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, LOW);
    digitalWrite(Buzzer_pin, LOW);
  } // of stop()

  void go_fwd(int _speed) {
    
    digitalWrite(Led2_yellow, LOW);
    digitalWrite(Led3_blue, HIGH);  
    digitalWrite(Led4_green, LOW);
    digitalWrite(in_1, LOW); // 
    digitalWrite(in_2, HIGH); //
    //digitalWrite(en,HIGH);   // moved to PWM
    ledcWrite(ledChannel, _speed);
    
  } // of go_fwd()

  void go_back(int _speed) {
    digitalWrite(Led2_yellow, LOW);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, HIGH);
    digitalWrite(in_1, HIGH); // 
    digitalWrite(in_2, LOW); // 
    //digitalWrite(en,HIGH);
    ledcWrite(ledChannel, _speed);
  } // of go_back()

  void make_buzz(int _buz_time) {
    digitalWrite(Buzzer_pin, HIGH);
    delay(_buz_time);
    digitalWrite(Buzzer_pin, LOW);
  } // of make_buzz()

}; // of class motor_control



motor_control motor_1;
motor_control motor_2;


// ********************* SETUP
void setup() {
  Serial.begin(9600);
  
  pinMode(Led2_yellow, OUTPUT);
  pinMode(Led3_blue, OUTPUT);
  pinMode(Led4_green, OUTPUT);
  pinMode(Buzzer_pin, OUTPUT);

  //pinMode(EN1_Pin, OUTPUT); // changes to PWM
  pinMode(IN1A_Pin, OUTPUT);
  pinMode(IN1B_Pin, OUTPUT);
  pinMode(EN2_Pin, OUTPUT);
  pinMode(IN2A_Pin, OUTPUT);
  pinMode(IN2B_Pin, OUTPUT);

  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(EN1_Pin, ledChannel);




  motor_1.en   = EN1_Pin;
  motor_1.in_1 = IN1A_Pin;
  motor_1.in_2 = IN1B_Pin;
  motor_1.stop();
  
  motor_2.en   = EN2_Pin;
  motor_2.in_1 = IN2A_Pin;
  motor_2.in_2 = IN2B_Pin;
  motor_2.stop();


  //tmp = HIGH;

  

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
unsigned long time_now = 0;
int ADC_val = 0;


// ************   LOOP
void loop() {

  int dist = measure_it();
  Serial.println(dist);
  wait_millies(100);

  return;

  for (int i=255;i>100;i-=10) {
    motor_1.go_fwd(i);
    Serial.print("FWD ");
    Serial.println(i);
    wait_millies(5000);
  }

;

  motor_1.go_fwd(128);
  Serial.println("FWD 128 for 5 seconds");
  wait_millies(5000);

  motor_1.go_fwd(24);
  Serial.println("FWD 24 for 5 seconds");
  wait_millies(5000);
  
  motor_1.stop();
  Serial.println("STOP for 1 second");
  wait_millies(1000);

  motor_1.go_back(255);
  Serial.println("BCK 255 for 5 seconds");
  wait_millies(5000);

  motor_1.stop();
  Serial.println("STOP for 1 second");
  wait_millies(1000);

  return;


    digitalWrite(IN1A_Pin, LOW);  
    digitalWrite(IN1B_Pin, HIGH); 
    //ledcWrite(ledChannel, 255);
    motor_1.go_fwd(255);
    digitalWrite(Led2_yellow, HIGH);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, LOW);
    Serial.println("255");
    delay(3000);

    ledcWrite(ledChannel, 128);
    digitalWrite(Led2_yellow, LOW);
    digitalWrite(Led3_blue, HIGH);  
    digitalWrite(Led4_green, LOW);
    Serial.println("128");
    delay(3000);

    ledcWrite(ledChannel, 80);
    digitalWrite(Led2_yellow, LOW);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, HIGH);
    Serial.println("80");
    delay(3000);
    
return;

 // increase the LED brightness
  for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle+=80 ){   
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    Serial.print(dutyCycle);
    Serial.println("...");
    delay(3000);
  }

  return;




  time_now = millis();
  while(millis() < time_now + period){
        //wait approx. [period] ms
    }

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


} // of main()


void wait_millies(int _period) {
  time_now = millis();
  while(millis() < time_now + _period){
        //wait approx. [period] ms
    } // of while() 

} // of wait_millies()

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
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

int measure_it() {
  int duration = 0;
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW); 
  duration = pulseIn(echoPin, HIGH);
  int distance = (duration*.0343)/2;
  return(distance);
} // of measure_it