//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>


// LED1 is power ...
int Led2_yellow = 21 ; 
int Led3_blue =   18 ; 
int Led4_green =   5 ; 
int Buzzer_pin     =  23 ; 

int EN1_Pin  = 12 ; // 35 is inpput only ! 
int IN1A_Pin = 32 ;
int IN1B_Pin = 14 ;  

int EN2_Pin  =  2 ;  
int IN2A_Pin =  19 ;  
int IN2B_Pin =  4 ;  

bool state = true;
bool tmp = HIGH;

typedef struct struct_message {
  bool stop;
  bool fwd;
  bool bck;
  bool buzz;
  int buzz_time;
  int speed;
} struct_message;
struct_message TrainCMD;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);


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
    digitalWrite(en,LOW); 
    digitalWrite(Led2_yellow, HIGH);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, LOW);
    digitalWrite(Buzzer_pin, LOW);
  } // go_fwd()

  void go_fwd(int _speed) {
    digitalWrite(Led2_yellow, LOW);
    digitalWrite(Led3_blue, HIGH);  
    digitalWrite(Led4_green, LOW);
    digitalWrite(in_1, LOW); // 
    digitalWrite(in_2, HIGH); //
    digitalWrite(en,HIGH); 
  } // go_fwd()

    void go_back(int _speed) {
    digitalWrite(Led2_yellow, LOW);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, HIGH);
    digitalWrite(in_1, HIGH); // 
    digitalWrite(in_2, LOW); // 
    digitalWrite(en,HIGH);
  } // go_back()

  void make_buzz(int _buz_time) {
    digitalWrite(Buzzer_pin, HIGH);
    delay(_buz_time);
    digitalWrite(Buzzer_pin, LOW);
  } // of make_buzz()

}; // of class motor_control



motor_control motor_1;
motor_control motor_2;

void setup() {

  // getMAC address
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  //sender: 7C:9E:BD:E3:F5:0C

  pinMode(Led2_yellow, OUTPUT);
  pinMode(Led3_blue, OUTPUT);
  pinMode(Led4_green, OUTPUT);
  pinMode(Buzzer_pin, OUTPUT);


  pinMode(EN1_Pin, OUTPUT);
  pinMode(IN1A_Pin, OUTPUT);
  pinMode(IN1B_Pin, OUTPUT);
  pinMode(EN2_Pin, OUTPUT);
  pinMode(IN2A_Pin, OUTPUT);
  pinMode(IN2B_Pin, OUTPUT);


  motor_1.en   = EN1_Pin;
  motor_1.in_1 = IN1A_Pin;
  motor_1.in_2 = IN1B_Pin;
  motor_1.stop();

  Serial.begin(9600);
  tmp = HIGH;

  // setup ESPnow to recieve from remote control
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } // of if()
  
  // Register for recv CB to get recv packer info
  esp_now_register_recv_cb(OnDataRecv);




  Serial.println("end of Setup()");
}

int time_on = 30*1000; // in ms
int time_off= 10*1000; // in ms
int tmp_speed=777;

void loop() {

  if (TrainCMD.buzz) {
    motor_1.make_buzz(TrainCMD.buzz_time);
  }// inner if()
  
  if (TrainCMD.stop) {
    motor_1.stop();
    return;
  }  // if()

  if (TrainCMD.fwd) {
    motor_1.go_fwd(TrainCMD.speed);
    return;
  }  // if()

    if (TrainCMD.bck) {
    motor_1.go_back(TrainCMD.speed);
    return;
  }  // if()



/*
  motor_1.go_fwd(tmp_speed);
  delay(time_off);

  motor_1.stop();
  delay(1000);

  motor_1.go_back(tmp_speed);
  delay(time_on);

  motor_1.stop();
  delay(time_off);
  */
}



// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&TrainCMD, incomingData, sizeof(TrainCMD));
  Serial.print("Bytes received: ");
  Serial.println(len);
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
}

