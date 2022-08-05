#include <Arduino.h>

// LED1 is power ...
int Led2_yellow = 21 ; 
int Led3_blue =   18 ; 
int Led4_green =   5 ; 
int Buzzer     =  23 ; 

int EN1_Pin  = 35 ;  
int IN1A_Pin = 32 ;
int IN1B_Pin = 14 ;  

int EN2_Pin  =  2 ;  
int IN2A_Pin =  19 ;  
int IN2B_Pin =  4 ;  


bool state = true;

bool tmp = HIGH;




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
    digitalWrite(EN1_Pin,LOW); 
    digitalWrite(Led2_yellow, HIGH);
    digitalWrite(Led3_blue, LOW);  
    digitalWrite(Led4_green, LOW);
    digitalWrite(Buzzer, LOW);
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

}; // of class motor_control



motor_control motor_1;
motor_control motor_2;

void setup() {
  pinMode(Led2_yellow, OUTPUT);
  pinMode(Led3_blue, OUTPUT);
  pinMode(Led4_green, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  motor_1.en = EN1_Pin;
  motor_1.in_1 = IN1A_Pin;
  motor_1.in_2 = IN2A_Pin;
  motor_1.stop();

  //digitalWrite(Led2_yellow, LOW);
  //digitalWrite(Led3_blue, LOW);  
  //digitalWrite(Led4_green, LOW);
  //digitalWrite(Buzzer, LOW);

  Serial.begin(9600);
  tmp = HIGH;

// disable notors
//motor_1.en = LOW;
//motor_1.in_1 = LOW;
//motor_1.in_2 = LOW;
//motor_2.en = LOW;
//motor_2.in_1 = LOW;
//motor_2.in_2 = LOW;
Serial.println("end of Setup()");
}

void loop() {
  int tmp_speed=777;
  

  motor_1.go_fwd(tmp_speed);
  delay(2000);

  motor_1.stop();
  delay(1000);

  motor_1.go_back(tmp_speed);
  delay(2000);

  motor_1.stop();
  delay(1000);
  
}

