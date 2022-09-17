#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h> // for the temp sensor (one wire bus)
#include <DallasTemperature.h>

void wait_millies(int _period) {
    unsigned long time_now = millis();
  while(millis() < time_now + _period){
        //wait approx. [period] ms
    } // of while() 

} // of wait_millies()



class leds {
    public:
    int _led;

    void set_led_on() {
        digitalWrite(_led,HIGH);
    }
    void set_led_off() {
        digitalWrite(_led,LOW);
    }
}; // of class leds

class buzzer {
    public:
    int _buz_pin;

    void set_buzzer_on() {
        digitalWrite(_buz_pin,HIGH);
    }
    void set_buzzer_off() {
        digitalWrite(_buz_pin,LOW);
    }
};
// ***********************************************************

class UltraSonic_sensor {
    public:
    int _trig;
    int _echo;

    void init(int __trig, int __echo) {
        _trig = __trig;
        _echo = __echo; 
    } // of init()
    
    int measure_it() {
        int duration = 0;
        digitalWrite(_trig, LOW); 
        delayMicroseconds(2); 
        digitalWrite(_trig, HIGH); 
        delayMicroseconds(10); 
        digitalWrite(_trig, LOW); 
        duration = pulseIn(_echo, HIGH);
        int distance = (duration*.0343)/2;
        return(distance);
    } // of measure_it
  }; // of UltraSonic_sensor()

// ***********************************************************
  class motor_control {
    public:
    int _en;
    int _in_1;
    int _in_2;
    //int _channel;
    int _freq;
    int _resolution;;
    int _led_channel;
    
    

    //buzzer _buzzer;


    void init(int __en,int __in_1, int __in_2, int __freq, int __res, int __chan) {
        _en = __en;
        _in_1 = __in_1;
        _in_2 = __in_2;
        _freq = __freq;
        _resolution = __res;
        _led_channel = __chan;

        ledcSetup(_led_channel, _freq,_resolution);
        ledcAttachPin(_en, _led_channel);

    } // of init()

    

void stop() {
    digitalWrite(_in_1, LOW); // 
    digitalWrite(_in_2, LOW); //
    ledcWrite(_led_channel, 0); 
  } // of stop()

  void go_fwd(int _speed) {
    digitalWrite(_in_1, LOW); // 
    digitalWrite(_in_2, HIGH); // 
    ledcWrite(_led_channel, _speed);    
  } // of go_fwd()

  void go_back(int _speed) {
    digitalWrite(_in_1, HIGH); // 
    digitalWrite(_in_2, LOW); // 
    ledcWrite(_led_channel, _speed);
  } // of go_back()

/*
  void make_buzz(int _buz_time) {
    _buzzer.set_buzzer_on();
    //digitalWrite(Buzzer_pin, HIGH);
    delay(_buz_time); // TBD: change to wait_millies
    _buzzer.set_buzzer_off();
    //digitalWrite(Buzzer_pin, LOW);
  } // of make_buzz()
*/

}; // of class motor_control

/*
class leds {
    Public:
    int _led;

    void set_led_on() {
        digitalWrite(_led,HIGH);
    }
    void set_led_off() {
        digitalWrite(_led,LOW);
    }

*/

class car {
    public:
    int f_dist;
    int b_dist;
    // for PWM (driving motors)
    const int freq = 5000;
    const int ledChannel0 = 0;
    const int ledChannel2 = 2;
    const int resolution = 8;

    int dist_th = 50;       // distance from obstacle to make the car stop
    int turn_time = 100;    // minimal time per turn

    motor_control motors_left;
    motor_control motors_right;
    leds led_stop;
    leds led_fwd;
    leds led_bck;

    UltraSonic_sensor f_sns;
    UltraSonic_sensor b_sns;

   

    void init() {
        
        // TBD
        //ledcSetup(_led_channel, _freq,_resolution);
        //ledcAttachPin(_en, _led_channel);
    } // of init()

    void go_fwd(int _speed) {
        motors_left.go_fwd(_speed);
        motors_right.go_fwd(_speed);
    } // of go_fwd()

    void go_bck(int _speed) {
        motors_left.go_back(_speed);
        motors_right.go_back(_speed);
    } // of go_bck()

    void stop() {
        motors_left.go_back(0);
        motors_right.go_back(0);
    } // of stop()

    void turn_left(int _tm) {
        motors_left.go_fwd(_tm);
        motors_right.go_back(_tm);
    } // of turn_left

        void turn_right(int _tm) {
        motors_left.go_back(_tm);
        motors_right.go_fwd(_tm);
        
    } // of turn_right

    //void go_f_auto(UltraSonic_sensor _sens_f, UltraSonic_sensor _sens_b) {
    void go_f_auto() {
        //int d1 = _sens_f.measure_it();
        //int d2 = _sens_b.measure_it();
        int d1 = f_sns.measure_it();
        int d2 = b_sns.measure_it();

  
        if (d1 < dist_th) {
            stop();
            wait_millies(50);
            go_fwd(200);
            wait_millies(500);
            turn_left(200);
            wait_millies(500);
            go_bck(200);
        }
        else { 
            go_bck(200);
        } // of else()




    } // of go_f_auto

}; // of class car