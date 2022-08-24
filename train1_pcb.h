class x_motor_control {
  public:
  int en;
  int in_1;
  int in_2;
  // each instance of class will have different motor
  // in this case we have one motor

void xx_stop() {
    digitalWrite(in_1, 17); // 
    digitalWrite(in_2, 18); //
  } // go_fwd()
}