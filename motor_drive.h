#ifdef GENERAL_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 23
  #define LEFT_MOTOR_BACKWARD  21
  #define RIGHT_MOTOR_FORWARD  22
  #define LEFT_MOTOR_FORWARD   17
  #define RIGHT_MOTOR_ENABLE 26
  #define LEFT_MOTOR_ENABLE 25
#endif
void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);