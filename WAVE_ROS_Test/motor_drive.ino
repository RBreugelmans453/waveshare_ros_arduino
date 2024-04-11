#ifdef GENERAL_MOTOR_DRIVER
const uint16_t PWMA = 25;         
const uint16_t AIN2 = 17;        
const uint16_t AIN1 = 21;         
const uint16_t BIN1 = 22;       
const uint16_t BIN2 = 23;        
const uint16_t PWMB = 26;   

const uint16_t ANALOG_WRITE_BITS = 8;

int freq = 100000;
int channel_A = 0; //left wheel, screen is on the back!
int channel_B = 1; //right wheel
int resolution = ANALOG_WRITE_BITS;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
const uint16_t MIN_PWM = MAX_PWM/4;

// true:forward, false:backward
bool direction_A = true;
bool direction_B = true;


void initMotorController() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    ledcSetup(channel_A, freq, resolution);
    ledcAttachPin(PWMA, channel_A);

    ledcSetup(channel_B, freq, resolution);
    ledcAttachPin(PWMB, channel_B);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);      
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    int conv_spd = -spd;
    
    if (i == LEFT) { 
      if      (conv_spd > 0) {    //left forward
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        ledcWrite(channel_A, conv_spd);
        }
      else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        ledcWrite(channel_A, -conv_spd);
        }
      }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (conv_spd > 0) {    //right forward
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        ledcWrite(channel_B, conv_spd);
        }
      // if (reverse == 1) {    //right reverse
      else {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        ledcWrite(channel_B, -conv_spd);
        }
      }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if(leftSpeed > 0){
      direction_A = true;
    }
    else{
      direction_A = false;
    }

    if(rightSpeed > 0){
      direction_B = true;
    }
    else{
      direction_B = false;
    }
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#endif