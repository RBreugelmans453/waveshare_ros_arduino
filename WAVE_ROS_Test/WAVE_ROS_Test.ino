#define BAUDRATE     115200
#define GENERAL_MOTOR_DRIVER

#include <Wire.h>
#include "commands.h"
#include "motor_drive.h"
#include "screenCtrl.h"
#include "IMU.h"

#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int indextest = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

//IMU variables
#define S_SCL   33
#define S_SDA   32

float INA219_DATA_V = -1;
int   IMU_Roll = 100;
int   IMU_Pitch = 100;
int   IMU_Yaw = 100;

int acce_X;
int acce_Y;
int acce_Z;

int gyro_X;
int gyro_Y;
int gyro_Z;

int magn_X;
int magn_Y;
int magn_Z;

int   IMU_Temp = 100;

IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

//UNCOMMENT IF THESE COME UP TO BE THE ONES THAT SHOULD BE SEND TO THE JETSON, think I should use these, a imu msg in ros also uses 9
//s16 acce_X;
//s16 acce_Y;
//s16 acce_Z;

//s16 gyro_X;
//s16 gyro_Y;
//s16 gyro_Z;

//s16 magn_X;
//s16 magn_Y;
//s16 magn_Z;

//For getting the IMU data. Again, uncomment if these are required for the jetson.
void getIMU(){
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  IMU_Temp = QMI8658_readTemp();
  IMU_Roll  = stAngles.fRoll;
  IMU_Pitch = stAngles.fPitch;
  IMU_Yaw   = stAngles.fYaw;

  acce_X = stAccelRawData.s16X;
  acce_Y = stAccelRawData.s16Y;
  acce_Z = stAccelRawData.s16Z;

  gyro_X = stGyroRawData.s16X;
  gyro_Y = stGyroRawData.s16Y;
  gyro_Z = stGyroRawData.s16Z;

  magn_X = stMagnRawData.s16X;
  magn_Y = stMagnRawData.s16Y;
  magn_Z = stMagnRawData.s16Z;
  sendIMU();
}

void sendIMU(){
  Serial.print("imu,");
  Serial.print(acce_X);
  Serial.print(",");
  Serial.print(acce_Y);
  Serial.print(",");
  Serial.print(acce_Z);
  Serial.print(",");

  //Serial.print("GYRO,");
  Serial.print(gyro_X);
  Serial.print(",");
  Serial.print(gyro_Y);
  Serial.print(",");
  Serial.print(gyro_Z);
  Serial.print(",");

  //Serial.print("MAGN,");
  Serial.print(magn_X);
  Serial.print(",");
  Serial.print(magn_Y);
  Serial.print(",");
  Serial.print(magn_Z);
  Serial.print(",");

  //printing the converted imu msg
  Serial.print(IMU_Roll);
  Serial.print(",");
  Serial.print(IMU_Pitch);
  Serial.print(",");
  Serial.print(IMU_Yaw);
  Serial.print(",");
  Serial.println(" ");  
  //Serial.print(",");
  //Serial.println("reset");
  //maybe add a println here, so it will stop reading after this
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  indextest = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  //Serial.println("runcommand called");  
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    ledcWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    //if (arg2 == 0) pinMode(arg1, INPUT);
    //else if (arg2 == 1) pinMode(arg1, OUTPUT);
    //Serial.println("OK");
    //getIMU();
    break;
  case READ_ENCODERS:
    Serial.print("50");
    Serial.print(" ");
    Serial.println("50");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    Serial.println("joe");    
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
    }
    else setMotorSpeeds(arg1, arg2);    
    Serial.println("OK"); 
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Wire.begin(S_SDA, S_SCL);
  Serial.begin(BAUDRATE);
  delay(2000);
  initMotorController();
  imuInit();
  Serial.println("setup complete");  

  //InitScreen();
  //allDataUpdate();  
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  currentMillis = millis();
  //Serial.println("test4");
   while (Serial.available() > 0){
    //Serial.println("test1");
    // Read the next character
    chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13) {
      Serial.println("d");
      if (arg == 1) argv1[indextest] = NULL;
      else if (arg == 2) argv2[indextest] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      Serial.println("e");      
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[indextest] = NULL;
        arg = 2;
        indextest = 0;
        Serial.println("f");
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
        Serial.println("g");
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[indextest] = chr;
        indextest++;
        Serial.println("h");
      }
      else if (arg == 2) {
        argv2[indextest] = chr;
        indextest++;
        Serial.println("i");
      }
    }
  }
  //Serial.println("test2");
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
  }
  //getIMU();
  delay(100);  
}
