#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <IntervalTimer.h>
#include <flap.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define ICM_CS 10

const bool DEBUG = false;

IntervalTimer logDataTimer; // Create an IntervalTimer object
IntervalTimer printDataTimer;
IntervalTimer saveSDTimer;
IntervalTimer statusBeepTimer;

File myFile;

Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mag;

Adafruit_BMP3XX bmp;

Servo release;

float pressure;
float altitude;
float temperature;

bool sensorError = false;

float roll = 0;
float pitch = 0;

float rollingLoopTime = 0;
float previousLoopTime = 0;
float currentLoopTime = 0;

int delay_start_millis = 0;
int loop_start_millis = 0;

int fileIndex = 1;

const int ematch_1 = 10;
const int ematch_2 = 11;
const int motor_1 = 8;
const int motor_2 = 9;

const int buzzer = 34;

const int servo = 12;

const float sensitivity = 2.8;

float dz = 0; //change in altitude meters per second
float previousAltitude = 0;

float ax = 0;
float ay = 0;
float az = 0;

float motorPower = 0;

enum State{
  WAITING,
  LAUNCHED,
  APOGEE,
  BELLYFLOP,
  CHUTE,
  LANDED
};

void waiting_state();
void launched_state();
void apogee_state();
void bellyflop_state();
void chute_state();

State currentState = WAITING;

void logData() {
  myFile.print(temp.temperature);
  myFile.print(", ");
  myFile.print(ax);
  myFile.print(", ");
  myFile.print(ay);
  myFile.print(", ");
  myFile.print(az);
  myFile.print(", ");
  myFile.print(gyro.gyro.x);
  myFile.print(", ");
  myFile.print(gyro.gyro.y);
  myFile.print(", ");
  myFile.print(gyro.gyro.z);
  myFile.print(", ");
  myFile.print(mag.magnetic.x);
  myFile.print(", ");
  myFile.print(mag.magnetic.y);
  myFile.print(", ");
  myFile.print(mag.magnetic.z);
  myFile.print(", ");
  myFile.print(temp.temperature);
  myFile.print(", ");
  myFile.print(pressure);
  myFile.print(", ");
  myFile.print(pitch);
  myFile.print(", ");
  myFile.print(roll);
  myFile.print(", ");
  myFile.print(millis());
  myFile.print(", ");
  myFile.print(sensorError);
  myFile.print(", ");
  myFile.println(currentState);
  myFile.print(", ");
  myFile.println(dz);
  myFile.print(", ");
  myFile.println(motorPower);
}

void saveSD(){
  myFile.close();
  String fileName = "test" + String(fileIndex) + ".txt";
  myFile = SD.open(fileName.c_str(), FILE_WRITE);
  if(!myFile){
    Serial.println("Error opening file");
    sensorError = true;
  }
}

void printData(){
  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(ax);
  Serial.print(" \tY: ");
  Serial.print(ay);
  Serial.print(" \tZ: ");
  Serial.print(az);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  // Serial.print("\t\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.println(" uT");

  Serial.print("\t\tpressure = "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("\t\taltitude = "); Serial.print(altitude); Serial.println(" m");
  Serial.print("\t\ttemperature = "); Serial.print(temperature); Serial.println(" C");
  // //delay();

  Serial.print("\t\tPitch: "); Serial.print(pitch); Serial.print(" Roll: "); Serial.println(roll);
  Serial.print ("\t\tLoop Time: "); Serial.println(rollingLoopTime);
  Serial.print("\t\tError: "); Serial.println(sensorError);
  Serial.print("\t\tState: "); Serial.println(currentState); 
  Serial.print("\t\tDz: "); Serial.println(dz);
  Serial.print("\t\tMotor Power: ");Serial.println(motorPower);

}

int beepLoopCount = 0;
void statusBeep(){
  switch (currentState){
    case WAITING:
      if(beepLoopCount < 1){
        digitalWrite(buzzer, LOW);
      }else{
        digitalWrite(buzzer, HIGH);
      }
      break;
    //base case
    default:
      if(beepLoopCount < 8){
        digitalWrite(buzzer, LOW);
      }else{
        digitalWrite(buzzer, HIGH);
      }
  }
  beepLoopCount++;
  if (beepLoopCount > 10) beepLoopCount = 0;

}

void setup(void) {
  Serial.begin(115200);

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip (accelerometer, gyroscope, magnetometer)!");
    sensorError = true;
    while (1) {}
  }

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Initialization failed!");
    sensorError = true;
    while (1) {}
  }

  while (true) {
    String fileName = "test" + String(fileIndex) + ".txt";
    if (!SD.exists(fileName.c_str())) { // Check if file exists
      // File does not exist, create it
      myFile = SD.open(fileName.c_str(), FILE_WRITE);
      if (!myFile) {
        Serial.println("Error opening " + fileName);
      }
      break;
    }
    fileIndex++; // Increment the file index if the file exists
  }  


  if(!myFile){
    Serial.println("Error opening file");
    sensorError = true;
    while(1);
  }

  if (!bmp.begin_I2C()) {
    Serial.println("Failed to find BMP388 chip (barometer)!");
    sensorError = true;
    while (1) {}
  }


  myFile.print("Temperature (Accel), Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, Mag X, Mag Y, Mag Z, Temp (baro), Pressure (baro), Pitch, Roll, Time, Sensor Error, State, Dz\n");

  Serial.println("ICM20948 Found!");
  icm_temp = icm.getTemperatureSensor();
  icm_temp->printSensorDetails();

  icm_accel = icm.getAccelerometerSensor();
  icm_accel->printSensorDetails();

  icm_gyro = icm.getGyroSensor();
  icm_gyro->printSensorDetails();

  icm_mag = icm.getMagnetometerSensor();
  icm_mag->printSensorDetails();

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay_start_millis = millis();

  release.attach(servo);
  release.writeMicroseconds(2000); // 2000 closed, 950 open

  if(DEBUG){ 
    printDataTimer.begin(printData, 100000); // 10 hz printing
  }else{
    logDataTimer.begin(logData, 50000); // 20 hz logging
    saveSDTimer.begin(saveSD, 2000000); // 0.5 hz saving to SD
  }

  statusBeepTimer.begin(statusBeep, 100000);

}


void loop() {
  if(loop_start_millis == 0){
    loop_start_millis = millis();
  }

  az = accel.acceleration.x;
  ay = accel.acceleration.y;
  ax = accel.acceleration.z;

  previousLoopTime = currentLoopTime;
  currentLoopTime = millis();
  rollingLoopTime = currentLoopTime - previousLoopTime;

  //  /* Get a new normalized sensor event */
  icm_temp->getEvent(&temp);
  icm_accel->getEvent(&accel);
  icm_gyro->getEvent(&gyro);
  icm_mag->getEvent(&mag);

  pressure = bmp.pressure / 100.0;
  altitude = bmp.readAltitude(1013.25);
  temperature = bmp.temperature;

  dz = (altitude - previousAltitude) / (rollingLoopTime / 1000);
  previousAltitude = altitude;

  roll = (atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI);
  pitch = (atan2(az, sqrt(ay * ay + ax * ax)) * 180 / M_PI);

  switch(currentState){
    case WAITING:
      waiting_state();
      break;
    case LAUNCHED:
      launched_state();
      break;
    case APOGEE:
      apogee_state();
      break;
    case BELLYFLOP:
      bellyflop_state();
      break;
    case CHUTE:
      chute_state();
      break;
    case LANDED:
      break;
  }

}

float ignitionConditionsMet = 0;
int launchTime = 0;
float launchAltitude = 0;
void waiting_state(){
  


  //Next state
  if(az > 20) ignitionConditionsMet += 1;
  if(dz > 10) ignitionConditionsMet += 1;

  if(ignitionConditionsMet > 10){
    launchTime = millis();
    launchAltitude = altitude; //Not ideal: altitude will be higher than actual launch altitude
    currentState = LAUNCHED;
  }

  ignitionConditionsMet -= 0.5;
}

float apogeeConditionsMet = 0;
float maxAltitude = 0;
int apogeeTime = 0;
void launched_state(){

  //Next state
  if(millis() - launchTime > 13000) apogeeConditionsMet += 1;
  if(abs(az) < 7) apogeeConditionsMet += 1;
  if((altitude - maxAltitude) < 100) apogeeConditionsMet += 1;
  if(dz < 5) apogeeConditionsMet += 1;

  if(apogeeConditionsMet > 10){
    apogeeTime = millis();
    maxAltitude = altitude;
    currentState = APOGEE;
  }
  apogeeConditionsMet -= 1;

}


void apogee_state(){
  release.writeMicroseconds(950);

  //Next state
  if(millis() - apogeeTime > 500) currentState = BELLYFLOP;
}

float chuteConditionsMet = 0;
int chuteIgnitionTime = 0;
void bellyflop_state(){
  release.writeMicroseconds(950);  

  motorPower = pitch * sensitivity;

  if(motorPower > 0){
    digitalWrite(motor_1, LOW);
    analogWrite(motor_2, abs(motorPower));
  }else{
    digitalWrite(motor_2, LOW);
    analogWrite(motor_1, abs(motorPower));
  }


  //Next state
  if(altitude - launchAltitude < 100) chuteConditionsMet += 1;
  if(pitch < -60 && az < 4) chuteConditionsMet += 0.15; //Takes 2.4 seconds before moving on at just pitch
  if(dz > 70) chuteConditionsMet += 0.1;

  chuteConditionsMet -= 0.1;

  if(chuteConditionsMet > 10){
    chuteIgnitionTime = millis();
    currentState = CHUTE;
  }

}


void chute_state(){ //DANGER - Only bellyflop should transition into this
  if(chuteIgnitionTime == 0){
    chuteIgnitionTime = millis();
  }
  if(millis() - chuteIgnitionTime < 5000){
    digitalWrite(ematch_1, HIGH);
    digitalWrite(ematch_2, LOW);
  // }else if (millis() - chuteIgnitionTime < 10000){
  //   digitalWrite(ematch_1, LOW);
  //   digitalWrite(ematch_2, HIGH);
  }else{
    digitalWrite(ematch_1, LOW);
    digitalWrite(ematch_2, LOW);
  }

}


