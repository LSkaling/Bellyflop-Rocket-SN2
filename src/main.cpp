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

const bool DEBUG = false;

IntervalTimer logDataTimer; // Create an IntervalTimer object
IntervalTimer printDataTimer;
IntervalTimer saveSDTimer;

File myFile;

Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mag;

Adafruit_BMP3XX bmp;

float pressure;
float altitude;
float temperature;

bool sensorError = false;

float roll = 0;
float pitch = 0;

float rollingLoopTime = 0;
float previousLoopTime = 0;
float currentLoopTime = 0;

int topLeftError = 0;
int topRightError = 0;
int bottomLeftError = 0;
int bottomRightError = 0;

int previousTopLeftError = 0;
int previousTopRightError = 0;
int previousBottomLeftError = 0;
int previousBottomRightError = 0;

int tlFlap = 0;
int trFlap = 0;
int blFlap = 0;
int brFlap = 0;

const float alpha = 0.3;
const float roll_sensitivity = 0; //1 for SN2 drop 2.
const float pitch_sensitivity = 1.5; // 0.9?

#define ICM_CS 10

Flap topLeft(9, 1775, 885);
Flap topRight(10, 1165, 1955);
Flap bottomLeft(11, 995, 1930);
Flap bottomRight(12, 1525, 705);

int risingEdgeTime = 0;
int fallingEdgeTime = 0;
int onTime = 0;
int offTime = 0;

bool balance = false;

const float P = 1;
const float D = 0;

int delay_start_millis = 0;
int loop_start_millis = 0;

int neutral_angle = 45;

int fileIndex = 1;

void logData() {
  myFile.print(temp.temperature);
  myFile.print(", ");
  myFile.print(accel.acceleration.x);
  myFile.print(", ");
  myFile.print(accel.acceleration.y);
  myFile.print(", ");
  myFile.print(accel.acceleration.z);
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
  myFile.print(tlFlap);
  myFile.print(", ");
  myFile.print(trFlap);
  myFile.print(", ");
  myFile.print(blFlap);
  myFile.print(", ");
  myFile.print(brFlap);
  myFile.println();
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
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
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

  Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");

  Serial.print("\t\tpressure = "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("\t\taltitude = "); Serial.print(altitude); Serial.println(" m");
  Serial.print("\t\ttemperature = "); Serial.print(temperature); Serial.println(" C");
  // //delay();

  Serial.print("\t\tPitch: "); Serial.print(pitch); Serial.print(" Roll: "); Serial.println(roll);
  Serial.print ("\t\tLoop Time: "); Serial.println(rollingLoopTime);
  Serial.print("\t\tError: "); Serial.println(sensorError);

  Serial.print("\t\tTop Left Error: "); Serial.print(topLeftError); Serial.print("\tTop Right Error: "); Serial.print(topRightError); Serial.print("\tBottom Left Error: "); Serial.print(bottomLeftError); Serial.print("\tBottom Right Error: "); Serial.println(bottomRightError);

  Serial.print("receiver"); Serial.print(", ");Serial.print(onTime); Serial.print(", ");Serial.print(offTime);Serial.print(", start delay: ");Serial.print(loop_start_millis - delay_start_millis);

}

void pwmChange(){
  if(digitalRead(39) == HIGH){
    risingEdgeTime = millis();
    offTime = risingEdgeTime - fallingEdgeTime;
  }
  else{
    fallingEdgeTime = millis();
    onTime = millis() - risingEdgeTime;
  }
  
}

void setup(void) {
  Serial.begin(115200);

  topLeft.attachServo();
  topRight.attachServo();
  bottomLeft.attachServo();
  bottomRight.attachServo();

  //attachInterrupt(digitalPinToInterrupt(39), pwmChange, CHANGE);

  // topLeft.calibrate(); //1775 is down, 885 is up
  // topRight.calibrate(); // 1165 is down, 1955 is up
  // bottomLeft.calibrate(); //995 is down, 1930 is up
  // bottomRight.calibrate(); //1525 is down, 705 is upq

  // topLeft.writePercentage(0);
  // topRight.writePercentage(0);
  // bottomLeft.writePercentage(0);
  // bottomRight.writePercentage(0);

  // delay(600);

  // topLeft.writePercentage(100);
  // topRight.writePercentage(100);
  // bottomLeft.writePercentage(100);
  // bottomRight.writePercentage(100);

  // delay(600);

  // Initialize peripherals

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


  myFile.print("Temperature (Accel), Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, Mag X, Mag Y, Mag Z, Temp (baro), Pressure (baro), Pitch, Roll, Time, Sensor Error, TL, TR, BL, BR\n");

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

  Serial.println("Setup complete! Waiting for release.");


  // topLeft.writePercentage(100);
  // topRight.writePercentage(100);
  // bottomLeft.writePercentage(0);
  // bottomRight.writePercentage(0); 

  //while(onTime < 2){  }
  
  delay_start_millis = millis();


  if(DEBUG){ 
    printDataTimer.begin(printData, 100000); // 10 hz printing
  }else{
    logDataTimer.begin(logData, 50000); // 20 hz logging
    saveSDTimer.begin(saveSD, 2000000); // 0.5 hz saving to SD
  }
  //detachInterrupt(digitalPinToInterrupt(39));

}

int loopCount = 0;

float cumAx = 0;
float cumAy = 0;
float cumAz = 0;

float ax = 0;
float ay = 0;
float az = 0;

void loop() {
  if(loop_start_millis == 0){
    loop_start_millis = millis();
  }

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

  ax = alpha * accel.acceleration.y + (1 - alpha) * ax;
  ay = alpha * accel.acceleration.x + (1 - alpha) * ay;
  az = alpha * accel.acceleration.z + (1 - alpha) * az;

  pitch = (atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI) * pitch_sensitivity;
  roll = (atan2(ax, sqrt(ay * ay + az * az)) * 180 / M_PI) * roll_sensitivity;

  topLeftError = pitch + roll;
  topRightError = pitch - roll;
  bottomLeftError = -pitch + roll;
  bottomRightError = -pitch - roll;

  float d_tle = topLeftError - previousTopLeftError;
  float d_tre = topRightError - previousTopRightError;
  float d_ble = bottomLeftError - previousBottomLeftError;
  float d_bre = bottomRightError - previousBottomLeftError;

  tlFlap = P*topLeftError + D*d_tle + neutral_angle;
  trFlap = P*topRightError + D*d_tre+ neutral_angle;
  blFlap = P*bottomLeftError + D*d_ble + neutral_angle;
  brFlap = P*bottomRightError + D*d_bre+ neutral_angle;

  topLeft.writePercentage(tlFlap);
  topRight.writePercentage(trFlap);
  bottomLeft.writePercentage(blFlap);
  bottomRight.writePercentage(brFlap);

  previousTopLeftError = topLeftError;
  previousTopRightError = topRightError;
  previousBottomLeftError = bottomLeftError;
  previousBottomRightError = bottomRightError;
}


