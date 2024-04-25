#ifndef Flap_h
#define Flap_h

#include <Arduino.h>
#include <Servo.h>

class Flap {
public:
    Flap(int pin, int minAngle, int maxAngle);  // Constructor
    void attachServo();                         // Attaches the servo
    void write(int angle);                // Set the servo's position within bounds
    int getPosition();                          // Get the current servo position
    void calibrate();
    void writePercentage(float percentage);

private:
    Servo servo;           // Servo object
    int servoPin;          // Pin number connected to the servo
    int minPos;            // Minimum angle position
    int maxPos;            // Maximum angle position
    int currentPosition;   // Current position of the servo
    int previousPosition;  // Previous position of the servo
    static const float ALPHA;
    bool clockwise;
};

#endif
