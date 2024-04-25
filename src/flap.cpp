#include "Flap.h"

const float Flap::ALPHA = 0.1; // Initialize the ALPHA constant

Flap::Flap(int pin, int minAngle, int maxAngle) {
    servoPin = pin;
    minPos = minAngle;
    maxPos = maxAngle;
    currentPosition = minAngle; // Default to minimum position
    previousPosition = minAngle;
    this->clockwise = clockwise;
}

void Flap::attachServo() {
    servo.attach(servoPin, 600, 2400);
    // setPosition(minPos);  // Initialize servo to minimum position
}

void Flap::write(int angle) {
    // if (angle > maxPos)
    //     angle = maxPos;
    // else if (angle < minPos)
    //     angle = minPos;

    currentPosition = angle;
    servo.writeMicroseconds(angle);
}

void Flap::writePercentage(float percentage) {
    int angle = map(percentage, 0, 100, minPos, maxPos);
    if (angle > max(maxPos, minPos))
        angle = max(maxPos, minPos);
    else if (angle < min(maxPos, minPos))
        angle = min(maxPos, minPos);
    servo.writeMicroseconds(angle);
}

int Flap::getPosition() {
    return currentPosition;
}

void Flap::calibrate() {
    int angle = 1500;
    const int step = 5;
    while(true){
        if (Serial.available() > 0) {
            // read the incoming byte:
            char input = Serial.read();
            if(input == 'q'){
                break;
            }
            if(input == 'w'){
                angle += step;
            }
            if(input == 's'){
                angle -= step;
            }
            write(angle);
            Serial.println(angle);
        }
    }
}
