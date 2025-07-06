#include <Arduino.h>

// Define L293D Motor A pins
const int EN1 = 9;   // Enable pin for Motor A (PWM)
const int IN1 = 12;   // Input 1 for Motor A
const int IN2 = 13;   // Input 2 for Motor A

void setup() {
    Serial.begin(9600);
    // Set motor pins as output
    pinMode(EN1, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
}

void loop() {
    // Clockwise rotation
    Serial.println("CW Rotation");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, 200); // PWM for speed control
    delay(2000); // Rotate for 2 seconds
    analogWrite(EN1, 100);
    delay(2000);

    // Counter-clockwise rotation
    Serial.println("CCW Rotation");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, 200); // PWM for speed control
    delay(2000); // Rotate for 2 seconds
    analogWrite(EN1, 100);
    delay(2000);
}
