#include <Arduino.h>

const int sensorpin[5] = {A0, A1, A2, A3, A4}; // pin number attached to each IR sensor

void setup(){
    Serial.begin(9600);
    for(int i=0; i<5; i++){
        pinMode(sensorpin[i], INPUT);
    }
    Serial.println("Setup done...");
    delay(200);
}

void loop(){
    for(int i=0; i<5; i++){
        Serial.print(analogRead(sensorpin[i]));
        Serial.print("\t");
    }
    Serial.println();
    delay(500);
}