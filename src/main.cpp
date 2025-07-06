#include <Arduino.h>

// motor pin defines
#define lmf 10 //IN1
#define lmb 9 //IN2
#define rmf 8 //IN3
#define rmb 7 //IN4
#define lms 6 //EnA
#define rms 5 //EnB

// declaration of necessary variable to line follow
const int refPosition = 3; // ideal position is the 4th IR on black line
const int sensorpinnumber = 5; // number of IR pins
const int sensorpin[sensorpinnumber] = {A0, A1, A2, A3, A4}; // pin number attached to each IR sensor
int s[sensorpinnumber], sum; // sum of active sensors
int position[sensorpinnumber] = { 1, 2, 3, 4, 5}; // weight of each position
int sensor; // sensor stores the weight of active sensors
int lbase = 225, rbase = 225; // base speed for each motor
int lmotor, rmotor; // speed of left and right motor
float avg = 0; // avg = ir[0]*1 + ir[1]*2 .... + ir[6]*7 / ir[0]+ir[1]+...+ir[6]
int kp = 40; // separate proportional gain for left and right motors
int kd = 0; // derivative gain
int PID; // separate PID values for left and right motors
float error, last_error;
char turn = 's';
int tsp = 175;  // turn speed
int max_speed = 250;
int threshold[5] = {500, 500, 500, 500, 500};

// motor driver function
void motor(int a, int b) {
    if (a > 0) {
        digitalWrite(lmf, 1);
        digitalWrite(lmb, 0);
    } 
    else {
        a = -a;
        digitalWrite(lmf, 0);
        digitalWrite(lmb, 1);
    }
    if (b > 0) {
        digitalWrite(rmf, 0);
        digitalWrite(rmb, 1);
    } 
    else {
        b = -b;
        digitalWrite(rmf, 1);
        digitalWrite(rmb, 0);
    }   
    if (a > max_speed) a = max_speed;
    if (b > max_speed) b = max_speed;

    analogWrite(lms, a);
    analogWrite(rms, b);
}

// gives an average position of the IR array
void PID_reading() {
    sensor = 0;
    sum = 0;
    for (byte i = 0; i < sensorpinnumber; i++) {
        s[i] = analogRead(sensorpin[i]);
        // s[i] = map(s[i], 0, 1023, 0, 2000); // Mapping sensor values to a larger range

        s[i] = (s[i] > threshold[i]) ? 0 : 1; // Binary thresholding
        sensor += s[i] * position[i];
        sum += s[i];
    }
    if (sum) avg = sensor / (sum * 1.0); // Average value
}

void reading() {
    sensor = 0;
    sum = 0;
    for (byte i = 0; i < sensorpinnumber; i++) {
        s[i] = analogRead(sensorpin[i]);
        // s[i] = map(s[i], 0, 1023, 0, 2000); // Mapping sensor values to a larger range

        s[i] = (s[i] > threshold[i]) ? 0 : 1; // Binary thresholding
        sensor += s[i] * position[i];
        sum += s[i];
    }
}


void PID_LINE_FOLLOW() {
    while (1) {
        reading();
        if (sum == 0) {
            if (turn != 's') {
                delay(15); // Slight delay for stability
                motor(0, 0);
                (turn == 'r') ? motor(tsp, -tsp*.5) : motor(-tsp*.5, tsp);
                
                int stableReadCount = 0;  // Counter for stable readings
                while (stableReadCount < 5) {  // Require 5 consecutive stable readings
                    reading();
                    if (s[3] && (!s[2] || !s[4])) stableReadCount++;  // Increment if sensors detect the line
                    else stableReadCount = 0;  // Reset if any sensor fails
                }
                turn = 's';
            }
        }

        PID_reading();

        for(int i=0; i<5; i++){
            Serial.print(s[i]);
            Serial.print("\t");
        }
        Serial.println();

        error = refPosition - avg;
        PID = (0.3 * PID) + (0.7 * (error * kp + kd * (error - last_error)));

        last_error = error; 
        lmotor = lbase - PID;
        rmotor = rbase + PID;


        // for debugging
        // Serial.print("L motor: ");
        // Serial.print(lmotor);
        // Serial.print("\t");
        // Serial.print("R motor: ");
        // Serial.print(rmotor);
        // Serial.println();


        motor(lmotor, rmotor);  

        if (s[0] && !s[4]) turn = 'l';
        if (s[4] && !s[0]) turn = 'r';
        else if (sum == 5) {
            delay(20);
            reading();
            if (sum == 5) {
                motor(0, 0);
                while (sum == 5) reading();
            }
            else if (sum == 0) turn = 'r';
        }
    }
}

void setup() {
    Serial.begin(9600);

    int inpin[5] = {A0, A1, A2, A3, A4};
    int outpin[6] = {rmf, rmb, lmf, lmb, rms, lms};

    // set pin as input
    for (int i = 0; i < 5; i++) {
        pinMode(inpin[i], INPUT);
    }

    // set pin as output
    for (int i = 0; i < 6; i++) {
        pinMode(outpin[i], OUTPUT);
    }

    delay(250);
}

void loop() {
    PID_LINE_FOLLOW(); // line follow using PID
}