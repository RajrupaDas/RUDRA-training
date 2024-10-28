#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "PIDpilot.h"

const int motorAPin = 9; // PWM pin for Motor A
const int motorBPin = 10; // PWM pin for Motor B
const int stopButtonPin = 2; 
const int receiverCEPin = 7; 
const int receiverCSNPin = 8; 
RF24 radio(receiverCEPin, receiverCSNPin);

float maxSpeed = 200; 
float safeSpeed = maxSpeed - 50; // Safety speed limit

float setpointA = 0; // Desired speed for A
float setpointB = 0; // Desired speed for B
float actualA = 0;   // Current speed for A
float actualB = 0;   // Current speed for B

float kp = 1.0; // PID gain for A
float ki = 0.5; // PID gain for A
float kd = 0.1; // PID gain for A
float outputA = 0; // PID output for A

float kpB = 1.0; // PID gain for B
float kiB = 0.5; // PID gain for B
float kdB = 0.1; // PID gain for B
float outputB = 0; // PID output for B

PIDpilot pidMotorA(kp, ki, kd);
PIDpilot pidMotorB(kpB, kiB, kdB);

void setup() {
    Serial.begin(9600);
    pinMode(motorAPin, OUTPUT);
    pinMode(motorBPin, OUTPUT);
    pinMode(stopButtonPin, INPUT_PULLUP);
    
    radio.begin();
    radio.openReadingPipe(1, 0xF0F0F0F0E1LL);
    radio.startListening();
}

void loop() {
    if (radio.available()) {
        int receivedData[2]; 
        radio.read(&receivedData, sizeof(receivedData));
        
        // Set the speed values based on received data
        setpointA = map(receivedData[0], 0, 1023, 0, safeSpeed); 
        setpointB = map(receivedData[1], 0, 1023, 0, safeSpeed); 
        
        // Compute PID outputs
        outputA = pidMotorA.compute(setpointA, actualA);
        outputB = pidMotorB.compute(setpointB, actualB);
        
        // Control motors based on PID output
        analogWrite(motorAPin, constrain(outputA, 0, 255)); 
        analogWrite(motorBPin, constrain(outputB, 0, 255)); /
    }

    // Stop motors if stop button is pressed
    if (digitalRead(stopButtonPin) == LOW) {
        analogWrite(motorAPin, 0);
        analogWrite(motorBPin, 0);
    }

    // Simulate actual speed feedback (this part should be replaced with actual feedback reading)
    actualA = analogRead(motorAPin);
    actualB = analogRead(motorBPin);
}
