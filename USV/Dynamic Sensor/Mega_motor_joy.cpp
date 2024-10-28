#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "PIDpilot.h"

const int motorAPin = 9; // PWM pin for Motor A
const int motorBPin = 10; // PWM pin for Motor B
const int stopButtonPin = 2; // Pin for the stop button
const int receiverCEPin = 7; // CE pin for nRF module
const int receiverCSNPin = 8; // CSN pin for nRF module
RF24 radio(receiverCEPin, receiverCSNPin);

float maxSpeed = 200; // Adjust as necessary
float safeSpeed = maxSpeed - 50; // Safety speed limit

float setpointA = 0; // Desired speed for Motor A
float setpointB = 0; // Desired speed for Motor B
float actualA = 0;   // Current speed for Motor A
float actualB = 0;   // Current speed for Motor B

float kp = 1.0; // PID gain for Motor A
float ki = 0.5; // PID gain for Motor A
float kd = 0.1; // PID gain for Motor A
float outputA = 0; // PID output for Motor A

float kpB = 1.0; // PID gain for Motor B
float kiB = 0.5; // PID gain for Motor B
float kdB = 0.1; // PID gain for Motor B
float outputB = 0; // PID output for Motor B

PIDController pidMotorA(kp, ki, kd);
PIDController pidMotorB(kpB, kiB, kdB);

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
        int receivedData[2]; // Assuming two values received for Motor A and B
        radio.read(&receivedData, sizeof(receivedData));
        
        // Set the speed values based on received data
        setpointA = map(receivedData[0], 0, 1023, 0, safeSpeed); // Adjust mapping as needed
        setpointB = map(receivedData[1], 0, 1023, 0, safeSpeed); // Adjust mapping as needed
        
        // Compute PID outputs
        outputA = pidMotorA.compute(setpointA, actualA);
        outputB = pidMotorB.compute(setpointB, actualB);
        
        // Control motors based on PID output
        analogWrite(motorAPin, constrain(outputA, 0, 255)); // Constrain the output to valid PWM range
        analogWrite(motorBPin, constrain(outputB, 0, 255)); // Constrain the output to valid PWM range
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
