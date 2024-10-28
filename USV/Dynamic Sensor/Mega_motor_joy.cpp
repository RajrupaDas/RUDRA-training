include <SPI.h>
include <nRF24L01.h>
include <RF24.h>

const int motorPinA = 5;
const int motorPinB = 6;
const int maxSpeed = 220;
const int stopDelay = 10;

RF24 radio(7, 8); 
const byte address[6] = "00001";

struct JoystickData {
    int x;
    int y;
    bool stop;
} data;

void setup() {
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();
}

void loop() {
    if (radio.available()) {
        radio.read(&data, sizeof(data));
        
        if (data.stop) {
            for (int speed = analogRead(motorPinA); speed > 0; speed--) {
                analogWrite(motorPinA, speed);
                analogWrite(motorPinB, speed);
                delay(stopDelay);
            }
        } else {
            int speedA = map(data.y, -512, 512, 0, maxSpeed);
            int speedB = map(data.x, -512, 512, 0, maxSpeed);
            analogWrite(motorPinA, speedA);
            analogWrite(motorPinB, speedB);
        }
    }
}
