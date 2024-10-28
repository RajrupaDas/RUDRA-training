#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);
const byte address[6] = "00001";

const int xPin = A0;
const int yPin = A1;
const int stopButtonPin = 2;

struct JoystickData {
    int x;
    int y;
    bool stop;
} data;

void setup() {
    pinMode(stopButtonPin, INPUT_PULLUP);
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.stopListening();
}

void loop() {
    int rawX = analogRead(xPin) - 512;
    int rawY = analogRead(yPin) - 512;
    data.x = constrain(rawX, -512, 512);
    data.y = constrain(rawY, -512, 512);

    data.stop = digitalRead(stopButtonPin) == LOW;

    radio.write(&data, sizeof(data));
    delay(50);
}
