include <SPI.h>
include <nRF24L01.h>
include <RF24.h>

RF24 radio(9, 10); // CE, CSN pins

const byte address[6] = "00001"; // Address for the NRF24L01
int gasConcentration = 0; // Variable to store the received gas concentration

void setup() {
    Serial.begin(9600);
    radio.begin();
    radio.openReadingPipe(1, address);
    radio.startListening();
}

void loop() {
    if (radio.available()) {
        radio.read(&gasConcentration, sizeof(gasConcentration));
        Serial.print("Gas Concentration (MQ-2),");
        Serial.println(gasConcentration);
    }
}
