const int mq2Pin= A0;
const float calibrationFactor= 5.0; 

void setup() {
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  int sensorValue= analogRead(mq2Pin);
  float voltage= sensorValue *(5.0 /1023.0);
  float concentration= voltage *calibrationFactor;
  float concentration_ratio= concentration /1.9;
  
  Serial.println(concentration);
  delay(500);
}
