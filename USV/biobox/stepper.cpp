const int stepPin= 2;   //step pin
const int dirPin =3;    //direction pin
const int enablePin= 8; // enable pin  

const int stepsPerRevolution = 200; 
int delayBetweenTurns = 5000;  //milliseconds
int angle =36;   //step angle

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  digitalWrite(enablePin, LOW);   //driver enabled
}

void loop() {
  int stepsPerAngle= map(angle, 0, 360, 0, stepsPerRevolution); //angle to steps
  
  for (int i=0; i<5; i++) {
    
    digitalWrite(dirPin, HIGH); //direction setting
    for (int j = 0; j < stepsPerAngle; j++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);    //puls width (can be adjusted)
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }

    delay(delayBetweenTurns);
  }
  
  digitalWrite(enablePin, HIGH);  //stop driver
  while (true);//no further execution
}
