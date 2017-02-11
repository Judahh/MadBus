#include <Servo.h>

Servo myServo;  // create a servo object

//int const potentiometer = A0; // analog pin used to connect the potentiometer

int const serialBaudRate = 9600;
long const microsecondsToCentimetersFactor = 56.88;
int const distanceIn = 3;
int const distanceOut = 2;
int const motorF = 8; 
int const motorR = 7;
int const servo = 9; 
int const wL = 12; 
int const wR = 13; 


void setup() {
  pinMode(distanceIn, INPUT);
  pinMode(distanceOut, OUTPUT);
  pinMode(motorF, OUTPUT);
  pinMode(motorR, OUTPUT);
  pinMode(wL, OUTPUT);
  pinMode(wR, OUTPUT);
  digitalWrite(motorF, 0);
  digitalWrite(motorR, 0);
  digitalWrite(wL, 0);
  digitalWrite(wR, 0);
  myServo.attach(servo); // attaches the servo on pin 9 to the servo object
  Serial.begin(serialBaudRate); // open a serial connection to your computer
}

void loop() {
  int potVal;  // variable to read the value from the analog pin
  long distVal,duration;
  int angle=0;   // variable to hold the angle for the servo motor
  
  Serial.print("START: \n");
  
  //pinMode(distanceOut, OUTPUT);
  //digitalWrite(distanceOut, LOW);
  //delayMicroseconds(2);
  //digitalWrite(distanceOut, HIGH);
  //delayMicroseconds(5);
  //digitalWrite(distanceOut, LOW);
  
  //pinMode(distanceOut, INPUT);
  //duration = pulseIn(distanceOut, HIGH);
  //distVal = microsecondsToCentimeters(duration);
  
  //Serial.print("Dist: ");
  //Serial.print(distVal);
  //Serial.print("\n");

  // set the servo position
  
  myServo.write(angle);

  // wait for the servo to get there
  delay(15);
  angle=190;
  myServo.write(angle);
  delay(15);
  //turn(1,1);
  //run(1,1);
  //delay(1000);
  //turn(1,0);
  //run(1,0);
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / microsecondsToCentimetersFactor;
}

void turn(bool on,bool right){
  if(on){
    digitalWrite(wR, right);
    digitalWrite(wL, !right);
  }else{
    digitalWrite(wR, 0);
    digitalWrite(wL, 0);
  }
}

void run(bool on,bool forward){
  if(on){
    digitalWrite(motorF, forward);
    digitalWrite(motorR, !forward);
  }else{
    digitalWrite(motorF, 0);
    digitalWrite(motorR, 0);
  }
}
