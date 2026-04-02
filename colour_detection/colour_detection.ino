#include <Pixy2.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>  

const int centerMOE = 80;
const int minSize = 50;
const int base_speed = 150;
const int pingInterval = 50;

const int redLED = 4;
const int greenLED = 2; 
const int blueLED = 4;  //placeholder

const int pwmMotorA = 10;
const int dirMotorA = 8;
const int pwmMotorB = 9;
const int dirMotorB = 7;

const int trigPin = 12;
const int echoPin = 13;
long lastPingTime = 0;
float currentDistance = 0;

Pixy2 pixy;

bool isGrabbing = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(pwmMotorA, OUTPUT);
  pinMode(dirMotorA, OUTPUT);
  pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorB, OUTPUT);
  pixy.init();
}

String detectPrimaryBall(){
  int bestBlockIdx = -1;
  if (pixy.ccc.numBlocks) {

    for (int i = 0; i < pixy.ccc.numBlocks; i++){
      Block currentBlock = pixy.ccc.blocks[i];
      bool isCentered = (currentBlock.m_x > 156 - centerMOE) && (currentBlock.m_x < 157 + centerMOE);
      bool isCloseEnough = (currentBlock.m_width > minSize) && (currentBlock.m_height > minSize / 3);
      if (isCentered && isCloseEnough) {
        if (bestBlockIdx == -1 || currentBlock.m_width > pixy.ccc.blocks[bestBlockIdx].m_width){
          bestBlockIdx = i;
        }
      }
    }
    if (bestBlockIdx != -1) {
      int sig = pixy.ccc.blocks[bestBlockIdx].m_signature;
      if (sig == 1) return "red";
      if (sig == 2) return "green";
      if (sig == 3) return "blue";
    }
  }
  return "none";
}

void powerWheels(float leftStrength, float rightStrength){
  if (leftStrength > 0) digitalWrite(dirMotorA, LOW); else digitalWrite(dirMotorA, HIGH);
  if (rightStrength > 0) digitalWrite(dirMotorB, LOW); else digitalWrite(dirMotorB, HIGH);
  leftStrength = abs(leftStrength) * base_speed;
  rightStrength = abs(rightStrength) * base_speed;
  analogWrite(pwmMotorA, leftStrength);
  analogWrite(pwmMotorB, rightStrength);
  }



void getDistance(){
  long current_time = millis() + pingInterval;
  if (current_time - lastPingTime >= pingInterval) {
    lastPingTime = current_time;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000);
    if (duration == 0) {
      currentDistance = -1.0; 
    } else {
      currentDistance = (duration * 0.0343) / 2.0;
    }
    
    Serial.println(currentDistance);

  }
}

long grab_start_time;
long grab_end_time = -1001; 
void loop() {
  pixy.ccc.getBlocks();
  getDistance();

  String ball_detected = detectPrimaryBall();
  if (ball_detected == "red"){
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  } else if (ball_detected == "green") {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  } else if (ball_detected == "blue") {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
  } else {
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
  }

  if (ball_detected == "red" && !isGrabbing && millis() - grab_end_time > 8000){
    isGrabbing = true;
    grab_start_time = millis();
  }
  if (isGrabbing){
    if (millis() - grab_start_time < 3000){
      powerWheels(1, 1);
    }
    else{
      isGrabbing = false;
      powerWheels(0, 0);
      grab_end_time = millis();
    }
  }
  delay(10);
}
