#include <Pixy2.h>
#include <Servo.h>

const int base_speed = 120; // 0 to 255
const int pingInterval = 50;
const int minSize = 10; 
const float reverseDistThreshold = 15; //cm
const float ballDistThreshold = 5;
const float baseDistThreshold = 12;
const int maxBasePlateDepthDownPage = 140;
const float baseSenditThreshold = 30;
const float turnStrength = 0.8; //[0, 1]
const int clawTime = 1000;

const int ballOverrideWallStoppingThreshold = 20;

const int startingServoAngle = 70;

const int topScreenThreshold = 40;  // Max Y value to confirm the ball is in the claws
const int verifyReverseTime = 1000; // Milliseconds to reverse on failed grab
const int verifyRotateTime = 300;   // Milliseconds to rotate right on failed grab

const int startingTime = 2500;
const int reversingTime = 1000;
const int rammingTime = 2000;
const int rechargingTime = 3000;
const int spinningTime = 2000;

const int redLED = 2;
const int greenLED = 7;
const int blueLED = 9;  //placeholder

const int ballTrigPin = A1;
const int ballEchoPin = A0;

const int wallTrigPin = 4; //unset
const int wallEchoPin = 3; //unset

const int irPin = 11;

const int pwmMotorA = 6; //analog
const int dirMotorA = A2;
const int pwmMotorB = 5; //analog
const int dirMotorB = 8;

const int buttonPin = A4;

const int servoRightPin = A5; //analog
const int servoLeftPin = A3; //analog
Servo servoR;
Servo servoL;
const int clawOpenAngle = 170;
const int clawClosedAngle = 100;

const int orangePlateSig = 5;
const int pinkPlateSig = 4;
const int purplePlateSig = 6;
const int yellowPlateSig = 7;

const int basePlate = pinkPlateSig;

const int redBallSig = 1;
const int greenBallSig = 2;
const int blueBallSig = 3;

int stage = 1;
int desiredBallIdx = 0; //0 is red, 1 is green, 2 is blue
const bool collectMultipleColors = true;
Pixy2 pixy;
long lastPingTime = 0;
float ballCurrentDistance = 0;
float wallCurrentDistance = 0;

  
void openClaws(){
  servoR.write(clawOpenAngle);
  servoL.write(180 - clawOpenAngle);
  delay(clawTime);
}

void closeClaws(){
  servoR.write(clawClosedAngle);
  servoL.write(180 -clawClosedAngle);
  delay(clawTime);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

  pinMode(pwmMotorA, OUTPUT);
  pinMode(dirMotorA, OUTPUT);
  pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorB, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ballTrigPin, OUTPUT);
  pinMode(ballEchoPin, INPUT);
  pinMode(wallTrigPin, OUTPUT);
  pinMode(wallEchoPin, INPUT);

  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  servoR.attach(servoRightPin);
  servoL.attach(servoLeftPin);
  pixy.init();
}

bool headingTowardsBase = false;
void resetRobot() {
  // 1. Reset logic variables
  stage = 1;
  desiredBallIdx = 0;
  headingTowardsBase = false;
  
  // 2. Stop the motors immediately
  powerWheels(0, 0);
  
  // 3. Reset hardware state
  openClaws(); // or closeClaws(), depending on your preferred start state
  
  Serial.println("Robot Reset Triggered");
  delay(50); // Small debounce/safety delay
}

Block* getClosestBall(int desiredBallIdx){
  int bestBlockIdx = -1;

  int desiredSig;
  if (desiredBallIdx == 0){
    desiredSig = redBallSig;
  } else if (desiredBallIdx == 1){
    desiredSig = greenBallSig;
  } else if (desiredBallIdx == 2){
    desiredSig = blueBallSig;
  }

  if (pixy.ccc.numBlocks) {

    for (int i = 0; i < pixy.ccc.numBlocks; i++){
      Block currentBlock = pixy.ccc.blocks[i];
      bool isCloseEnough = (currentBlock.m_width > minSize) && (currentBlock.m_height > minSize) && currentBlock.m_y < maxBasePlateDepthDownPage;
      if (isCloseEnough) {
        if (currentBlock.m_signature == desiredSig && (bestBlockIdx == -1 || currentBlock.m_width > pixy.ccc.blocks[bestBlockIdx].m_width)){
          bestBlockIdx = i;
        }
      }
    }
    if (bestBlockIdx != -1) {
      return &(pixy.ccc.blocks[bestBlockIdx]);
    }
  }
  return nullptr;
}

Block* findBasePlate(){
  int bestBlockIdx = -1;
    if (pixy.ccc.numBlocks) {
      for (int i = 0; i < pixy.ccc.numBlocks; i++){
        Block currentBlock = pixy.ccc.blocks[i];
        if (currentBlock.m_signature == basePlate  && currentBlock.m_width > 15 && currentBlock.m_y < maxBasePlateDepthDownPage) {
          if (bestBlockIdx == -1 || currentBlock.m_width > pixy.ccc.blocks[bestBlockIdx].m_width){
            bestBlockIdx = i;
          }
        }
      }
      if (bestBlockIdx != -1) {
        Serial.println((float)pixy.ccc.blocks[bestBlockIdx].m_x / 315.0);
      return &(pixy.ccc.blocks[bestBlockIdx]);
    }
  }

  return nullptr;
}

void setLEDColor(int color) {
  // Turn everything off first (clean state)
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);

  if (color == 1) {
    digitalWrite(greenLED, HIGH);
  } else if (color == 2) {
    digitalWrite(blueLED, HIGH);
  } else if (color == 3) {
    digitalWrite(redLED, HIGH);
  }
}

void powerWheels(float leftStrength, float rightStrength) {

  // Determine directions based on input signs
  int dirA_state = (leftStrength > 0) ? LOW : HIGH;
  int dirB_state = (rightStrength > 0) ? HIGH : LOW;

  digitalWrite(dirMotorA, dirA_state);
  digitalWrite(dirMotorB, dirB_state);

  // Calculate final PWM values
  float leftPwm = fabs(leftStrength) * base_speed;
  float rightPwm = fabs(rightStrength) * base_speed;

  analogWrite(pwmMotorA, leftPwm);
  analogWrite(pwmMotorB, rightPwm);
}



void navigate_towards_block(Block block) {
  float x_norm = ((block.m_x / 315.0) - 0.5) * 2.0; // -1 coz camera flipped
  

  float leftTarget, rightTarget;

  if (x_norm < 0) {
    leftTarget = 1.0;
    rightTarget = (turnStrength * 2.0 * x_norm + 1.0);
  } else {
    leftTarget = -2.0 * x_norm * turnStrength + 1.0;
    rightTarget = 1.0;
  }

  powerWheels(leftTarget, rightTarget);
}

void getDistance() {
  long current_time = millis(); 

  // Check if enough time has passed since the last ping
  if (current_time - lastPingTime >= pingInterval) {
    lastPingTime = current_time;

    // -------------------------
    // 1. Read Ball Distance
    // -------------------------
    digitalWrite(ballTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ballTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ballTrigPin, LOW);

    long ballDuration = pulseIn(ballEchoPin, HIGH, 20000);
    if (ballDuration == 0) {
      ballCurrentDistance = -1.0; 
    } else {
      ballCurrentDistance = (ballDuration * 0.0343) / 2.0;
    }
    
    // -------------------------
    // 2. Read Wall Distance
    // -------------------------
    // A tiny delay between pings helps prevent the wall sensor 
    // from hearing the echo of the ball sensor
    delay(30); 
    
    digitalWrite(wallTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(wallTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(wallTrigPin, LOW);

    long wallDuration = pulseIn(wallEchoPin, HIGH, 20000);
    if (wallDuration == 0) {
      wallCurrentDistance = -1.0; 
    } else {
      wallCurrentDistance = (wallDuration * 0.0343) / 2.0;
    }

    Serial.println(ballCurrentDistance);
  }
}

bool verifyGrabSuccess(int desiredIdx) {
  int desiredSig;
  if (desiredIdx == 0) {
    desiredSig = redBallSig;
  } else if (desiredIdx == 1) {
    desiredSig = greenBallSig;
  } else if (desiredIdx == 2) {
    desiredSig = blueBallSig;
  }  
  bool grabSuccessful = false;
  
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      Block currentBlock = pixy.ccc.blocks[i];
      // Check if the block matches the signature AND is at the top of the screen
      if (currentBlock.m_signature == desiredSig && currentBlock.m_y <= topScreenThreshold) {
        grabSuccessful = true;
        break;
      }
    }
  }

  if (!grabSuccessful) {
    Serial.println("Grab failed! Retrying...");
    openClaws();
    
    // Reverse
    powerWheels(-1, -1);
    delay(verifyReverseTime);
    
    // Rotate Right
    powerWheels(1, -1);
    delay(verifyRotateTime);
    
    // Stop motors after the evasive maneuver
    powerWheels(0, 0); 
  }

  return grabSuccessful;
}

bool is_on = false;
unsigned long buttonHoldStartTime = 0; // Tracks when the 3-second countdown starts
bool buttonTiming = false;             // Keeps track of whether we are currently timing
// get largest one, steer towards it.
void loop() {
  if (!is_on) {
    servoR.write(startingServoAngle);
    servoL.write(180 - startingServoAngle);
    // Check if the button is HIGH
    if (digitalRead(buttonPin) == HIGH) {
      if (!buttonTiming) {
        // The button just went HIGH, start the timer
        buttonHoldStartTime = millis();
        buttonTiming = true;
      } else if (millis() - buttonHoldStartTime >= 3000) {
        // The button has been HIGH continuously for 3000ms (3 seconds)
        is_on = true; 
        setLEDColor(1);
        openClaws();
        Serial.println("Startup complete. Robot is ON!");
        powerWheels(1,1);
        delay(startingTime);
      }
    } else {
      // If the button drops back to LOW, reset the timer
      buttonTiming = false;
    }
    
    // Return early so the rest of the loop doesn't run until is_on is true
    return; 
  }

  pixy.ccc.getBlocks(); 
  getDistance();

  if (stage == 1){
    setLEDColor(1);
    Block* ball_detected = getClosestBall(desiredBallIdx);
    if ((wallCurrentDistance < reverseDistThreshold && wallCurrentDistance != -1) && !(ball_detected != nullptr && (*ball_detected).m_width > ballOverrideWallStoppingThreshold)){
        powerWheels(-1, -1);
    } else{
      if (ballCurrentDistance < ballDistThreshold && ballCurrentDistance != -1){
        Serial.println("Grabbing");
        setLEDColor(2);
        powerWheels(0, 0);
        closeClaws();

        if (verifyGrabSuccess(desiredBallIdx)) {
          stage = 2;
        }
      } else if (ball_detected == nullptr){
        powerWheels(1.2, -1.2);
      } else {
        navigate_towards_block(*ball_detected);
        }
    }
  } else if (stage == 2){
    setLEDColor(3);
    Block* base_detected = findBasePlate();
    if (headingTowardsBase == true && wallCurrentDistance < baseDistThreshold && wallCurrentDistance != -1) {
      Serial.println("Base arrived");
      openClaws();
      powerWheels(-1, -1);
      delay(reversingTime);
      powerWheels(0, 0);
      closeClaws();
      powerWheels(2, 2);
      delay(rammingTime);
      if (collectMultipleColors == true){
        desiredBallIdx = (desiredBallIdx + 1) % 3;
      }
      stage = 1;
      headingTowardsBase = false;
      powerWheels(-1, -1);
      delay(rechargingTime);
      openClaws();
    }
    else if (headingTowardsBase == true && wallCurrentDistance <= baseSenditThreshold && wallCurrentDistance != -1){
        Serial.println("Sending it towards base");
        powerWheels(1, 1);
    } else if (base_detected != nullptr){
      if (!headingTowardsBase){
        headingTowardsBase = true;
      }
      navigate_towards_block(*base_detected);
      Serial.println("Heading towards base in sight");
    }else {
      Serial.println("Finding base");
      powerWheels(1, -1);
      }
    // } else if (headingTowardsBase == true) {
    //   Serial.println("Blindly heading towards base");
    //   setLEDColor(3);
    //   powerWheels(1, 1);
    // } 
  
  }
}
