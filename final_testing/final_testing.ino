    #include <Pixy2.h>
    #include <Servo.h>

    const int base_speed = 100; // 0 to 255
    const int pingInterval = 50;
    const int centerMOE = 80;
    const int minSizeForStage1 = 130;
    const int minSize = 30; 
    const float reverseDistThreshold = 15; //cm
    const float ballDistThreshold = 10;
    const float baseDistThreshold = 15;
    const int clawTime = 1000;
    const int rammingTime = 500;

    const int redLED = 2;
    const int greenLED = 7;
    const int blueLED = 9;  //placeholder

    const int trigPin = A1;
    const int echoPin = A0;

    const int hindTrigPin = 5; //unset
    const int hindEchoPin = 6; //unset

    const int irPin = 11;

    const int pwmMotorA = 6; //analog
    const int dirMotorA = A2;
    const int pwmMotorB = 5; //analog
    const int dirMotorB = 8;

    const int buttonPin = 4;

    const int servoRightPin = A5; //analog
    const int servoLeftPin = A3; //analog
    Servo servoR;
    Servo servoL;
    const int clawOpenAngle = 30;
    const int clawClosedAngle = 150;

    const int orangePlateSig = 4;
    const int pinkPlateSig = 4;
    const int purplePlateSig = 4;
    const int yellowPlateSig = 4;

    const int basePlate = pinkPlateSig;

    const int redBallSig = 1;
    const int greenBallSig = 2;
    const int blueBallSig = 3;

    int stage = 1;
    int desiredBallIdx = 0;
    Pixy2 pixy;
    long lastPingTime = 0;
    float currentDistance = 0;

      
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
      pinMode(buttonPin, INPUT);
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      servoR.attach(servoRightPin);
      servoL.attach(servoLeftPin);
      pixy.init();
      openClaws();
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
      delay(500); // Small debounce/safety delay
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
          bool isCloseEnough = (currentBlock.m_width > minSize) && (currentBlock.m_height > minSize);
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
            if (currentBlock.m_signature == 4  && currentBlock.m_width > 15) {
              if (bestBlockIdx == -1 || currentBlock.m_width > pixy.ccc.blocks[bestBlockIdx].m_width){
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

    void powerWheels(float leftStrength, float rightStrength){
      if (leftStrength > 0) digitalWrite(dirMotorA, LOW); else digitalWrite(dirMotorA, HIGH);
      if (rightStrength > 0) digitalWrite(dirMotorB, LOW); else digitalWrite(dirMotorB, HIGH);
      leftStrength = abs(leftStrength) * base_speed;
      rightStrength = abs(rightStrength) * base_speed;
      analogWrite(pwmMotorA, leftStrength);
      analogWrite(pwmMotorB, rightStrength);
      }

    void roam(){
      if (currentDistance > reverseDistThreshold || currentDistance == -1){
        powerWheels(1, 1);
      } else {
        powerWheels(1, -1);
      }
    }

    void navigate_towards_block(Block block){
      float x_norm = ((block.m_x / 315.0) - 0.5) * 2.0; // -1 coz camera flipped
      if (x_norm < 0) {
        powerWheels(1, 2 * x_norm + 1);
      } else {
        powerWheels(-2 * x_norm + 1, 1);
      }
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

    // get largest one, steer towards it.
    void loop() {
      if (digitalRead(buttonPin) == HIGH) {
        resetRobot();
      }
      pixy.ccc.getBlocks(); 
      getDistance();

      if (stage == 1){
        setLEDColor(1);
        Block* ball_detected = getClosestBall(desiredBallIdx);
        if (ball_detected == nullptr){
          roam();
        } else if (currentDistance < ballDistThreshold && currentDistance != -1){
          setLEDColor(2);
          powerWheels(0, 0);
          closeClaws();
          stage = 2;
        } else {
          navigate_towards_block(*ball_detected);
          }
      } else if (stage == 2){
        Block* base_detected = findBasePlate();
        setLEDColor(3);
        if (headingTowardsBase == true){
          if (currentDistance < baseDistThreshold) {
            openClaws();
            powerWheels(1, 1);
            delay(rammingTime);
            powerWheels(0, 0);
            delay(rammingTime);
            desiredBallIdx = (desiredBallIdx + 1) % 3;
            stage = 1;
            headingTowardsBase = false;
          }
        }

        if (base_detected != nullptr){
          if (!headingTowardsBase){
            headingTowardsBase = true;
          }
          navigate_towards_block(*base_detected);
        } else if (headingTowardsBase == true) {
          powerWheels(1, 1);
        } else {
          powerWheels(1, -0.9);
        }

      }

    }
