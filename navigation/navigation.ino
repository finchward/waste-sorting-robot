  #include <Pixy2.h>

  const int base_speed = 100; // 0 to 255
  const int pingInterval = 50;
  const int centerMOE = 80;
  const int minSizeForStage1 = 130;
  const int minSize = 30; 
  const float reverseThreshold = 15; //cm

  const int redLED = 2;
  const int greenLED = 2;
  const int blueLED = 4;  //placeholder

  const int trigPin = 5;
  const int echoPin = 6;
  const int irPin = 11;

  const int pwmMotorA = 10;
  const int dirMotorA = 8;
  const int pwmMotorB = 9;
  const int dirMotorB = 7;

  const int basePlateSignature = 4;

  int stage = 0;
  Pixy2 pixy;
  long lastPingTime = 0;
  float currentDistance = 0;

  void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600); 

    pinMode(pwmMotorA, OUTPUT);
    pinMode(dirMotorA, OUTPUT);
    pinMode(pwmMotorB, OUTPUT);
    pinMode(dirMotorB, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pixy.init();
  }

  Block* getClosestBall(){
    int bestBlockIdx = -1;
    if (pixy.ccc.numBlocks) {

      for (int i = 0; i < pixy.ccc.numBlocks; i++){
        Block currentBlock = pixy.ccc.blocks[i];
        bool isCloseEnough = (currentBlock.m_width > minSize) && (currentBlock.m_height > minSize);
        if (isCloseEnough) {
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

  void powerWheels(float leftStrength, float rightStrength){
    if (leftStrength > 0) digitalWrite(dirMotorA, LOW); else digitalWrite(dirMotorA, HIGH);
    if (rightStrength > 0) digitalWrite(dirMotorB, LOW); else digitalWrite(dirMotorB, HIGH);
    leftStrength = abs(leftStrength) * base_speed;
    rightStrength = abs(rightStrength) * base_speed;
    analogWrite(pwmMotorA, leftStrength);
    analogWrite(pwmMotorB, rightStrength);
    }

  void roam(){
    //go forward, if we are approaching wall reverse and turn.
    powerWheels(1, 1);
    // Serial.println(currentDistance);
    // if (currentDistance > reverseThreshold || currentDistance == -1){
    //   powerWheels(1, 1);
    // } else {
    //   powerWheels(0, -1);
    // }
  }

  void navigate_towards_block(Block block){
    float x_norm = ((block.m_x / 315.0) - 0.5) * 2.0 * -1; // -1 coz camera flipped
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

unsigned long timeSinceFinish = 0;

bool headingTowardsBase = false;
unsigned long timeSinceBaseSeen;
  // get largest one, steer towards it.
  void loop() {
    pixy.ccc.getBlocks();
    getDistance();

    if (stage == 0){
      Block* ball_detected = getClosestBall();
      if (ball_detected == nullptr){
        roam();
      // } else if ((*ball_detected).m_width > minSizeForStage1) {
      //     powerWheels(0, 0);
      //     stage = 1;
      //     timeSinceFinish = millis();
      // } else {
      } else if (currentDistance < 10){
        powerWheels(0, 0);
        stage = 1;
        timeSinceFinish = millis();
      } else {
        navigate_towards_block(*ball_detected);
      }
    } else if (stage == 1){
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
      }
      if (bestBlockIdx != -1) {
        if (!headingTowardsBase){
          headingTowardsBase = true;
          timeSinceBaseSeen = millis();
        }
        navigate_towards_block(pixy.ccc.blocks[bestBlockIdx]);
      } else if (headingTowardsBase == true) {
        powerWheels(1, 1);
        if (millis() - timeSinceBaseSeen > 10000){
          powerWheels(0, 0);
          stage = 2;
          timeSinceFinish = millis();
        }
      } else {
        powerWheels(1, -0.9); // spin until we see it
      }
    } else {
      if (millis() - timeSinceFinish > 10000) {
        stage = 0;
        timeSinceFinish = 0;
        headingTowardsBase = false;
      }

    }

  }
