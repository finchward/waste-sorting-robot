  const int base_speed = 150; // 0 to 255
  const int pingInterval = 50;
  const float reverseThreshold = 15;

  const int pwmMotorA = 10;
  const int dirMotorA = 8;
  const int pwmMotorB = 9;
  const int dirMotorB = 7;

  const int trigPin = 5;
  const int echoPin = 6;

  bool isForward = true;
  unsigned long lastSwitchedSecond = -1;
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
  }

  void powerWheels(float leftStrength, float rightStrength){
    if (leftStrength > 0) digitalWrite(dirMotorA, HIGH); else digitalWrite(dirMotorA, LOW);
    if (rightStrength > 0) digitalWrite(dirMotorB, HIGH); else digitalWrite(dirMotorB, LOW);
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

  // get largest one, steer towards it.
  void loop() {
    getDistance();
    unsigned long t_sec = millis() / 1000;
    if (t_sec > 0 && t_sec % 5 == 0 && t_sec != lastSwitchedSecond) {
      isForward = !isForward;
      lastSwitchedSecond = t_sec;
    }
    if (currentDistance < reverseThreshold && currentDistance != -1) {
      powerWheels(0, 0);
    } else if (isForward) {
      powerWheels(1, 1);
    } else{
      powerWheels(-1, -1);
    }

    delay(5);
  }
