#include <NewPing.h> // library for sonar sensor

# define echoPinR 3      // Right Sensor
# define trigPinR 2      // Right Sensor
# define echoPinF 4      // Forward Sensor
# define trigPinF 5      // Forward Sensor
# define echoPinL 7      // Left Sensor
# define trigPinL 8      // Left Sensor
#define motorPin1 8      // Motor driver input 1
#define motorPin2 9      // Motor driver input 2
#define motorPin3 10     // Motor driver input 3
#define motorPin4 11     // Motor driver input 4
#define enablePin1 12    // Enable pin for motor 1
#define enablePin2 13    // Enable pin for motor 2

#define frontDistanceThreshold 5   // Threshold distance to detect obstacle (in centimeters)
#define sideDistanceThreshold 23 
#define turnDuration 805      // Duration for turning (in milliseconds)

NewPing sonarR(trigPinR, echoPinR, 400);
NewPing sonarF(trigPinF, echoPinF, 400);
NewPing sonarL(trigPinL, echoPinL, 400);

void setup() {
  // Initialize ultrasonic sensor pins
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  
  // Initialize motor pins
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT); 
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
}

void loop() {
  // Measure distance using ultrasonic sensors
  double frontDistance = sonarF.ping_cm();
  double leftDistance = sonarL.ping_cm();
  double rightDistance = sonarR.ping_cm();
  
  // set enable pins to HIGH
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);
  
  // LSRB pathfinding algorithm
  
  // if (leftDistance > sideDistanceThreshold) {
  //   // If left path is free, turn left
  //   turnLeft();
  //   delay(turnDuration);
  // } else if (frontDistance > frontDistanceThreshold) {
  //   // If left not free, go straight
  //   moveForward();
  // } else if (rightDistance > sideDistanceThreshold) {
  //   // If left and straight not free, turn right
  //   turnRight();
  //   delay(turnDuration);
  // } else {
  //   // If everything blocked, u turn
  //   uTurn();
  //   delay(2*turnDuration);
  // }

  // -- Forward Block == frontDistance < frontDistanceThreshold 
  // -- Forward Max == frontDistance > 90
  // -- Forward Reading == frontDistanceThreshold < frontDistance < 90

  // -- Right Block == rightDistance < sideDistanceThreshold
  // -- Right Max == rightDistance > 90
  // -- Right Reading == sideDistanceThreshold < rightDistance < 90

  // -- Left Block == leftDistance < sideDistanceThreshold
  // -- Left Max == leftDistance > 90
  // -- Left Reading == sideDistanceThreshold < leftDistance < 90



  if (frontDistance < frontDistanceThreshold & rightDistance < sideDistanceThreshold & leftDistance > 90) {
  
  // leftPivot90*
    one();
    delay(turnDuration);

  } else if (frontDistance > 90 & rightDistance < sideDistanceThreshold & sideDistanceThreshold < leftDistance < 90) {

  // leftPivot90*
    two();
    delay(turnDuration);


  } else if (frontDistance < frontDistanceThreshold & rightDistance < sideDistanceThreshold & sideDistanceThreshold < leftDistance < 90) {

  // leftPivot90*
    three();
    delay(turnDuration);

  } else if (frontDistanceThreshold < frontDistance < 90 & rightDistance < sideDistanceThreshold & leftDistance < sideDistanceThreshold) {
  
  // forwardDrive
    four();

  } else if (frontDistanceThreshold < frontDistance < 90 & rightDistance < sideDistanceThreshold & leftDistance > 90) {
  
  // forwardDrive
    five();

  } else if (frontDistance > 90 & rightDistance < sideDistanceThreshold & leftDistance > 90) {
  
  // forwardDrive
    six();

  } else if (frontDistanceThreshold < frontDistance < 90 & rightDistance > 90 & sideDistanceThreshold < leftDistance < 90) {
  
  // forwardDrive
    seven();

  } else if (frontDistanceThreshold < frontDistance < 90 & rightDistance < sideDistanceThreshold & sideDistanceThreshold < leftDistance < 90) {
  
  // forwardDrive
    eight();

  } else if (frontDistanceThreshold < frontDistance < 90 & sideDistanceThreshold < rightDistance < 90 & leftDistance < sideDistanceThreshold) {
  
  // forwardDrive
    nine();

  } else if (frontDistance < frontDistanceThreshold & sideDistanceThreshold < rightDistance < 90 & leftDistance < sideDistanceThreshold) {
  
  // rightTurn
    ten();
    delay(turnDuration);

  } else {

    uTurn();
    delay(2*turnDuration);

  }


}

void one() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void two() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void three() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void four() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void five() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void six() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void seven() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void eight() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void nine() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void ten() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}
  
void moveForward() {
  // Set motor direction for forward motion
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}

void turnLeft() {
  // Set motor direction for right turn
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}
  
void turnRight() {
  // Set motor direction for right turn
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}

void uTurn() {
  // Set motor direction for u turn
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
