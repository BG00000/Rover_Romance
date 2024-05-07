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
#define turnDuration 1675      // Duration for turning (in milliseconds)

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
  
  // set enable pins to HIGH
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);
  
  // hard code
  moveForward();
  delay(_);

  turnLeft();
  delay(_);

  moveForward();
  delay();

  turnLeft();
  delay(_);

  moveForward();
  delay();

  turnRight();
  delay(_);

  moveForward();
  delay(_);

  turnLeft();
  delay(_);

  moveForward();
  delay(_);

  turnLeft();
  delay(_);

  moveForward();
  delay(_);

  turnRight();
  delay(_);

  moveForward();
  delay(_);
}

  void turnLeft() {
  // Set motor direction for left turn
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
  
  void moveForward() {
  // Set motor direction for forward motion
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}

void turnRight() {
  // Set motor direction for right turn
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}
  
  void uTurn() {
  // Set motor direction for u turn
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}