#include <NewPing.h> // library for sonar sensor


# define echoPinR 3      // Right Sensor
# define trigPinR 2      // Right Sensor
# define echoPinF 4      // Forward Sensor
# define trigPinF 5      // Forward Sensor
# define echoPinL 7      // Left Sensor
# define trigPinL 6      // Left Sensor
#define motorPin1 8      // Motor driver input 1
#define motorPin2 9      // Motor driver input 2
#define motorPin3 10     // Motor driver input 3
#define motorPin4 11     // Motor driver input 4
#define enablePin1 12    // Enable pin for motor 1
#define enablePin2 13    // Enable pin for motor 2


#define frontDistanceThreshold 2   // Threshold distance to detect obstacle (in centimeters)
#define sideDistanceThreshold 23
#define turnDuration 805      // Duration for turning (in milliseconds)


NewPing sonarR(trigPinR, echoPinR, 400);
NewPing sonarF(trigPinF, echoPinF, 400);
NewPing sonarL(trigPinL, echoPinL, 400);


void setup() {


  Serial.begin(960)
  // Initialize ultrasonic sensor pins
  // Serial.begin(9600);
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
  if (leftDistance > sideDistanceThreshold) {
   unsigned long startTime = millis();
   while (startTime < 5000) { // Replace 5000 with the time it takes to travel 2 tiles
      moveForward();  // Move forward while checking
      delay(100);      // Short delay to maintain responsiveness
      // Update front distance inside the loop
      if (frontDistance < frontDistanceThreshold) {
        break;  // Break if an obstacle is detected
      }
    turnLeft();
   }
  } else if (frontDistance > frontDistanceThreshold) {
    // If left not free, go straight
    moveForward();
  } else if (rightDistance > sideDistanceThreshold) {
    // If left and straight not free, turn right
    delay(2000);
    turnRight();
    delay(turnDuration);
  } else {
    // If everything blocked, u turn
    uTurn();
    delay(2*turnDuration);
  }
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


 //  if (leftDistance > sideDistanceThreshold) {
 //    // If left path is free, turn left
 //    delay(2000);
 //    turnLeft();
 //    delay(turnDuration);

