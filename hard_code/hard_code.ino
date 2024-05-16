
#define motorPin1 8      // Motor driver input 1
#define motorPin2 9      // Motor driver input 2
#define motorPin3 10     // Motor driver input 3
#define motorPin4 11     // Motor driver input 4
#define enablePin1 12    // Enable pin for motor 1
#define enablePin2 13    // Enable pin for motor 2

#define turnDuration 805      // Duration for turning (in milliseconds)

void setup() {
  
  // Initialize motor pins
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT); 
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
}

void loop() {

  digitalWrite(enablePin1, LOW);
  digitalWrite(enablePin2, LOW);
  delay(2000);

  // set enable pins to HIGH
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);
  
  // hard code
  moveForward();
  delay(3000);

  /*turnLeft();
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
  */
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