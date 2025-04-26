#include <Wire.h>
#include <VL53L0X.h>

/*******************************************************
 * PIN DEFINITIONS
 *******************************************************/
#define IN1 5
#define IN2 4
#define IN3 18
#define IN4 19
#define ENA 12
#define ENB 13

#define IR_LEFT 34
#define IR_RIGHT 35

#define ENCA_A 26
#define ENCA_B 27
#define ENCB_A 23
#define ENCB_B 25

VL53L0X lidar; // LiDAR sensor object

/*******************************************************
 * ENCODER VARIABLES & INTERRUPTS
 *******************************************************/
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;

// Interrupts to track encoder pulses for left motor
void IRAM_ATTR encoderAInterrupt() {
  if (digitalRead(ENCA_B)) pulseCountA++;
  else                     pulseCountA--;
}

void IRAM_ATTR encoderBInterrupt() {
  if (digitalRead(ENCA_A)) pulseCountA--;
  else                     pulseCountA++;
}

// Interrupts to track encoder pulses for right motor
void IRAM_ATTR encoderCInterrupt() {
  if (digitalRead(ENCB_B)) pulseCountB++;
  else                     pulseCountB--;
}

void IRAM_ATTR encoderDInterrupt() {
  if (digitalRead(ENCB_A)) pulseCountB--;
  else                     pulseCountB++;
}

/*******************************************************
 * MOTOR CONTROL FUNCTIONS
 *******************************************************/

// Drive both motors forward with independent speeds
void drive(int speedLeft, int speedRight) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speedLeft);
  analogWrite(ENB, speedRight);
}

// Stop all motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Motors Stopped.");
}

// Turn left in place
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
}

// Turn right in place
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
}

// Turn left by 90 degrees (calibrated with delay)
void turnLeft90() {
  turnLeft();
  delay(400);  // Adjust timing as needed
  stopMotors();
}

// Turn right by 90 degrees
void turnRight90() {
  turnRight();
  delay(400);  // Adjust timing as needed
  stopMotors();
}

/*******************************************************
 * SETUP
 *******************************************************/
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Encoder pins
  pinMode(ENCA_A, INPUT);
  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_A, INPUT);
  pinMode(ENCB_B, INPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), encoderBInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderCInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), encoderDInterrupt, CHANGE);

  // Initialize LiDAR
  Wire.begin();
  lidar.init();
  lidar.setTimeout(500);

  Serial.println("Setup complete. Robot starting...");
}

/*******************************************************
 * MAIN LOOP (Real-Time Maze Adaptation)
 *******************************************************/
void loop() {
  int distanceMM = lidar.readRangeSingleMillimeters();
  float distanceCM = distanceMM / 10.0;

  bool leftWall  = digitalRead(IR_LEFT);  // 1 = wall close on left
  bool rightWall = digitalRead(IR_RIGHT); // 1 = wall close on right

  int baseSpeed = 80;
  int speedLeft = baseSpeed;
  int speedRight = baseSpeed;

  // Encoder feedback correction for straight motion
  long error = pulseCountA - pulseCountB;
  int correction = abs(error) / 2; 

  if (error > 0) { 
    speedRight -= correction;
  } else if (error < 0) { 
    speedLeft -= correction;
  }

  // Obstacle detected ahead
  if (distanceCM < 20) {
    stopMotors();
    delay(200);

    if (leftWall && !rightWall) {
      Serial.println("Obstacle ahead + left wall -> Turning right");
      turnRight90();
    } else if (rightWall && !leftWall) {
      Serial.println("Obstacle ahead + right wall -> Turning left");
      turnLeft90();
    } else {
      Serial.println("Obstacle ahead -> Default turning right");
      turnRight90();
    }
    return;
  }

  // Wall following corrections
  if (leftWall) {
    Serial.println("LeftWall detected, nudging right...");
    speedLeft -= 5;
  }
  if (rightWall) {
    Serial.println("RightWall detected, nudging left...");
    speedRight -= 5;
  }

  // Safety limits on speeds
  speedLeft  = constrain(speedLeft, 60, 90);
  speedRight = constrain(speedRight, 60, 90);

  // Drive motors with corrected speeds
  drive(speedLeft, speedRight);

  // Debug output
  Serial.print("FrontDist: ");
  Serial.print(distanceCM);
  Serial.print("cm | LWall=");
  Serial.print(leftWall);
  Serial.print(" | RWall=");
  Serial.print(rightWall);
  Serial.print(" | speedL=");
  Serial.print(speedLeft);
  Serial.print(" | speedR=");
  Serial.print(speedRight);
  Serial.print(" | encA=");
  Serial.print(pulseCountA);
  Serial.print(" | encB=");
  Serial.println(pulseCountB);

  delay(50);  // Short loop delay
}
