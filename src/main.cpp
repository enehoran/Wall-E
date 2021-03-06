#include <Arduino.h>
#include <Metro.h>

/*-----------------------------Module Defines-----------------------------*/
#define IR_BEACON_FREQUENCY     1000
#define GAME_TIMER_INTERVAL     50000
#define ROTATE_PI_INTERVAL      3000
#define OB_BACK_INTERVAL        500
#define OB_ROTATE_INTERVAL      1200
#define REVERSE_INTERVAL        500
#define MOTOR_FULL_SPEED        255
#define MOTOR_HALF_SPEED        124
#define MOTOR_SLOW_SPEED        90
#define MOTOR_STOP              0
#define LINE_THRESHOLD          150

/*-----------------------Module Function Prototypes-----------------------*/
void checkGlobalEvents(void);
void handleIdle(void);
void handleLocalize(void);
void handleAlign(void);
void handleMoveForward(void);
void handleOutOfBounds(void);
void handlePushing(void);
void handleReversing(void);
void handleSwitching(void);
void activateSpeed(int value);
void handleOutOfBoundsRotate(void);
void handleOutOfBoundsBackup(void);
void handleOutOfBoundsForward(void);
void gameFinished(void);
void alignmentFinished(void);
void outOfBoundsFinished(void);


/*---------------------------State Definitions----------------------------*/
typedef enum {
  STATE_IDLE,
  STATE_LOCALIZE,
  STATE_ALIGN,
  STATE_FORWARD,
  STATE_OUT_OB_ROT,
  STATE_OUT_OB_BACK,
  STATE_OUT_OB_FORWARD,
  STATE_PUSHING,
} States_t;

/*----------------------------Module Variables----------------------------*/
States_t state;
static Metro gameTimer = Metro(GAME_TIMER_INTERVAL);
static Metro alignTimer = Metro(ROTATE_PI_INTERVAL);
static Metro outOfBoundsRotateTimer = Metro(OB_ROTATE_INTERVAL);
static Metro outOfBoundsBackwardForwardTimer = Metro(OB_BACK_INTERVAL);
static Metro reverseTimer = Metro(REVERSE_INTERVAL);
static Metro switchTimer = Metro(ROTATE_PI_INTERVAL * 1.5);
IntervalTimer inputFrequencyTimer;

bool gameActive = true;

const uint8_t tapeIR_FRONT_LEFT = 23;          // Front Left Tape Sensor
const uint8_t tapeIR_FRONT_RIGHT = 21;         // Front Right Tape Sensor
const uint8_t tapeIR_BACK_LEFT = 22;           // Back Left Tape Sensor
const uint8_t tapeIR_BACK_RIGHT = 20;          // Back Right Tape Sensor

const uint8_t senseIR_1 = 14;                  // IR Beacon Sensor

const uint8_t rightMotors_IN1_IN3 = 0;         // Logic 1
const uint8_t rightMotors_IN2_IN4 = 1;         // Logic 2
const uint8_t rightMotorFront_EnA = 3;         // PWM Speed Control
const uint8_t rightMotorBack_EnB = 4;          // PWM Speed Control

const uint8_t leftMotors_IN1_IN3 = 5;          // Logic 1
const uint8_t leftMotors_IN2_IN4 = 6;          // Logic 2
const uint8_t leftMotorFront_EnA = 9;          // PWM Speed Control
const uint8_t leftMotorBack_EnB = 10;          // PWM Speed Control

const uint8_t limit_1 = 7;                     // Limit Switch 1
const uint8_t limit_2 = 8;                     // Limit Switch 2

const uint8_t COMM_IN = 11;                    // Input from other bot
const uint8_t COMM_OUT = 12;                   // Output to other bot

int key;                                       // Motor Test Input
int limit1Input;                               // First limit switch input reading
int limit2Input;                               // Second limit switch input reading
int tapeInput1;                                // Left tape sensor input reading
int tapeInput2;                                // Right tape sensor input reading
int measuredIRFrequency = 0;                   // IR sensor input reading

volatile uint16_t edgeCount = 0;               // Track number of falling waves in waveform
volatile float irMeasurementFrequency = 10000; // Calculate every 10,000 micro-seconds
float irFrequencyPrecision = 20;               // Need to calibrate

bool rotationDirection = 'R';                  

/*-----------------------------Main Functions-----------------------------*/

void countFallingEdges() {
  edgeCount++;
}

void recordMeasuredIRFrequency() {
  noInterrupts();
  measuredIRFrequency = edgeCount * 100;
  edgeCount = 0;
  interrupts();
}

void setup() {
  Serial.begin(9300);
  //while(!Serial);

  state = STATE_IDLE;

  pinMode(tapeIR_FRONT_LEFT, INPUT);            // Set Pin 23 as tape sensor input
  pinMode(tapeIR_FRONT_RIGHT, INPUT);           // Set Pin 23 as tape sensor input
  pinMode(tapeIR_BACK_LEFT, INPUT);             // Set Pin 23 as tape sensor input
  pinMode(tapeIR_BACK_RIGHT, INPUT);            // Set Pin 23 as tape sensor input

  pinMode(senseIR_1, INPUT);                    // Set Pin 14 as IR beacon sensor

  pinMode(rightMotors_IN1_IN3, OUTPUT);         // Set Pin 0 as Right Motor Logic Input 1
  pinMode(rightMotors_IN2_IN4, OUTPUT);         // Set Pin 1 as Right Motor Logic Input 2
  pinMode(rightMotorFront_EnA, OUTPUT);         // Set Pin 3 as Right Front Motor PWM Speed Control
  pinMode(rightMotorBack_EnB, OUTPUT);          // Set Pin 4 as Right Back Motor PWM Speed Control

  pinMode(leftMotors_IN1_IN3, OUTPUT);          // Set Pin 5 as Left Motor Logic Input 1
  pinMode(leftMotors_IN2_IN4, OUTPUT);          // Set Pin 6 as Left Motor Logic Input 2
  pinMode(leftMotorFront_EnA, OUTPUT);          // Set Pin 9 as Left Front Motor PWM Speed Control
  pinMode(leftMotorBack_EnB, OUTPUT);           // Set Pin 10 as Left Back Motor PWM Speed Control

  pinMode(limit_1, INPUT);                      // Set Pin 7 as Limit Switch 1
  pinMode(limit_2, INPUT);                      // Set Pin 8 as Limit Switch 2

  attachInterrupt(digitalPinToInterrupt(senseIR_1), countFallingEdges, FALLING);
  inputFrequencyTimer.begin(recordMeasuredIRFrequency, irMeasurementFrequency);
}

void endGame() {
  state = STATE_IDLE;
  activateSpeed(MOTOR_STOP);
  Serial.println("Game over");
}

void loop() {
  checkGlobalEvents();
  if (!gameActive) {
    endGame();
  }
  switch (state) {
    case STATE_IDLE:
      Serial.println("State: Idle");
      handleIdle();
      break;
    case STATE_LOCALIZE:
      Serial.println("State: Localize");
      handleLocalize();
      break;
    case STATE_ALIGN:
      Serial.println("State: Align");
      handleAlign();
      break;
    case STATE_FORWARD:
      Serial.println("State: Forward");
      handleMoveForward();
      break;
    case STATE_OUT_OB_ROT:
      Serial.println("State: Out of bounds, turning");
      handleOutOfBoundsRotate();
      break;
    case STATE_OUT_OB_BACK:
      Serial.println("State: Out of bounds, backing up");
      handleOutOfBoundsBackup();
      break;
    case STATE_OUT_OB_FORWARD:
      Serial.println("State: Out of bounds, going forward");
      handleOutOfBoundsForward();
      break;
    case STATE_PUSHING:
      Serial.println("State: Pushing");
      handlePushing();
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

uint8_t testGameTimerExpired(void) {
  return (uint8_t) gameTimer.check();
}

uint8_t testAlignTimerExpired(void) {
  return (uint8_t) alignTimer.check();
}

uint8_t testOutOfBoundsRotateTimerExpired(void) {
  return (uint8_t) outOfBoundsRotateTimer.check();
}

uint8_t testOutOfBoundsBackupForwardTimerExpired(void) {
  return (uint8_t) outOfBoundsBackwardForwardTimer.check();
}

void checkGlobalEvents() {
  limit1Input = digitalRead(limit_1);
  limit2Input = digitalRead(limit_2);
  if (testGameTimerExpired()) gameFinished();
}

void gameFinished(){
  gameActive = false;
}

void handleIdle(){
  if (gameActive) {
    gameTimer.reset();
    state = STATE_LOCALIZE;
  }
};

void setDirectionLeft(){
  digitalWrite(rightMotors_IN1_IN3, HIGH);
  digitalWrite(rightMotors_IN2_IN4, LOW);

  digitalWrite(leftMotors_IN1_IN3, LOW);
  digitalWrite(leftMotors_IN2_IN4, HIGH);
}

void setDirectionRight(){
  digitalWrite(rightMotors_IN1_IN3, LOW);
  digitalWrite(rightMotors_IN2_IN4, HIGH);

  digitalWrite(leftMotors_IN1_IN3, HIGH);
  digitalWrite(leftMotors_IN2_IN4, LOW);
}

void activateSpeed(int value){
  analogWrite(rightMotorFront_EnA, value);
  analogWrite(rightMotorBack_EnB, value);

  analogWrite(leftMotorFront_EnA, value);
  analogWrite(leftMotorBack_EnB, value);
}

void alignmentFinished(){
  Serial.println("Alignment finished");
  state = STATE_FORWARD;
}

void handleLocalize(){
  setDirectionRight();
  activateSpeed(MOTOR_SLOW_SPEED);
  Serial.println(abs(IR_BEACON_FREQUENCY - measuredIRFrequency));
  if (abs(IR_BEACON_FREQUENCY - measuredIRFrequency) < irFrequencyPrecision){
    state = STATE_ALIGN;
    // Send beacon signal via comm
    digitalWrite(COMM_OUT, HIGH);
    alignTimer.reset();
  }
  if (digitalRead(COMM_IN) == HIGH){
    state = STATE_ALIGN;
    rotationDirection = 'L';
  }
};

void handleAlign(){
  if (testAlignTimerExpired()) {
    Serial.println("Align timer expired");
    alignmentFinished();
  }

  if (rotationDirection == 'R') {
    setDirectionRight();
  }
  else if (rotationDirection == 'L') {
    setDirectionLeft();
  }

  activateSpeed(MOTOR_HALF_SPEED);
};

void setDirectionForward() {
  digitalWrite(rightMotors_IN1_IN3, LOW);
  digitalWrite(rightMotors_IN2_IN4, HIGH);

  digitalWrite(leftMotors_IN1_IN3, LOW);
  digitalWrite(leftMotors_IN2_IN4, HIGH);
}

void setDirectionBackward() {
  digitalWrite(rightMotors_IN1_IN3, HIGH);
  digitalWrite(rightMotors_IN2_IN4, LOW);

  digitalWrite(leftMotors_IN1_IN3, HIGH);
  digitalWrite(leftMotors_IN2_IN4, LOW);
}

void outOfBoundsFinished() {
  state = STATE_FORWARD;
}

void avoidLines() {
  uint16_t tapeSensorMeasurementFrontLeft = analogRead(tapeIR_FRONT_LEFT);
  uint16_t tapeSensorMeasurementFrontRight = analogRead(tapeIR_FRONT_RIGHT);
  uint16_t tapeSensorMeasurementBackLeft = analogRead(tapeIR_BACK_LEFT);
  uint16_t tapeSensorMeasurementBackRight = analogRead(tapeIR_BACK_RIGHT);
  Serial.println(LINE_THRESHOLD);
  if (tapeSensorMeasurementFrontLeft < LINE_THRESHOLD || tapeSensorMeasurementFrontRight < LINE_THRESHOLD) {
    state = STATE_OUT_OB_BACK;
    outOfBoundsBackwardForwardTimer.reset();
  }
  else if (tapeSensorMeasurementBackLeft < LINE_THRESHOLD || tapeSensorMeasurementBackRight < LINE_THRESHOLD) {
    state = STATE_OUT_OB_FORWARD;
    outOfBoundsBackwardForwardTimer.reset();
  }
}

void stopMotors(){
  digitalWrite(rightMotors_IN1_IN3, HIGH);
  digitalWrite(rightMotors_IN2_IN4, HIGH);

  digitalWrite(leftMotors_IN1_IN3, HIGH);
  digitalWrite(leftMotors_IN2_IN4, HIGH);
}

void resumeForward(){
  state = STATE_FORWARD;
}

void handleMoveForward(){
  setDirectionForward();
  activateSpeed(MOTOR_HALF_SPEED);
  if (limit1Input == 1 || limit2Input == 1){
    state = STATE_PUSHING;
  }
  avoidLines();
};

void handleOutOfBoundsRotate(){
  if (rotationDirection == 'L'){
    setDirectionLeft();
  }
  else if (rotationDirection == 'R'){
    setDirectionRight();
  }
  activateSpeed(MOTOR_HALF_SPEED);
  if (testOutOfBoundsRotateTimerExpired()) outOfBoundsFinished();
};

void handleOutOfBoundsBackup(){
  setDirectionBackward();
  activateSpeed(MOTOR_HALF_SPEED);
  if (testOutOfBoundsBackupForwardTimerExpired()){
    state = STATE_OUT_OB_ROT;
    outOfBoundsRotateTimer.reset();
  }
};

void handleOutOfBoundsForward(){
  setDirectionForward();
  activateSpeed(MOTOR_HALF_SPEED);
  if (testOutOfBoundsBackupForwardTimerExpired()){
    state = STATE_OUT_OB_ROT;
    outOfBoundsRotateTimer.reset();
  }
};

void handlePushing(){
  setDirectionForward();
  activateSpeed(MOTOR_FULL_SPEED);
  avoidLines();
};