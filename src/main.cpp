#include <Arduino.h>
#include <Metro.h>

/*-----------------------------Module Defines-----------------------------*/
#define IR_BEACON_FREQUENCY     1000
#define GAME_TIMER_INTERVAL     128000  // Need to correct
#define ALIGNMENT_INTERVAL      5000   // Need to calibrate
#define FAKE_INTERVAL           150000 // Longer than game timer; for Metro calibration
#define OB_BACKUP_INTERVAL      5000   // Need to calibrate
#define OB_ROTATE_INTERVAL      5000   // Need to calibrate
#define MOTOR_FULL_SPEED        255
#define MOTOR_HALF_SPEED        124
#define MOTOR_SLOW_SPEED        80   // Need to calibrate
#define MOTOR_STOP              0
#define LIMIT_THRESHOLD         10   // Need to calibrate
#define LINE_THRESHOLD          700  // Need to calibrate
#define ALIGN_ROTATE_DIRECTION  "right"

/*-----------------------Module Function Prototypes-----------------------*/
void checkGlobalEvents(void);
void handleIdle(void);
void handleLocalize(void);
void handleAlign(void);
void handleMoveForward(void);
void handleOutOfBounds(void);
void handlePushing(void);
void activateSpeed(int value);
void handleOutOfBoundsLeft(void);
void handleOutOfBoundsRight(void);
void gameFinished(void);
void alignmentFinished(void);
void outOfBoundsFinished(void);

/*---------------------------State Definitions----------------------------*/
typedef enum {
  STATE_IDLE,
  STATE_LOCALIZE,
  STATE_ALIGN,
  STATE_MOVE_FORWARD, 
  STATE_OUT_OB_L,
  STATE_OUT_OB_R,
  STATE_PUSHING
} States_t;

/*----------------------------Module Variables----------------------------*/
States_t state;
static Metro gameTimer = Metro(GAME_TIMER_INTERVAL); // TODO change to millis()
static Metro alignTimer = Metro(FAKE_INTERVAL);
static Metro outOfBoundsTimer = Metro(FAKE_INTERVAL);
IntervalTimer inputFrequencyTimer;

bool gameActive = true;

const uint8_t tapeIR_FRONT_LEFT = 22;          // Front Left Tape Sensor
const uint8_t tapeIR_FRONT_RIGHT = 23;         // Front Right Tape Sensor
const uint8_t tapeIR_BACK_LEFT = 27;           // Back Left Tape Sensor
const uint8_t tapeIR_BACK_RIGHT = 28;          // Back Right Tape Sensor

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

int key;                                       // Motor Test Input
int limit1Input;                               // First limit switch input reading
int limit2Input;                               // Second limit switch input reading
int tapeInput1;                                // Left tape sensor input reading
int tapeInput2;                                // Right tape sensor input reading
int measuredIRFrequency = 0;                   // IR sensor input reading

volatile uint16_t edgeCount = 0;               // Track number of falling waves in waveform
volatile float irMeasurementFrequency = 10000; // Calculate every 10,000 micro-seconds
float irFrequencyPrecision = 20;              // Need to calibrate

/* TODO:
  Test out of bounds code
  Replace limit switch threshold with digital input
  Test for out of bounds in global events check
  Test if stops after 2:10
*/

/*-----------------------------Main Functions-----------------------------*/

void countFallingEdges() {
  edgeCount++;
}

void recordMeasuredIRFrequency() {
  //noInterrupts();
  measuredIRFrequency = edgeCount * 100;
  edgeCount = 0;
  //interrupts();
}

void setup() {
  state = STATE_LOCALIZE;
  pinMode(tapeIR_FRONT_LEFT, INPUT);                     // Set Pin 23 as tape sensor input
  
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
  delay(1000);
  Serial.print("Frequency: ");
  Serial.println(measuredIRFrequency);
  checkGlobalEvents();
  if (!gameActive) {
    endGame();
  }
  switch (state) {
    case STATE_IDLE:
      Serial.println("State: Idle");
      handleIdle();
    case STATE_LOCALIZE:
      Serial.println("State: Localize");
      handleLocalize();
      break;
    case STATE_ALIGN:
      Serial.println("State: Align");
      handleAlign();
      break;
    case STATE_MOVE_FORWARD:
      Serial.println("State: Forward");
      handleMoveForward();
      break;
    case STATE_OUT_OB_L:
      Serial.println("State: Out of bounds (left)");
      handleOutOfBoundsLeft();
      break;
    case STATE_OUT_OB_R:
      Serial.println("State: Out of bounds (right)");
      handleOutOfBoundsRight();
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

uint8_t testOutOfBoundsTimerExpired(void) {
  return (uint8_t) outOfBoundsTimer.check();
}

void checkGlobalEvents() {
  limit1Input = digitalRead(limit_1);
  limit2Input = digitalRead(limit_2);
  tapeInput1 = analogRead(tapeIR_FRONT_LEFT);
  tapeInput2 = analogRead(tapeIR_FRONT_RIGHT);
  if (testGameTimerExpired()) gameFinished();
  if (testAlignTimerExpired()) {
    Serial.println("Align timer expired");
    alignmentFinished();}
  if (testOutOfBoundsTimerExpired()) outOfBoundsFinished();
  /*Serial.print("Ir input: ");
  Serial.print(irInput);
  Serial.print("      Tape pin: ");
  Serial.println(tape_in);
  delay(100);*/
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
  Serial.println("Turn Left");

  digitalWrite(rightMotors_IN1_IN3, HIGH);
  digitalWrite(rightMotors_IN2_IN4, LOW);

  digitalWrite(leftMotors_IN1_IN3, LOW);
  digitalWrite(leftMotors_IN2_IN4, HIGH);
}

void setDirectionRight(){
  Serial.println("Turn Right");

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
  state = STATE_MOVE_FORWARD;
}

void handleLocalize(){
  setDirectionRight();
  activateSpeed(MOTOR_SLOW_SPEED);
  if (abs(IR_BEACON_FREQUENCY - measuredIRFrequency) < irFrequencyPrecision){
    state = STATE_ALIGN;
    alignTimer.interval(ALIGNMENT_INTERVAL);
    //alignTimer.reset();
  }
};

void handleAlign(){
  if (strcmp(ALIGN_ROTATE_DIRECTION, "right") == 0) {
    setDirectionRight();
  }
  else if (strcmp(ALIGN_ROTATE_DIRECTION, "left") == 0) {
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
  state = STATE_MOVE_FORWARD;
}

void testLineThreshold() {
  return; // TODO remove when tape sensors are added
  if (tapeIR_FRONT_LEFT > LINE_THRESHOLD) {
    state = STATE_OUT_OB_L;
    outOfBoundsTimer.interval(OB_ROTATE_INTERVAL);
    outOfBoundsTimer.reset();
    //outOfBoundsTimer.reset();
  }
  else if (tapeIR_FRONT_RIGHT > LINE_THRESHOLD) {
    state = STATE_OUT_OB_R;
    outOfBoundsTimer.interval(OB_ROTATE_INTERVAL);
    outOfBoundsTimer.reset();
    //outOfBoundsTimer.reset();
  }
}

void handleMoveForward(){
  setDirectionForward();
  activateSpeed(MOTOR_HALF_SPEED);
  if (limit1Input > LIMIT_THRESHOLD || limit2Input > LIMIT_THRESHOLD){
    state = STATE_PUSHING;
  }
  testLineThreshold();
};

void handleOutOfBoundsLeft(){
  setDirectionRight();
  activateSpeed(MOTOR_HALF_SPEED);
};

void handleOutOfBoundsRight(){
  setDirectionLeft();
  activateSpeed(MOTOR_HALF_SPEED);
};

void handlePushing(){
  setDirectionForward();
  activateSpeed(MOTOR_FULL_SPEED);
  testLineThreshold();
};
