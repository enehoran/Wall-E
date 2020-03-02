#include <Arduino.h>

/*-----------------------------Module Defines-----------------------------*/
#define MOTOR_TIMER_INTERVAL    2000 // Need to clarify var name
#define GAME_TIMER_INTERVAL     130  // Need to correct
#define ALIGNMENT_INTERVAL      15   // Need to calibrate
#define OB_BACKUP_INTERVAL      15   // Need to calibrate
#define OB_ROTATE_INTERVAL      15   // Need to calibrate
#define MOTOR_FULL_SPEED        255
#define MOTOR_HALF_SPEED        124
#define MOTOR_SLOW_SPEED        10
#define MOTOR_STOP              0
#define IR_THRESHOLD            10   // Need to calibrate
#define LIMIT_THRESHOLD         10   // Need to calibrate
#define LINE_THRESHOLD          700  // Need to calibrate

/*-----------------------Module Function Prototypes-----------------------*/
void checkGlobalEvents(void);
void handleIdle(void);
void handleLocalize(void);
void handleAlign(void);
void handleMoveForward(void);
void handleOutOfBounds(void);
void handlePushing(void);

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

IntervalTimer gameTimer;
IntervalTimer motorTimer;
IntervalTimer alignTimer;
IntervalTimer outOfBoundsTimer;

bool gameActive = true;

const uint8_t tapeIR_L = 22;              // Left Tape Sensor
const uint8_t tapeIR_R = 23;              // Right Tape Sensor

const uint8_t senseIR_1 = 14;             // IR Beacon Sensor

const uint8_t rightMotors_IN1_IN3 = 0;    // Logic 1
const uint8_t rightMotors_IN2_IN4 = 1;    // Logic 2
const uint8_t rightMotorFront_EnA = 3;    // PWM Speed Control
const uint8_t rightMotorBack_EnB = 4;     // PWM Speed Control

const uint8_t leftMotors_IN1_IN3 = 5;     // Logic 1
const uint8_t leftMotors_IN2_IN4 = 6;     // Logic 2
const uint8_t leftMotorFront_EnA = 9;     // PWM Speed Control
const uint8_t leftMotorBack_EnB = 10;     // PWM Speed Control

const uint8_t limit_1 = 7;                // Limit Switch 1
const uint8_t limit_2 = 8;                // Limit Switch 2

int key;                                  // Motor Test Input
int limit1Input;                          // First limit switch input reading
int limit2Input;                          // Second limit switch input reading
int tapeInput1;                           // Left tape sensor input reading
int tapeInput2;                           // Right tape sensor input reading
int irInput;                              // IR sensor input reading


/* TODO:
  Limit number of interrupts to two (use Metro library) b/c it's 
    all the Teensy can handle
  Use frequency output for IR sensor instead of analogRead
  Update alignment code
  Test out of bounds code
  Add two more tape sensors in the code (each bot has 4)
  Replace limit switch threshold with digital input
  Test for out of bounds in global events check
  Calibrate slow motor speed pwm setting
*/


/*-----------------------------Main Functions-----------------------------*/
void setup() {
  state = STATE_LOCALIZE;
  pinMode(tapeIR_L, INPUT);                     // Set Pin 23 as tape sensor input
  
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
}

void endGame() {
  state = STATE_IDLE;
  activateSpeed(MOTOR_STOP);
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

void checkGlobalEvents() {
  limit1Input = digitalRead(limit_1);
  limit2Input = digitalRead(limit_2);
  tapeInput1 = analogRead(tapeIR_L);
  tapeInput2 = analogRead(tapeIR_R);
  irInput = digitalRead(senseIR_1);
  /*Serial.print("Ir input: ");
  Serial.print(irInput);
  Serial.print("      Tape pin: ");
  Serial.println(tape_in);
  delay(100);*/
}

void gameTimerExpired(){
  gameActive = false;
}

void handleIdle(){
  if (gameActive) {
    gameTimer.begin(gameTimerExpired, GAME_TIMER_INTERVAL);
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
  state = STATE_MOVE_FORWARD;
}

void handleLocalize(){
  setDirectionRight();
  activateSpeed(MOTOR_HALF_SPEED);
  if (irInput > IR_THRESHOLD){
    state = STATE_ALIGN;
    alignTimer.begin(alignmentFinished, ALIGNMENT_INTERVAL);
  }
};

void handleAlign(){
  setDirectionRight();
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
  if (tapeIR_L > LINE_THRESHOLD) {
    state = STATE_OUT_OB_L;
    outOfBoundsTimer.begin(outOfBoundsFinished, OB_ROTATE_INTERVAL);
  }
  else if (tapeIR_R > LINE_THRESHOLD) {
    state = STATE_OUT_OB_R;
    outOfBoundsTimer.begin(outOfBoundsFinished, OB_ROTATE_INTERVAL);
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