#include <Arduino.h>

#define LINE_THRESHOLD          700   
#define MOTOR_TIME_INTERVAL     2000
#define MOTOR_HALF_SPEED        50
#define GAME_TIMER            130

void checkGlobalEvents(void);
void handleIdle(void);
void handleLocalize(void);
void handleAlign(void);
void handleMoveForward(void);
void handleOutOfBounds(void);
void handlePushing(void);
void driveMotor(int dutyCycle);
//unsigned char TestForBounds(void);
//void RespToBounds(void);
unsigned char TestMotorTimerExpired(void);
void RespMotorTimerExpired(void);


typedef enum {
  STATE_IDLE,
  STATE_LOCALIZE,
  STATE_ALIGN,
  STATE_MOVE_FORWARD, 
  STATE_OUT_OB,
  STATE_PUSHING
} States_t;


States_t state;
int robotID = 1;
int limitPin = 8;
int tapePin = 20;
int irPin = 14;
int motorPin = 10; //TODO change

IntervalTimer gameTimer;
IntervalTimer motorTimer;


void setup() {
  // TODO start timer
  state = STATE_LOCALIZE;
  pinMode(limitPin, INPUT);
  pinMode(tapePin, INPUT);
  pinMode(irPin, INPUT);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  checkGlobalEvents();
  switch (state) {
    case STATE_IDLE:
      Serial.println("Idle");
      handleIdle();
    case STATE_LOCALIZE:
      Serial.println("Localize");
      handleLocalize();
      break;
    case STATE_ALIGN:
      Serial.println("Align");
      handleAlign();
      break;
    case STATE_MOVE_FORWARD:
      Serial.println("Forward");
      handleMoveForward();
      break;
    case STATE_OUT_OB:
      Serial.println("Out of bounds");
      handleOutOfBounds();
      break;
    case STATE_PUSHING:
      Serial.println("Pushing");
      handlePushing();
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

void checkGlobalEvents() {
  int limit_in = digitalRead(limitPin);
  int tape_in = analogRead(tapePin);
  int ir_in = digitalRead(irPin);
  Serial.print("Ir in: ");
  Serial.print(ir_in);
  Serial.print("      Tape pin: ");
  Serial.println(tape_in);
  delay(100);
}

void handleIdle(){};
void handleLocalize(){};
void handleAlign(){};
void handleMoveForward(){};
void handleOutOfBounds(){};
void handlePushing(){};
void driveMotor(int dutyCycle) {};
