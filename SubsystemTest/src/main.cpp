#include <Arduino.h>


/*-----------------------------Module Defines-----------------------------*/
#define MOTOR_FULL_SPEED        255
#define MOTOR_HALF_SPEED        124
#define MOTOR_SLOW_SPEED        50
#define MOTOR_STOP              0

/*-----------------------Module Function Prototypes-----------------------*/
uint8_t testForKey(void);
void RespToKey(void);
void CountFallingEdges();
void inputFreq();

/*---------------------------State Definitions----------------------------*/


/*----------------------------Module Variables----------------------------*/
const uint8_t TapeIR_1 = 23;              // Tape Sensor
const uint8_t TapeIR_2 = 22;              // Tape Sensor
const uint8_t TapeIR_3 = 21;              // Tape Sensor
const uint8_t TapeIR_4 = 20;              // Tape Sensor

const uint8_t SenseIR_1 = 14;             // IR Beacon Sensor

const uint8_t Right_Motors_IN1_IN3 = 0;   // Logic 1
const uint8_t Right_Motors_IN2_IN4 = 1;   // Logic 2
const uint8_t Right_Motor_Front_EnA = 3;  // PWM Speed Control
const uint8_t Right_Motor_Back_EnB = 4;   // PWM Speed Control

const uint8_t Left_Motors_IN1_IN3 = 5;    // Logic 1
const uint8_t Left_Motors_IN2_IN4 = 6;    // Logic 2
const uint8_t Left_Motor_Front_EnA = 9;   // PWM Speed Control
const uint8_t Left_Motor_Back_EnB = 10;   // PWM Speed Control

const uint8_t Limit_1 = 7;                // Limit Switch

int key;                                  // Motor Test Input

// Edge Counting Variables
IntervalTimer InputFreqTimer;
volatile uint16_t EdgeCount = 0;            // use volatile for shared variables
volatile float Frequency_Measure = 10000;   // Calculate every 10,000 micro-s

/*-----------------------------Main Functions-----------------------------*/

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(TapeIR_1, INPUT);                 // Set Pin 23 as tape sensor input
  pinMode(TapeIR_2, INPUT);                 // Set Pin 23 as tape sensor input
  pinMode(TapeIR_3, INPUT);                 // Set Pin 23 as tape sensor input
  pinMode(TapeIR_4, INPUT);                 // Set Pin 23 as tape sensor input
  pinMode(SenseIR_1, INPUT);                // Set Pin 14 as IR beacon sensor

  pinMode(Limit_1, INPUT);                  // Set Pin 7 as Limit Switch 1

  pinMode(Right_Motors_IN1_IN3, OUTPUT);    // Set Pin 0 as Right Motor Logic Input 1
  pinMode(Right_Motors_IN2_IN4, OUTPUT);    // Set Pin 1 as Right Motor Logic Input 2
  pinMode(Right_Motor_Front_EnA, OUTPUT);   // Set Pin 3 as Right Front Motor PWM Speed Control
  pinMode(Right_Motor_Back_EnB, OUTPUT);    // Set Pin 4 as Right Back Motor PWM Speed Control

  pinMode(Left_Motors_IN1_IN3, OUTPUT);     // Set Pin 5 as Left Motor Logic Input 1
  pinMode(Left_Motors_IN2_IN4, OUTPUT);     // Set Pin 6 as Left Motor Logic Input 2
  pinMode(Left_Motor_Front_EnA, OUTPUT);    // Set Pin 9 as Left Front Motor PWM Speed Control
  pinMode(Left_Motor_Back_EnB, OUTPUT);     // Set Pin 10 as Left Back Motor PWM Speed Control

  //Initial Idle Motor Setup
  digitalWrite(Right_Motors_IN1_IN3, LOW);  //
  digitalWrite(Right_Motors_IN2_IN4, LOW);  // This sets right motors to break (LogicInput1 = LogicInput2)
  digitalWrite(Left_Motors_IN2_IN4,  LOW);  //
  digitalWrite(Left_Motors_IN1_IN3,  LOW);  // This sets left motors to break (LogicInput1 = LogicInput2)

  //Edge counting timer
  attachInterrupt(digitalPinToInterrupt(SenseIR_1), CountFallingEdges, FALLING);
  InputFreqTimer.begin(inputFreq, Frequency_Measure);
  
  while(!Serial);
}
  
void loop() 
{
  // put your main code here, to run repeatedly:
  uint16_t photo_tr_1 = analogRead(TapeIR_1);
  uint16_t photo_tr_2 = analogRead(TapeIR_2);
  uint16_t photo_tr_3 = analogRead(TapeIR_3);
  uint16_t photo_tr_4 = analogRead(TapeIR_4);
  uint16_t photo_ir = digitalRead(SenseIR_1);

  uint8_t limit_in1 = digitalRead(Limit_1);
  
  //Serial.print("Limit Switch: ");
  //Serial.print(limit_in1);
  Serial.print("    Tape Sensor 1: ");
  Serial.println(photo_tr_1);
  Serial.print("    Tape Sensor 2: ");
  Serial.println(photo_tr_2);
  Serial.print("    Tape Sensor 3: ");
  Serial.println(photo_tr_3);
  Serial.print("    Tape Sensor 4: ");
  Serial.println(photo_tr_4);
  /*Serial.print("    IR Sensor: ");
  Serial.print(photo_ir);
  Serial.print("    Frequency: ");
  Serial.println(EdgeCount*100);*/

  delay(1000); 

  if (testForKey()) RespToKey();

}

/*----------------------------Module Functions----------------------------*/

//----- Edge counting functions
void CountFallingEdges()
{
  EdgeCount++;
}
void inputFreq()
{
  // Serial.println(EdgeCount*100);
  EdgeCount = 0;
}
//-----

uint8_t testForKey(void) {
  uint8_t keyEventOccurred = Serial.available();
  if (keyEventOccurred > 0)
  {
    key = Serial.read();
  }
  Serial.read();
  return keyEventOccurred;
}

void RespToKey(void) {
  if (key == 72 || key == 104)              // Letter H
  {
    Serial.println("Motors Full Throttle");

    analogWrite(Right_Motor_Front_EnA, MOTOR_FULL_SPEED);
    analogWrite(Right_Motor_Back_EnB, MOTOR_FULL_SPEED);

    analogWrite(Left_Motor_Front_EnA, MOTOR_FULL_SPEED);
    analogWrite(Left_Motor_Back_EnB, MOTOR_FULL_SPEED);
  }
  else if (key == 77 || key == 109)         // Letter M
  {
    Serial.println("Motors Medium Speed");

    analogWrite(Right_Motor_Front_EnA, MOTOR_HALF_SPEED);
    analogWrite(Right_Motor_Back_EnB, MOTOR_HALF_SPEED);

    analogWrite(Left_Motor_Front_EnA, MOTOR_HALF_SPEED);
    analogWrite(Left_Motor_Back_EnB, MOTOR_HALF_SPEED);
  }
  else if (key == 'p' || key == 'P')         // Letter P
  {
    Serial.println("Motors Low Speed");

    analogWrite(Right_Motor_Front_EnA, MOTOR_SLOW_SPEED);
    analogWrite(Right_Motor_Back_EnB, MOTOR_SLOW_SPEED);

    analogWrite(Left_Motor_Front_EnA, MOTOR_SLOW_SPEED);
    analogWrite(Left_Motor_Back_EnB, MOTOR_SLOW_SPEED);
  }
  else if (key == 83 || key == 115)         // Letter S
  {
    Serial.println("Motors Stop"); 

    analogWrite(Right_Motor_Front_EnA, MOTOR_STOP);
    analogWrite(Right_Motor_Back_EnB, MOTOR_STOP);

    analogWrite(Left_Motor_Front_EnA, MOTOR_STOP);
    analogWrite(Left_Motor_Back_EnB, MOTOR_STOP);
  }
  else if (key == 70 || key == 102)         // Letter F
  {
    Serial.println("Motors Forward");

    digitalWrite(Right_Motors_IN1_IN3, LOW);
    digitalWrite(Right_Motors_IN2_IN4, HIGH);

    digitalWrite(Left_Motors_IN1_IN3, LOW);
    digitalWrite(Left_Motors_IN2_IN4, HIGH);
  }
  else if (key == 66 || key == 98)           // Letter B
  {
    Serial.println("Motors Backwards");

    digitalWrite(Right_Motors_IN1_IN3, HIGH);
    digitalWrite(Right_Motors_IN2_IN4, LOW);

    digitalWrite(Left_Motors_IN1_IN3, HIGH);
    digitalWrite(Left_Motors_IN2_IN4, LOW);    
  }
  else if (key == 82 || key == 114)         // Letter R
  {
    Serial.println("Turn Right");

    digitalWrite(Right_Motors_IN1_IN3, LOW);
    digitalWrite(Right_Motors_IN2_IN4, HIGH);

    digitalWrite(Left_Motors_IN1_IN3, HIGH);
    digitalWrite(Left_Motors_IN2_IN4, LOW); 
  }
  else if (key == 76 || key == 108)        // Letter L
  {
    Serial.println("Turn Left");

    digitalWrite(Right_Motors_IN1_IN3, HIGH);
    digitalWrite(Right_Motors_IN2_IN4, LOW);

    digitalWrite(Left_Motors_IN1_IN3, LOW);
    digitalWrite(Left_Motors_IN2_IN4, HIGH); 
  }
}