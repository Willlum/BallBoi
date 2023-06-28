#include <Arduino.h>
#include <Servo.h> // Disabled definition of timer 3
#include <SerialCommand.h> // change to serial port 3
#include <PID_v1.h>

//#define DEBUG
#define DEBUG_SERIAL

const uint8_t FAN = 9;
const uint8_t POT = 58;
const uint8_t ENABLE_PIN = 38;
const uint8_t STEP_PIN = 54;
const uint8_t DIR_PIN = 55;

Servo esc;
const int32_t MAX_STEP_VELOCITY = 210000;
const int32_t STOP_VELOCITY = 0;
const int RUN_DURATION = 2000;
const int STOP_DURATION = 1000;
const int RUN_COUNT = 4;

volatile uint32_t stepSpeed = 30;
volatile uint32_t curStepCount = 0;
volatile uint32_t targetStepCount = 0;
 /*
  System clock 16 Mhz and Prescalar 8
  Timer 1 speed = 16Mhz/8 = 2 MHz    
  Pulse time = 1/2 MHz  =  500ns
  Count up to = 5us / 500ns = 10
*/  
volatile uint32_t reload = 10;
SerialCommand cmd;

void unknownCommand(void);
void moveStepper(void);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  cmd.addCommand("S", moveStepper);
  cmd.addDefaultHandler(unknownCommand);

  //Timer 3 configuration
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1<<CS31);
  TIMSK3 |= (1<<OCIE3A);
  OCR3A = reload;
  interrupts();

  //Home stepper motors
}

void loop() {
  cmd.readSerial();
}

ISR(TIMER3_COMPA_vect){
  curStepCount++;
  TCNT3 = 0;  
  OCR3A = reload * stepSpeed;
  if(stepSpeed < 4990){
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
  }
  if(curStepCount > targetStepCount){
    digitalWrite(ENABLE_PIN, HIGH);
    curStepCount = 0;
    TIMSK3 |= (0<<OCIE3A);
  }
} //Timer 3 counter

void moveStepper(){
  char* arg = cmd.next();
  char* garbage = NULL;
  if(arg != NULL){
    int32_t stepNumber = strtol(arg, &garbage,0);
    stepNumber > 0 ? digitalWrite(DIR_PIN, HIGH) : digitalWrite(DIR_PIN, LOW);
    targetStepCount = abs(stepNumber);
    digitalWrite(ENABLE_PIN, LOW);
  }
  #ifdef DEBUG_SERIAL
      Serial.print("Stepper count: ");
      Serial.print(arg);
      Serial.print(", ");
      Serial.println(targetStepCount);
  #endif
  noInterrupts();
  TIMSK3 |= (1<<OCIE3A);
  interrupts();
}

void unknownCommand(){
  Serial.println("Command not found");
}

