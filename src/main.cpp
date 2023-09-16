#include <Arduino.h>
#include <Servo.h> // Disabled definition of timer 3 & 4
#include <SerialCommand.h> // change to serial port 3
#include <PID_v1.h>

//#define DEBUG
//#define DEBUG_SERIAL

const uint8_t POT = 58;
const uint8_t FEED_MOTOR = 9;
const uint8_t MOTOR1 = 11;
const uint8_t MOTOR2 = 21;

const uint8_t RX_PIN = 63;
const uint8_t TX_PIN = 40;
const uint8_t ENABLE_PIN = 38;
const uint8_t STEP_PIN = 54;
const uint8_t DIR_PIN = 55;
volatile uint32_t stepSpeed = 50;
volatile uint32_t curStepCount = 0;
volatile uint32_t targetStepCount = 0;
 /*
  System clock 16 Mhz and Prescalar 8
  Timer 3 speed = 16Mhz/8 = 2 MHz    
  Pulse time = 1 / 2 MHz  =  500ns
  Count up to = 5us / 500ns = 10 count
*/  
volatile uint16_t timer3Reload = 10;
 /*
  System clock 16 Mhz and Prescalar 1024
  Timer speed = 16Mhz/1024 = 15625 Hz    
  Pulse time = 1 / 15625 Hz = 64us 
  Count up to = 0.5s / 64us = 7813 count
*/  
volatile uint32_t numSec;
volatile uint16_t timer4Reload = 7813;
volatile uint8_t motorTurning = false;
uint8_t motorIndex = 0;

struct Motor
{
  const static uint16_t MIN_SPEED = 1000;
  const static uint16_t MAX_SPEED =  2000;
  const static uint8_t TELEMETRY_FRAME_SIZE = 10;
  //Kp = gain, increases speed to set point
  //Ki = repeats/time, determines speed of error of removal
  //Kd = derivative, not using
  constexpr static double Kp = .25, Ki = 0.2, Kd = 0; // tune depending on wheel mass
  
  HardwareSerial* hardSerial;
  Servo* esc = new Servo();
  uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = {0,};
  uint16_t motorSpeed;
  uint8_t bufferPosition = 0;
  bool motorInstruction = false;
  double inRPM, outRPM, targetRPM;
  PID* motorPID = new PID(&inRPM, &outRPM, &targetRPM, Kp, Ki, Kd, DIRECT);
  Motor(){}
  Motor(const uint8_t motorPin, HardwareSerial* serial) : hardSerial(serial){
    hardSerial->begin(115200); //esc uses 115200 as default baudrate
    motorPID->SetOutputLimits(1200, 6500);
    motorPID->SetSampleTime(200);
    motorPID->SetMode(AUTOMATIC);
    esc->attach(motorPin, MIN_SPEED, MAX_SPEED);
    esc->writeMicroseconds(MIN_SPEED); //need to fix jittering motor when supposed to be off
  }
};

Motor* m1; //D11, D17
Motor* m2; //D21, D15
Motor* motorArr[2];

SerialCommand cmd;
void unknownCommand(void);
void setMotorRPM(void);
void getMotorRPM(void);
void rotateMotor(void);
void moveStepper(void);

void setup() {
  Serial.begin(57600);
  Serial1.begin(9600); //blutooth D18, D19
  pinMode(MOTOR1, OUTPUT);
  digitalWrite(FEED_MOTOR, LOW);
  pinMode(FEED_MOTOR, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); //Enable is active low
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  //Home stepper motors. We can use sensorless cal with T2209

  cmd.addCommand("m", setMotorRPM);
  cmd.addCommand("ms", getMotorRPM);
  cmd.addCommand("s", moveStepper);
  cmd.addCommand("rm", rotateMotor);
  cmd.addDefaultHandler(unknownCommand);

  //Timer 3 configuration
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1<<CS31);
  TIMSK3 &= ~(1<<OCIE3A);
  OCR3A = timer3Reload;
  interrupts();
  //End Timer 3 

  //Timer 4 configuration
  noInterrupts();
  OCR4A = timer4Reload;
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= (1<<WGM12) | (1<<CS42);
  TIMSK4 &= ~(1<<OCIE4A);
  interrupts();
  //End Timer 4

  m1 = new Motor(MOTOR1, &Serial2); //D11, D17
  m2 = new Motor(MOTOR2, &Serial3); //D21, D15
  motorArr[0] = m1;
  motorArr[1] = m2;
}

void loop()
{

  cmd.readSerial();

  //while(Serial1.available())
  //  Serial.print((char)Serial1.read());
  
  while(motorArr[motorIndex]->hardSerial->available()){
      motorArr[motorIndex]->telemetryBuffer[motorArr[motorIndex]->bufferPosition++] = motorArr[motorIndex]->hardSerial->read();
      if(motorArr[motorIndex]->bufferPosition == Motor::TELEMETRY_FRAME_SIZE){
        // feature: add crc data validation
        motorArr[motorIndex]->bufferPosition = 0;
        float eRevPerMin = (motorArr[motorIndex]->telemetryBuffer[7] << 8) | motorArr[motorIndex]->telemetryBuffer[8];
        //To get the real Rpm of the motor, divide the Erpm by the magnetpole count divided by two.
        motorArr[motorIndex]->inRPM = (eRevPerMin*100) / 11.0;
        #ifdef DEBUG
          // - Temperature (resolution 1°C)
          // - Voltage (resolution 0.01V)
          // - Current (resolution 0.01A)
          // - Consumption (resolution 1mAh)
          // - Electrical Rpm (resolution 100Rpm)
          //float motorCurrent = 0;
          //float motorVoltage = 0;
          //motorVoltage = (telemetryBuffer[2] << 8) | telemetryBuffer[1];
          //motorCurrent = (telemetryBuffer[3] << 8) | telemetryBuffer[4];
          Serial1.print("RPM:");
          Serial1.println(motorArr[motorIndex].inRPM);
        #endif
      }
  }
  
  if(motorArr[motorIndex]->motorInstruction){
      motorArr[motorIndex]->motorPID->Compute();

    if(motorArr[motorIndex]->targetRPM == 0 and motorArr[motorIndex]->inRPM < 300){ //spin down motor
      motorArr[motorIndex]->esc->writeMicroseconds(Motor::MIN_SPEED);
      motorArr[motorIndex]->motorInstruction = false;
      Serial.print("Set point reached");
    }
    else if(motorArr[motorIndex]->inRPM != motorArr[motorIndex]->targetRPM){ 
      motorArr[motorIndex]->motorSpeed = map(motorArr[motorIndex]->outRPM, 1000, 6500, Motor::MIN_SPEED, Motor::MAX_SPEED); // Map current PID value to motor speed
      motorArr[motorIndex]->esc->writeMicroseconds(motorArr[motorIndex]->motorSpeed);
    }
    else { // inRPM == targetRPM
      motorArr[motorIndex]->motorInstruction = false;
      Serial.print("Set point reached");
    } 
  }

  motorIndex = !motorIndex;
}

ISR(TIMER3_COMPA_vect)
{
  TCNT3 = 0;  
  OCR3A = timer3Reload * stepSpeed;

  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);

  if(curStepCount == targetStepCount){
    digitalWrite(ENABLE_PIN, HIGH);
    targetStepCount = 0;
    curStepCount = 0;
    noInterrupts();           
    TIMSK3 &= ~(1<<OCIE3A); //Disable timer 3 ISR
    interrupts();
  }
  else curStepCount++;
} //Timer 3 counter

ISR(TIMER4_COMPA_vect)
{
  TCNT4 = 0;  
  motorTurning = !motorTurning;

  if(motorTurning){
    OCR4A = timer3Reload * numSec;
    analogWrite(FEED_MOTOR, 128); //Lower speed if time to rotate is < 1 sec
  }
  else{
    analogWrite(FEED_MOTOR, 0);
    noInterrupts();           
    TIMSK4 &= ~(1<<OCIE4A); //Disable timer 4 ISR
    interrupts();
  }
  
} //Timer 4 counter

void moveStepper()
{
  char* arg = cmd.next();
  char* garbage = NULL;
  if(arg != NULL){
    int32_t stepNumber = strtol(arg, &garbage,0);
    stepNumber > 0 ? digitalWrite(DIR_PIN, HIGH) : digitalWrite(DIR_PIN, LOW);
    targetStepCount += stepNumber; //accum steps if more are added while in process
    digitalWrite(ENABLE_PIN, LOW);
  }

  #ifdef DEBUG_SERIAL
    Serial.print("Stepper count: ");
    Serial.print(arg);
    Serial.print(", ");
    Serial.println(targetStepCount);
  #endif

  noInterrupts();
  TIMSK3 |= (1<<OCIE3A); //Enable timer 3 ISR
  interrupts();
}

void setMotorRPM()
{
  char* arg = cmd.next();
  char* garbage = NULL;
  
  if(arg != NULL){
    uint32_t motSpeed = (uint32_t) strtol(arg, &garbage,0);
    if((motSpeed >= 1500 && motSpeed <= 6500) || motSpeed == 0){
      arg = cmd.next();
      if(arg != NULL){  
        uint8_t motNum = (uint8_t) strtol(arg, &garbage,0);
        motorArr[motNum]->targetRPM = motSpeed;
        Serial.println(motorArr[motNum]->targetRPM);
        motorArr[motNum]->motorInstruction = true;
      }
    }
  }
}

void getMotorRPM()
{
  char* arg = cmd.next();
  char* garbage = NULL;
  if(arg != NULL){
    uint8_t motNum = (uint8_t) strtol(arg, &garbage,0);
    if(motNum == 0 || motNum == 1)
      Serial.println(motorArr[motNum]->inRPM);
  }
}

void rotateMotor(){
  char* arg = cmd.next();
  char* garbage = NULL;
  if(arg != NULL){
    uint32_t tempVal = ((uint8_t)strtol(arg, &garbage,0));
    if(tempVal > 0 && tempVal < 10){
      numSec = timer4Reload * tempVal * 2;
      noInterrupts();
      TIMSK4 |= (1<<OCIE4A); //Enable timer 4 ISR
      interrupts();
    }
  }
}
void unknownCommand()
{
  Serial.println("Command not found");
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
