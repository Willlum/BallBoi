#include <Arduino.h>
#include <Servo.h> // Disabled definition of timer 3
#include <SerialCommand.h> // change to serial port 3
#include <PID_v1.h>
//#include <TMC2209.h> // Uses softserial which restricts cpu time. Should only use softserial in setup

//#define DEBUG
#define DEBUG_SERIAL

const uint16_t MIN_SPEED = 1100;
const uint16_t MAX_SPEED =  2000;
const uint8_t FAN = 9;
const uint8_t MOTOR = 11;
const uint8_t POT = 58;
const uint8_t RX_PIN = 63;
const uint8_t TX_PIN = 40;
const uint8_t ENABLE_PIN = 38;
const uint8_t STEP_PIN = 54;
const uint8_t DIR_PIN = 55;
const uint8_t TELEMETRY_FRAME_SIZE = 10;
static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };
uint8_t bufferPosition = 0;
uint32_t motorRPM;
//SoftwareSerial soft_serial(RX_PIN, TX_PIN);
//TMC2209 stepper_driver;
Servo esc;
const int32_t MAX_STEP_VELOCITY = 210000;
const int32_t STOP_VELOCITY = 0;
const int RUN_DURATION = 2000;
const int STOP_DURATION = 1000;
const int RUN_COUNT = 4;

uint16_t motorspeed;
double inRevPerMin;
double outRevPerMin;
double targetRevPerMin;
double Kp=2, Ki=5, Kd=1;
bool motorArmed = false;
PID motorPID(&inRevPerMin, &outRevPerMin, &targetRevPerMin, Kp, Ki, Kd, DIRECT);

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
void setMotorRPM(void);
void moveStepper(void);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 6500);

  //esc.attach(MOTOR, MIN_SPEED, MAX_SPEED);
  pinMode(MOTOR, OUTPUT);
  digitalWrite(FAN, HIGH);
  pinMode(FAN, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  cmd.addCommand("M", setMotorRPM);
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
  uint16_t val = analogRead(POT);
  analogWrite(FAN, map(val, 0, 1023, 0, 255)); // timer 2 is used for pwm
  
  while(Serial2.available()){
    telemetryBuffer[bufferPosition++] = Serial2.read();

    if(bufferPosition == TELEMETRY_FRAME_SIZE){
      bufferPosition = 0;
      float eRevPerMin = (telemetryBuffer[7] << 8) | telemetryBuffer[8];
      //Note: To get the real Rpm of the motor, divide the Erpm by the magnetpole count divided by two.
      inRevPerMin = (eRevPerMin*100) / 11.0;
      
      // feature: add crc data validation
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
        // Serial.print("Voltage(V):");
        // Serial.println(motorVoltage,3);
        // Serial.print("Current(A):");
        // Serial.println(motorCurrent,3);
        Serial.print("RPM:");
        Serial.println(curRevPerMin);
      #endif
    }
  }

  if(motorArmed){
    if(targetRevPerMin == inRevPerMin) motorArmed = false;
    motorPID.Compute();
    motorspeed = map(outRevPerMin, 0, 6500, MIN_SPEED, MAX_SPEED);
    esc.writeMicroseconds(motorspeed);
  }
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

void setMotorRPM(){
  char* arg = cmd.next();
  char* garbage = NULL;
  
  if(arg != NULL){
    targetRevPerMin = (uint32_t) strtol(arg, &garbage,0);
  }
  motorArmed = true;
}

void unknownCommand(){
  Serial.println("Command not found");
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
