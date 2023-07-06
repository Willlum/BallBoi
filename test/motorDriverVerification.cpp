#include <Arduino.h>
#include <Servo.h> // Disabled definition of timer 3 & 4
#include <SerialCommand.h> // change to serial port 3
#include <PID_v1.h>

//#define DEBUG
#define DEBUG_SERIAL

const uint16_t MIN_SPEED = 1060;
const uint16_t MAX_SPEED =  2000;
const uint8_t FAN = 9;
const uint8_t MOTOR = 11;
const uint8_t POT = 58;

Servo esc;
uint16_t motorSpeed = MIN_SPEED;
double inRPM;
double outRPM;
double targetRPM;
double Kp=1, Ki=1, Kd=1;
PID motorPID(&inRPM, &outRPM, &targetRPM, Kp, Ki, Kd, DIRECT);
bool motorArmed = false;
const uint8_t TELEMETRY_FRAME_SIZE = 10;
static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };
uint8_t bufferPosition = 0;
SerialCommand cmd;

void unknownCommand(void);
void setMotorRPM(void);
void moveStepper(void);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  esc.attach(MOTOR, MIN_SPEED, MAX_SPEED);
  pinMode(MOTOR, OUTPUT);
  pinMode(POT, INPUT);
  
  motorPID.SetOutputLimits(1000, 6500);
  motorPID.SetSampleTime(10);
  motorPID.SetMode(AUTOMATIC);

  cmd.addCommand("M", setMotorRPM);
  cmd.addDefaultHandler(unknownCommand);
  // Home stepper motors. We can use sensorless cal with T2209
}

void loop()
{  
  cmd.readSerial();
  //uint16_t val = map(analogRead(POT), 0, 1023, MIN_SPEED, MAX_SPEED);
  //esc.writeMicroseconds(val);
  
  while(Serial2.available()){
    telemetryBuffer[bufferPosition++] = Serial2.read();
    if(bufferPosition == TELEMETRY_FRAME_SIZE){
      // feature: add crc data validation
      bufferPosition = 0;
      float eRevPerMin = (telemetryBuffer[7] << 8) | telemetryBuffer[8];
      //To get the real Rpm of the motor, divide the Erpm by the magnetpole count divided by two.
      inRPM = (eRevPerMin*100) / 11.0;
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
        Serial.print("RPM:");
        Serial.println(inRPM);
      #endif
    }
  }

  if(motorArmed){
    // Serial.print("Out:");
    // Serial.println(outRPM);
    // Serial.print("In:");
    // Serial.println(inRPM);
    // Serial.print("Motor time:");
    // Serial.println(motorSpeed);
    motorPID.Compute();
    if(targetRPM == 0) esc.writeMicroseconds(1000);
    else if(inRPM == 0) esc.writeMicroseconds(motorSpeed++);
    else if(inRPM != targetRPM){
      motorSpeed = map(outRPM, 1000, 6500, MIN_SPEED, MAX_SPEED);
      esc.writeMicroseconds(motorSpeed);
    } 
    else motorArmed = false;
  }
}

void setMotorRPM()
{
  char* arg = cmd.next();
  char* garbage = NULL;
  
  if(arg != NULL){
    targetRPM = (uint32_t) strtol(arg, &garbage,0);
    Serial.println(targetRPM);
  }
  motorArmed = true;
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

//compare this returned value to the value with crc byte
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}