/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/
#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h> // Disabled definition of timer 3 & 4
#define DEBUG 

const uint8_t MOTOR = 11;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double Kp=1, Ki=0.05, Kd=0.25;

//Specify the links and initial tuning parameters
const uint8_t TELEMETRY_FRAME_SIZE = 10;
static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };
uint8_t bufferPosition = 0;
double inRPM;
double outRPM;
double targetRPM;
PID motorPID(&inRPM, &outRPM, &targetRPM, Kp, Ki, Kd, DIRECT);
Servo esc;

const uint16_t MIN_SPEED = 1060;
const uint16_t MAX_SPEED =  2000;
uint16_t motorSpeed = MIN_SPEED;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(MOTOR, OUTPUT);
  esc.attach(MOTOR, MIN_SPEED, MAX_SPEED);
  targetRPM = 4000;

  //turn the PID on
  motorPID.SetMode(AUTOMATIC);
}

void loop()
{
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

  double gap = abs(targetRPM-inRPM); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    motorPID.SetTunings(Kp, Ki, Kd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     motorPID.SetTunings(aggKp, aggKi, aggKd);
  }

  motorPID.Compute();
  motorSpeed = map(outRPM, 1000, 6500, MIN_SPEED, MAX_SPEED); // Map current PID value to motor speed
  esc.writeMicroseconds(motorSpeed);
}


