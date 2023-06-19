#include <Arduino.h>
#include <Servo.h> // Disabled definition of timer
//#include <TMC2209.h> // Uses softserial which restricts cpu time. Should only use softserial in setup

#define DEBUG
const uint16_t MIN_SPEED = 1040;
const uint16_t MAX_SPEED =  2000;

const uint8_t FAN = 9;
const uint8_t MOTOR = 11;
const uint8_t POT = 58;

const uint8_t RX_PIN = 63;
const uint8_t TX_PIN = 40;
const uint8_t ENABLE_PIN = 38;
const uint8_t STEP_PIN = 54;
const uint8_t DIR_PIN = 55;

//SoftwareSerial soft_serial(RX_PIN, TX_PIN);
//TMC2209 stepper_driver;
const int32_t MAX_STEP_VELOCITY = 210000;
const int32_t STOP_VELOCITY = 0;
const int RUN_DURATION = 2000;
const int STOP_DURATION = 1000;
const int RUN_COUNT = 4;
Servo esc;

volatile uint32_t stepSpeed;
volatile uint32_t stepCount = 0;
 /*
  System clock 16 Mhz and Prescalar 8;
  Timer 1 speed = 16Mhz/8 = 2 MHz    
  Pulse time = 1/2 MHz  =  500ns
  Count up to = 5us / 500ns = 10 (so this is the value the OCR register should have)
*/  
volatile uint32_t reload = 10;

#define TELEMETRY_FRAME_SIZE 10
static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };
uint8_t bufferPosition = 0;
float eRevPerMin = 0;
uint8_t temperature = 0;

ISR(TIMER3_COMPA_vect){
  stepCount++;
  TCNT3 = 0;  
  OCR3A = reload * stepSpeed;
  if(stepSpeed < 4990){
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
  }
  if(stepCount == 100){
    //digitalWrite(ENABLE_PIN, !digitalRead(ENABLE_PIN));
    stepCount = 0;
  }
} //Timer 3 10us counter

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  Serial2.begin(115200);
  digitalWrite(FAN, HIGH);
  esc.attach(MOTOR, MIN_SPEED, MAX_SPEED);
  pinMode(FAN, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(POT, INPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  // stepper_driver.setup(soft_serial);
  // stepper_driver.setHardwareEnablePin(ENABLE_PIN);
  // stepper_driver.setRunCurrent(100);
  // stepper_driver.enableCoolStep();
  // stepper_driver.moveAtVelocity(MAX_STEP_VELOCITY);
  // //stepper_driver.enable();

  //Timer 3 configuration
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1<<CS31);
  TIMSK3 |= (1<<OCIE3A);
  OCR3A = reload;
  interrupts();
}

void loop() {
  while(Serial2.available()){
    telemetryBuffer[bufferPosition++] = Serial2.read();

    if(bufferPosition == TELEMETRY_FRAME_SIZE){
      bufferPosition = 0;
      // - Temperature (resolution 1°C)
      // - Voltage (resolution 0.01V)
      // - Current (resolution 0.01A)
      // - Consumption (resolution 1mAh)
      // - Electrical Rpm (resolution 100Rpm)
      eRevPerMin = (telemetryBuffer[7] << 8) | telemetryBuffer[8];
      //Note: To get the real Rpm of the motor, divide the Erpm by the magnetpole count divided by two.
      uint32_t revPerMin = (eRevPerMin*100) / 11.0;

      // feature: add crc data validation
      #ifdef DEBUG
      //float motorCurrent = 0;
      //float motorVoltage = 0;
      //motorVoltage = (telemetryBuffer[2] << 8) | telemetryBuffer[1];
      //motorCurrent = (telemetryBuffer[3] << 8) | telemetryBuffer[4];
      // Serial.print("Voltage(V):");
      // Serial.println(motorVoltage,3);
      // Serial.print("Current(A):");
      // Serial.println(motorCurrent,3);
      Serial.print("RPM:");
      Serial.println(revPerMin);
      #endif
    }
  }

  uint16_t val = analogRead(POT);
  uint16_t motorspeed = map(val, 0, 1023, MIN_SPEED, MAX_SPEED);
  analogWrite(FAN, map(val, 0, 1023, 0, 255)); // timer 2 is used for pwm
  
  //figure out best way to drive stepper. Currently ideling at 75mA
  stepSpeed = map(val, 0, 1023, 5000, 50);
  
  esc.writeMicroseconds(motorspeed);

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