#include <PID_v1.h>
#include <AFMotor.h>
#include <QTRSensors.h>

#define KP 0.5
#define KI 2
#define KD 2
#define M1_minumum_speed 50
#define M2_minumum_speed 50
#define M1_maksimum_speed 255
#define M2_maksimum_speed 255
#define MIDDLE_SENSOR 3
#define NUM_SENSORS 5
#define TIMEOUT 2500
#define EMITTER_PIN 2
#define DEBUG 0

double Input;
double motorSpeed;
double Setpoint;

AF_DCMotor motor1(1); 
AF_DCMotor motor2(2);

QTRSensors qtr;

uint16_t sensorValues[NUM_SENSORS];

PID pid(&Input, &motorSpeed, &Setpoint, KP, KI, KD, DIRECT);

void setup() 
{
  Serial.begin(9600);

  Setpoint = 2000;

  pid.SetMode(AUTOMATIC);
  pid.SetTunings(KP, KI, KD);
  

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

int lastError = 0;
//int last_proportional = 0;
//int integral = 0;

void loop() 
{
  int position = qtr.readLineBlack(sensorValues);

  Input = map(position, 0, 5000, 0, 255);

  pid.Compute();

  /*
  int error = position - 2000;  
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  */

  int leftMotorSpeed = M1_minumum_speed + motorSpeed;
  int rightMotorSpeed = M2_minumum_speed - motorSpeed;

  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_maksimum_speed ) motor1speed = M1_maksimum_speed;
  if (motor2speed > M2_maksimum_speed ) motor2speed = M2_maksimum_speed;
  if (motor1speed < 0) motor1speed = 0; 
  if (motor2speed < 0) motor2speed = 0; 
  motor1.setSpeed(motor1speed); 
  motor2.setSpeed(motor2speed);
  motor1.run(FORWARD); 
  motor2.run(FORWARD);
}
