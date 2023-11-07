#include <PID_v1.h>
#include <QTRSensors.h>
#include <AFMotor.h>


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output1, output2, setPoint = 2000;
double cumError, rateError;

double Kp=2, Ki=5, Kd=1;
PID myPID1(&input, &output1, &setPoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&input, &output2, &setPoint, Kp, Ki, Kd, DIRECT);

#define KP 1
#define KD 3
#define M1_min_speed 150
#define M2_min_speed 150
#define M1_max_speed 255
#define M2_max_speed 255
#define Mid_Sensor 3
#define Num_Sensors 5
#define Timeout 2500
#define Emiter_Pin 2
#define Debug 1

QTRSensors qtr;

unsigned int sensorVal[Num_Sensors];

//int lastError = 0;

void setup() 
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3, A2, A1, A0}, Num_Sensors);
  delay(1500);
  manual_calibration();
  set_motors(0, 0);
 
  myPID2.SetMode(AUTOMATIC);
}

void loop() 
{
  unsigned int sensors[5];
  input = qtr.readLineBlack(sensors);

  myPID1.Compute();
  myPID2.Compute();

  set_motors(output1, output1);
}

void set_motors(int m1speed, int m2speed)
{
  if (m1speed > M1_max_speed) m1speed = M1_max_speed;
  if (m2speed > M2_max_speed) m2speed = M2_max_speed;
  if (m1speed < 0) m1speed = 0;
  if (m2speed < 0) m2speed = 0;
  motor1.setSpeed(m1speed);
  motor2.setSpeed(m2speed);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
}

void manual_calibration() 
{  
  int i;
  /*
  for (i = 0; i < 2; i++)
  {
    qtr.calibrate();
    set_motors(70, 0);
    delay(500);
    set_motors(0, 70);
    delay(500);
  }
  */

  for (i = 0; i < 250; i++)
    {
      qtr.calibrate();
      delay(20);
    }
  
  if(Debug) 
  {
    Serial.begin(9600);
    for (int i = 0; i < Num_Sensors; i++)
    {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();
      
    for (int i = 0; i < Num_Sensors; i++)
    {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
}
