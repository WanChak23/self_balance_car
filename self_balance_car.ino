#include <MeMegaPi.h>
const byte interruptPin =18;
const byte NE1=31;

MeMegaPiDCMotor motor1(PORT3B);
MeMegaPiDCMotor motor2(PORT4B);

#include <Wire.h>

MeGyro gyro; 


double kp = 24.6;
double ki = 0;
double kd = 0.4;

unsigned long currentTime, previousTime;
double elapsedTime, error, lastError;
double input, output, setPoint;
double cumError, rateError;


void blink(){}
void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(NE1, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin),blink, RISING);
 
  gyro.begin();
  
  setPoint = 0;
  
}

void loop() {
  gyro.update();
  input = gyro.getAngleY();
  output = computePID(input);
  
  motor2.run(output);
  motor1.run(-1 * output);
  delay(5);
}

double computePID(double inp){
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  error = setPoint - inp;
  cumError += error * elapsedTime;
  rateError = (error - lastError)/elapsedTime;

  double out = (kp*error + ki*cumError + kd*rateError);

  lastError = error;
  previousTime = currentTime;

  return out;
  
  
  }
