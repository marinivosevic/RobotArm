#include <Stepper.h>
#include <Arduino.h>
#include <Servo.h>

const int stepsPerRevolution = 2000;
const int stepperButtonPinLeft = 2;
const int stepperButtonPinRight = 3;
const int servoButtonPinLeft = 4;
const int servoButtonPinRight = 5;
int stepperLeftButtonState = 0;
int stepperRightButtonState = 0;
int servoLeftButtonState = 0;
int servoRightButtonState = 0;
bool leftMotorEnabled = false;
bool rightMotorEnabled = false;
int latchPin = 0; // Latch pin of 74HC595 is connected to Digital pin 5
int clockPin = 1; // Clock pin of 74HC595 is connected to Digital pin 6
int dataPin = 7;

Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);
Servo gripperServo;

void updateShiftRegister(int joyXValue, int joyYvalue)
{
  digitalWrite(latchPin, LOW);
  if (joyYvalue >= 1000 || joyYvalue <= 50)
  {
    shiftOut(dataPin, clockPin, MSBFIRST, 0B00001111);
    
  }
  else if (joyXValue >= 1000 || joyXValue <= 50)
  {
    shiftOut(dataPin, clockPin, MSBFIRST, 0B11110000);
  }
}

void setup()
{

  Serial.begin(9600);
  pinMode(stepperButtonPinLeft, INPUT_PULLUP);
  pinMode(stepperButtonPinRight, INPUT_PULLUP);
  pinMode(servoButtonPinLeft, INPUT_PULLUP);
  pinMode(servoButtonPinRight, INPUT_PULLUP);
  myStepper.setSpeed(10);
  gripperServo.attach(6);
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
}

void loop()
{
  byte stepperPins = 0B00000000;
  stepperLeftButtonState = digitalRead(stepperButtonPinLeft);
  stepperRightButtonState = digitalRead(stepperButtonPinRight);
  servoLeftButtonState = digitalRead(servoButtonPinLeft);
  servoRightButtonState = digitalRead(servoButtonPinRight);

  int joyXValue = analogRead(A1);
  int joyYValue = analogRead(A0);
  Serial.print(joyYValue);
  Serial.print("\n");
  updateShiftRegister(joyXValue, joyYValue);

  // body stepper rotation
  if (stepperLeftButtonState == HIGH)
  {

    leftMotorEnabled = !leftMotorEnabled;
    delay(10);
  }
  else
  {
    leftMotorEnabled = false;
  }

  if (stepperRightButtonState == HIGH)
  {

    rightMotorEnabled = !rightMotorEnabled;
    delay(10);
  }
  else
  {
    rightMotorEnabled = false;
  }

  if (leftMotorEnabled)
  {

    myStepper.step(20);
  }

  if (rightMotorEnabled)
  {

    myStepper.step(-20);
  }

  // Servo for gripper
  if (servoLeftButtonState == HIGH)
  {

    gripperServo.write(170);
  }
  if (servoRightButtonState == HIGH)
  {

    gripperServo.write(90);
  }
}
