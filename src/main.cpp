#include <Stepper.h>
#include <Arduino.h>
#include <Servo.h>

#define number_of_74hc595s 1 
#define numOfRegisterPins number_of_74hc595s * 8

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

int SER_Pin = 8;   //pin 14 on the 75HC595
int RCLK_Pin = 9;  //pin 12 on the 75HC595
int SRCLK_Pin = 10; //pin 11 on the 75HC595

int motorPin1 = 0;	// Blue   - 28BYJ48 pin 1
int motorPin2 = 1;	// Pink   - 28BYJ48 pin 2
int motorPin3 = 2;	// Yellow - 28BYJ48 pin 3
int motorPin4 = 3;	// Orange - 28BYJ48 pin 4
int motorSpeed = 0;     //variable to set stepper speed

boolean registers[numOfRegisterPins];

Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);
Servo gripperServo;

//How many of the shift registers - change this
#define number_of_74hc595s 1 

//do not touch
#define numOfRegisterPins number_of_74hc595s * 8

boolean registers[numOfRegisterPins];

//set all register pins to LOW
void clearRegisters(){
  for(int i = numOfRegisterPins - 1; i >=  0; i--){
     registers[i] = LOW;
  }
} 


//Set and display registers
//Only call AFTER all values are set how you would like (slow otherwise)
void writeRegisters(){

  digitalWrite(RCLK_Pin, LOW);

  for(int i = numOfRegisterPins - 1; i >=  0; i--){
    digitalWrite(SRCLK_Pin, LOW);

    int val = registers[i];

    digitalWrite(SER_Pin, val);
    digitalWrite(SRCLK_Pin, HIGH);

  }
  digitalWrite(RCLK_Pin, HIGH);

}

//set an individual pin HIGH or LOW
void setRegisterPin(int index, int value){
  registers[index] = value;
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
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  clearRegisters();
  writeRegisters();
}

void loop()
{
  
  stepperLeftButtonState = digitalRead(stepperButtonPinLeft);
  stepperRightButtonState = digitalRead(stepperButtonPinRight);
  servoLeftButtonState = digitalRead(servoButtonPinLeft);
  servoRightButtonState = digitalRead(servoButtonPinRight);

  int joyXValue = analogRead(A1);
  int joyYValue = analogRead(A0);
  
  Serial.print(joyYValue);
  Serial.print("\n");


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

  if (joyYValue >= 1000 || joyYValue <= 50)
  {
    motorSpeed = 100;
    if(joyYValue >= 500){
      activeMotor1(1);
    }else{
      activeMotor1(-1);
    }
    
  }
  else if (joyXValue >= 1000 || joyXValue <= 50)
  {
    motorSpeed = 100;
     if(joyXValue >= 500){
      activeMotor2(1);
    }else{
      activeMotor2(-1);
    }
  }
}

void activeMotor1(int direction){
  int pin1 = 0;
  int pin2 = 1;
  int pin3 = 2;
  int pin4 = 3;
  
  if(direction == 1){
    clockwise(pin1,pin2,pin3,pin4);
  }else{
    counterclockwise(pin1,pin2,pin3,pin4);
  }
}

void activeMotor2(int direction){
  int pin1 = 4;
  int pin2 = 5;
  int pin3 = 6;
  int pin4 = 7;
  
  if(direction == 1){
    clockwise(pin1,pin2,pin3,pin4);
  }else{
    counterclockwise(pin1,pin2,pin3,pin4);
  }
}

void counterclockwise(int pin1,int pin2,int pin3,int pin4) {
  // Step 1
  setMotorPins(pin1,pin2,pin3,pin4,HIGH, LOW, LOW, LOW);
  delay(motorSpeed);

  // Step 2
  setMotorPins(pin1,pin2,pin3,pin4,HIGH, HIGH, LOW, LOW);
  delay(motorSpeed);

  // Step 3
  setMotorPins(pin1,pin2,pin3,pin4,LOW, HIGH, LOW, LOW);
  delay(motorSpeed);

  // Step 4
  setMotorPins(pin1,pin2,pin3,pin4,LOW, HIGH, HIGH, LOW);
  delay(motorSpeed);

  // Step 5
  setMotorPins(pin1,pin2,pin3,pin4,LOW, LOW, HIGH, LOW);
  delay(motorSpeed);

  // Step 6
  setMotorPins(pin1,pin2,pin3,pin4,LOW, LOW, HIGH, HIGH);
  delay(motorSpeed);

  // Step 7
  setMotorPins(pin1,pin2,pin3,pin4,LOW, LOW, LOW, HIGH);
  delay(motorSpeed);

  // Step 8
  setMotorPins(pin1,pin2,pin3,pin4,HIGH, LOW, LOW, HIGH);
  delay(motorSpeed);
}

void clockwise(int pin1,int pin2,int pin3,int pin4) {
  // Step 1
  setMotorPins(pin1,pin2,pin3,pin4,HIGH, LOW, LOW, LOW);
  delay(motorSpeed);

  // Step 2
  setMotorPins(pin1,pin2,pin3,pin4,HIGH, HIGH, LOW, LOW);
  delay(motorSpeed);

  // Step 3
  setMotorPins(pin1,pin2,pin3,pin4,LOW, HIGH, LOW, LOW);
  delay(motorSpeed);

  // Step 4
  setMotorPins(pin1,pin2,pin3,pin4,LOW, HIGH, HIGH, LOW);
  delay(motorSpeed);

  // Step 5
  setMotorPins(pin1,pin2,pin3,pin4,LOW, LOW, HIGH, LOW);
  delay(motorSpeed);

  // Step 6
  setMotorPins(pin1,pin2,pin3,pin4,LOW, LOW, HIGH, HIGH);
  delay(motorSpeed);

  // Step 7
  setMotorPins(pin1,pin2,pin3,pin4,LOW, LOW, LOW, HIGH);
  delay(motorSpeed);

  // Step 8
  setMotorPins(pin1,pin2,pin3,pin4,HIGH, LOW, LOW, HIGH);
  delay(motorSpeed);
}

void setMotorPins(int pin1,int pin2,int pin3,int pin4, int pin1State, int pin2State, int pin3State, int pin4State) {
  setRegisterPin(pin1, pin1State);
  setRegisterPin(pin2, pin2State);
  setRegisterPin(pin3, pin3State);
  setRegisterPin(pin4, pin4State);
  writeRegisters();
}