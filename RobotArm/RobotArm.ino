/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <SoftI2C_Adafruit_MotorShield.h>

// ADC motor joints pin number
#define M1_ADC_PIN 0
#define M2_ADC_PIN 4
#define M3_ADC_PIN 2
#define M4_ADC_PIN 3
#define IR_ADC_PIN 1

//Define known positions
#define M1_CLOSE 225
#define M1_OPEN 350

#define M1_RESET 350
#define M2_RESET 500
#define M3_RESET 100
#define M4_RESET 500

// Trimpot max values
#define M1_MAX 350  //open claw
#define M2_MAX 775  //down
#define M3_MAX 650  //down
#define M4_MAX 650  //up

// Trimpot min values
#define M1_MIN 150  //closed claw
#define M2_MIN 350  //up
#define M3_MIN 0  //up
#define M4_MIN 70  //down

// Initial speed for quickly moving to a position
#define M1_SPEED_1 30
#define M2_SPEED_1 70
#define M3_SPEED_1 60
#define M4_SPEED_1 60

// Slower speed to improve accuracy
#define M1_SPEED_2 30
#define M2_SPEED_2 30
#define M3_SPEED_2 20
#define M4_SPEED_2 25

// Slower speed to improve accuracy
#define M1_SPEED_3 30
#define M2_SPEED_3 30
#define M3_SPEED_3 20
#define M4_SPEED_3 20

#define MOTOR_ERROR -1

int moveMotorTo(int pMotorNum, int pTargetValue);
void readTrimpots();

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4 (1,2,3,4)
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(M1_SPEED_1);
  motor2->setSpeed(M2_SPEED_1);
  motor3->setSpeed(M3_SPEED_1);
  motor4->setSpeed(M4_SPEED_1);

  // Timer
  //Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  //Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

void loop() {
  String str;
  int curTargetVal = 0;
  int curMotorNum = 0;
  
  // Read ADC joints values 
  while(Serial.available() > 0)  {
    str = Serial.readStringUntil('\n');

    //== is the same as .equals for arduino strings
    if(str == "read") {
      //output all trimpot readings
      readTrimpots();
    } else if(str == "irtest") {
      IRTest();
    } else if(str == "reset") {
      moveMotorTo(4, M4_RESET);
      moveMotorTo(3, M3_RESET);
      moveMotorTo(2, M2_RESET);
      moveMotorTo(1, M1_RESET);
    } else if(str == "6cm") {
      moveMotorTo(4, 230);
      moveMotorTo(3, 575);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "7cm") {
      moveMotorTo(4, 230);
      moveMotorTo(3, 555);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "8cm") {
      moveMotorTo(4, 230);
      moveMotorTo(3, 538);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "9cm") {
      moveMotorTo(4, 220);
      moveMotorTo(3, 520);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "10cm") {
      moveMotorTo(4, 220);
      moveMotorTo(3, 508);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "11cm") {
      moveMotorTo(4, 220);
      moveMotorTo(3, 500);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "12cm") {
      moveMotorTo(4, 210);
      moveMotorTo(3, 484);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "13cm") {
      moveMotorTo(4, 175);
      moveMotorTo(3, 427);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "14cm") {
      moveMotorTo(4, 150);
      moveMotorTo(3, 368);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else if(str == "15cm") {
      moveMotorTo(4, 125);
      moveMotorTo(3, 339);
      moveMotorTo(3, 339);
      moveMotorTo(3, 339);
      moveMotorTo(2, 500);
      moveMotorTo(1, M1_CLOSE);
    } else {
      sscanf(str.c_str(),"%d %d", &curMotorNum, &curTargetVal);
      moveMotorTo(curMotorNum, curTargetVal);
    }
  }
}

int moveMotorTo(int pMotorNum, int pTargetValue) {
  int curADCVal = 0;
  int curADCPin = 0;
  int curMax = 1024;
  int curMin = 0;
  int curSpeed1 = 0;
  int curSpeed2 = 0;
  int curSpeed3 = 0;
  int curIRValue = 0;
  Adafruit_DCMotor *curMotor;
  bool done_flag;
  
  if(pMotorNum == 1) {
    curMotor = motor1;
    curADCPin = M1_ADC_PIN;
    curMax = M1_MAX;
    curMin = M1_MIN;
    curSpeed1 = M1_SPEED_1;
    curSpeed2 = M1_SPEED_2;
    curSpeed3 = M1_SPEED_3;
  } else if(pMotorNum == 2) {
    curMotor = motor2;
    curADCPin = M2_ADC_PIN;
    curMax = M2_MAX;
    curMin = M2_MIN;
    curSpeed1 = M2_SPEED_1;
    curSpeed2 = M2_SPEED_2;
    curSpeed3 = M2_SPEED_3;
  } else if(pMotorNum == 3) {
    curMotor = motor3;
    curADCPin = M3_ADC_PIN;
    curMax = M3_MAX;
    curMin = M3_MIN;
    curSpeed1 = M3_SPEED_1;
    curSpeed2 = M3_SPEED_2;
    curSpeed3 = M3_SPEED_3;
  } else if(pMotorNum == 4) {
    curMotor = motor4;
    curADCPin = M4_ADC_PIN;
    curMax = M4_MAX;
    curMin = M4_MIN;
    curSpeed1 = M4_SPEED_1;
    curSpeed2 = M4_SPEED_2;
    curSpeed3 = M4_SPEED_3;
  } else {
    // Invalid Motor Number
    Serial.println("Invalid Motor Number");  //for debugging
    Serial.println(MOTOR_ERROR);
    return -1;
  }
  
  if (pTargetValue > curMax || pTargetValue < curMin) {
    // Invalid Number Range
    Serial.println("Invalid Number Range");  //for debugging
    Serial.println(MOTOR_ERROR);
    return -1;
  }
  Serial.println(pTargetValue);  //for debugging

  curADCVal = analogRead(curADCPin);
  Serial.println(curADCVal);  //for debugging

  //Slow motor speed after first run
  if(abs(curADCVal - pTargetValue) > 100) {
    curMotor->setSpeed(curSpeed1);
  } else if(abs(curADCVal - pTargetValue) > 50) {
    curMotor->setSpeed(curSpeed2);
  } else {
    curMotor->setSpeed(curSpeed3);
  }

  done_flag = false;

  // Move motor until it reaches position
  if (curADCVal > pTargetValue) {
    Serial.println("Moving FORWARD"); //for debugging
    curMotor->run(FORWARD);
    
    while(done_flag != true) {
      curIRValue = analogRead(IR_ADC_PIN);
      curADCVal = analogRead(curADCPin);
      delay(10);
      //check how close to target we are and then slow if needed
      if(abs(curADCVal - pTargetValue) > 100) {
        curMotor->setSpeed(curSpeed1);
      } else if(abs(curADCVal - pTargetValue) > 50) {
        curMotor->setSpeed(curSpeed2);
      } else {
        curMotor->setSpeed(curSpeed3);
      }
      Serial.println(curADCVal);  //for debugging
      // Motor has reach position
      if (curADCVal <= pTargetValue)
          done_flag = true;
    }
  } else {
    Serial.println("Moving BACKWARD");  //for debugging
    curMotor->run(BACKWARD);
    while(done_flag != true) {
      curIRValue = analogRead(IR_ADC_PIN);
      curADCVal = analogRead(curADCPin);
      delay(10);
      //check how close to target we are and then slow if needed
      if(abs(curADCVal - pTargetValue) > 100) {
        curMotor->setSpeed(curSpeed1);
      } else if(abs(curADCVal - pTargetValue) > 50) {
        curMotor->setSpeed(curSpeed2);
      } else {
        curMotor->setSpeed(curSpeed3);
      }
      Serial.println(curADCVal);  //for debugging
      // Motor has reach position
      if (curADCVal >= pTargetValue)
          done_flag = true;
    }
  }

  // Release motors
  curMotor->run(RELEASE);

  delay(500); //delay for half a second to let the motor settle

  curADCVal = analogRead(curADCPin);
  delay(10);  //allow the read to finish
  Serial.println(curADCVal);  //for debugging
  return curADCVal;
}

void readTrimpots() {
  int ADCVal = analogRead(M1_ADC_PIN);
  delay(10);
  Serial.print("Trimpot 1: ");
  Serial.println(ADCVal);  //for debugging
  ADCVal = analogRead(M2_ADC_PIN);
  delay(10);
  Serial.print("Trimpot 2: ");
  Serial.println(ADCVal);  //for debugging
  ADCVal = analogRead(M3_ADC_PIN);
  delay(10);
  Serial.print("Trimpot 3: ");
  Serial.println(ADCVal);  //for debugging
  ADCVal = analogRead(M4_ADC_PIN);
  delay(10);
  Serial.print("Trimpot 4: ");
  Serial.println(ADCVal);  //for debugging
}

void IRTest() {
  while(1) {
    int IRValue = analogRead(IR_ADC_PIN);
    Serial.println(IRValue);  //for debugging
    delay(100);
  }
}

