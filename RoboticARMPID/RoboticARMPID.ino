/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <SoftI2C_Adafruit_MotorShield.h>
#include <PID_v1.h>
#include <elapsedMillis.h>
#include "Ultrasonic.h"

#define DEBUG 1

// ADC motor joints pin number
#define M1_ADC_PIN 0
#define M2_ADC_PIN 1
#define M3_ADC_PIN 2
#define M4_ADC_PIN 3
#define IR_ADC_PIN 4

//Define known positions
#define M1_CLOSE 228
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
#define M2_SPEED_1 50
#define M3_SPEED_1 60
#define M4_SPEED_1 60

// Slower speed to improve accuracy
#define M1_SPEED_2 30
#define M2_SPEED_2 30
#define M3_SPEED_2 22
#define M4_SPEED_2 25

// Slower speed to improve accuracy
#define M1_SPEED_3 30
#define M2_SPEED_3 30
#define M3_SPEED_3 22
#define M4_SPEED_3 20

#define US_MODE 1
#define ARM_MODE 2
#define NO_MODE 0

#define IR_SEES_BALL 420

#define MOTOR_ERROR -1

#define red_trig_pin  12
#define red_echo_pin  13
#define blue_trig_pin 10
#define blue_echo_pin 11
#define THRESHOLD     12.0 //inches

#define FREQ   38000
#define BEACON 4
#define D_SIZE 8

#define WAIT_NEXT() delay(1000/(BEACON * D_SIZE) )
#define WAIT_HALF() delay(1000/(2 * BEACON * D_SIZE) )

const uint8_t target = 0x57;
uint8_t bit_rd1;
uint8_t bit_rd2;

int moveMotorTo(int pMotorNum, int pTargetValue);
void movePIDMotorTo(int pMotorNum, int pTargetValue);
void readTrimpots();
int getIrSignal();

elapsedMillis timer0;

//Ultrasonic sensor declarations
Ultrasonic us_red(red_trig_pin, red_echo_pin);
Ultrasonic us_blue(blue_trig_pin, blue_echo_pin);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4 (1,2,3,4)
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

//PID variables
double Setpoint, Input, Output;

int cur_mode = NO_MODE;

//PID object and tunning
PID motorPID(&Input, &Output, &Setpoint,4,5,1, DIRECT);


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(M1_SPEED_1);
  motor2->setSpeed(M2_SPEED_1);
  motor3->setSpeed(M3_SPEED_1);
  motor4->setSpeed(M4_SPEED_1);

  // PIDs Init
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-100, 100);

  // Timer
  //Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  //Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

void loop() {
  String str;
  int curTargetVal = 0;
  int curMotorNum = 0;
  
  // Read ADC joints values 
  if(Serial.available() > 0)  {
    str = Serial.readStringUntil('\n');
    if(cur_mode == NO_MODE) {
      //== is the same as .equals for arduino strings
      if(str == "u") {
        cur_mode = US_MODE;
      } else if(str == "a") {
        cur_mode = ARM_MODE;
      } else {
        Serial.println("b");
      }
    } else if(cur_mode == US_MODE) {
      if(str == "u") {
        cur_mode = US_MODE;
      } else if(str == "a") {
        cur_mode = ARM_MODE;
      } else if(str == "pollu") {
        String res;
        bool red = false;
        bool blue = false;
        res = "none";
        //Run Ultrasonic code here
        float red_dist = us_red.check_distance();
        float blue_dist = us_blue.check_distance();
        if(red_dist < THRESHOLD) {
          res = "red";
          red = true;
        }
        if(blue_dist < THRESHOLD) {
          res = "blue";
          blue = true;
        }
        if(red && blue) {
          res = "both";
        }  
        Serial.println(res);  
        if(DEBUG == 1) {
          Serial.println(red_dist);   
          Serial.println(blue_dist); 
        }
      } else if(str == "pollb") {
        Serial.println(getIrSignal());
      } else {
        Serial.println("b");
      }
    } else if(cur_mode == ARM_MODE) {
      //== is the same as .equals for arduino strings
      if(str == "u") {
        cur_mode = US_MODE;
      } else if(str == "a") {
        cur_mode = ARM_MODE;
      } else if(str == "read") {
        //output all trimpot readings
        readTrimpots();
      } else if(str == "irtest") {
        IRTest();
      } else if(str == "open") {
        openClaw();
      } else if(str == "drop") {
        openClaw();
        Serial.println("g");
      }  else if(str == "reset") {
        movePIDMotorTo(4, 450);
        movePIDMotorTo(3, 200);
        openClaw();
        Serial.println("g");
      } else if(str == "12cm" || str == "13cm") {
        movePIDMotorTo(4, 290);
        sweepMotor(3, 480, 530);
        closeClaw();
        movePIDMotorTo(4, 450);
        movePIDMotorTo(3, 200);
        if((analogRead(IR_ADC_PIN) >= IR_SEES_BALL) && (analogRead(M1_ADC_PIN) > 215)) {
          char array[20];
          sprintf(array, "g%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        } else {
          openClaw();
          char array[20];
          sprintf(array, "b%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        }
      } else if(str == "14cm") {
        movePIDMotorTo(4, 285);
        sweepMotor(3, 460, 525);
        closeClaw();
        movePIDMotorTo(4, 450);
        movePIDMotorTo(3, 200);
        if((analogRead(IR_ADC_PIN) >= IR_SEES_BALL) && (analogRead(M1_ADC_PIN) > 215)) {
          char array[20];
          sprintf(array, "g%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        } else {
          openClaw();
          char array[20];
          sprintf(array, "b%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        }
      } else if(str == "15cm") {
        movePIDMotorTo(4, 260);
        sweepMotor(3, 420, 490);
        closeClaw();
        movePIDMotorTo(4, 450);
        movePIDMotorTo(3, 200);
        if((analogRead(IR_ADC_PIN) >= IR_SEES_BALL) && (analogRead(M1_ADC_PIN) > 215)) {
          char array[20];
          sprintf(array, "g%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        } else {
          openClaw();
          char array[20];
          sprintf(array, "b%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        }
      } else if(str == "16cm") {
        movePIDMotorTo(4, 255);
        sweepMotor(3, 410, 480);
        closeClaw();
        movePIDMotorTo(4, 450);
        movePIDMotorTo(3, 200);
        if((analogRead(IR_ADC_PIN) >= IR_SEES_BALL) && (analogRead(M1_ADC_PIN) > 215)) {
          char array[20];
          sprintf(array, "g%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        } else {
          openClaw();
          char array[20];
          sprintf(array, "b%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        }
      } else if(str == "17cm") {
        movePIDMotorTo(4, 250);
        sweepMotor(3, 390, 470);
        closeClaw();
        movePIDMotorTo(4, 450);
        movePIDMotorTo(3, 200);
        if((analogRead(IR_ADC_PIN) >= IR_SEES_BALL) && (analogRead(M1_ADC_PIN) > 215)) {
          char array[20];
          sprintf(array, "g%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        } else {
          openClaw();
          char array[20];
          sprintf(array, "b%i", analogRead(IR_ADC_PIN));
          Serial.println(array);
        }
      } else {
        sscanf(str.c_str(),"%d %d", &curMotorNum, &curTargetVal);
        if (curMotorNum == 3 || curMotorNum == 4) {
          movePIDMotorTo(curMotorNum, curTargetVal);
        } else if (curMotorNum == 1 || curMotorNum == 2) {
          moveMotorTo(curMotorNum, curTargetVal);
        } else {
          Serial.println("b");
        }
      }
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
    Serial.println("b");
    return -1;
  }
  
  if (pTargetValue > curMax || pTargetValue < curMin) {
    // Invalid Number Range
    Serial.println("b");
    return -1;
  }

  curADCVal = analogRead(curADCPin);

  //Slow motor speed after first run
  if(abs(curADCVal - pTargetValue) > 100) {
    curMotor->setSpeed(curSpeed1);
  } else if(abs(curADCVal - pTargetValue) > 50) {
    curMotor->setSpeed(curSpeed2);
  } else {
    curMotor->setSpeed(curSpeed3);
  }

  done_flag = false;

  timer0 = 0;

  // Move motor until it reaches position
  if (curADCVal > pTargetValue) {
    curMotor->run(FORWARD);
    
    while(done_flag != true && timer0 < 5000) {
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
      // Motor has reach position
      if (curADCVal <= pTargetValue)
          done_flag = true;
    }
  } else {
    curMotor->run(BACKWARD);
    while(done_flag != true && timer0 < 5000) {
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
  return curADCVal;
}


void movePIDMotorTo(int pMotorNum, int pTargetValue) {
  bool done_flag;
  int curMax = 1024;
  int curMin = 0;
  int curADCPin = 0;
  int counter = 0;
  int error;
  char textbuff[50];
  Adafruit_DCMotor *curMotor;
  
  if(pMotorNum == 1) {
    Serial.println("b");
    return;
  } else if(pMotorNum == 2) {
    Serial.println("b");
    return;
  } else if(pMotorNum == 3) {
    curMotor = motor3;
    curADCPin = M3_ADC_PIN;
    curMax = M3_MAX;
    curMin = M3_MIN;
  } else if(pMotorNum == 4) {
    curMotor = motor4;
    curADCPin = M4_ADC_PIN;
    curMax = M4_MAX;
    curMin = M4_MIN;
  } else {
    // Invalid Motor Number
    Serial.println("b");
    return;
  }

  if (pTargetValue > curMax || pTargetValue < curMin) {
    // Invalid Number Range
    Serial.println("b");
    return;
  }

  // Run PID
  Setpoint = pTargetValue;
  done_flag = false;
  while(done_flag != true) {

    Input = analogRead(curADCPin);
    motorPID.Compute();
    curMotor->setSpeed(abs(Output));

    sprintf(textbuff,"PID Input/ADC = %d, PID output = %d \n", (int)Input, (int)Output);
  
    if(Output < 0) {
      curMotor->run(FORWARD);
    } else {
      curMotor->run(BACKWARD);
    }
  
    error = abs(Input - Setpoint);
    if (error <= 1) {
      counter++;
      if (counter == 20) {
        curMotor->run(RELEASE);
        done_flag = true;
      }
    }   
  } 
}


int sweepMotor(int pMotorNum, int pLow, int pHigh) {
  //First, move the motor to the correct starting position
  movePIDMotorTo(pMotorNum, pLow);

  //Next set starting conditions and perform a move with a single speed
  int curADCVal = 0;
  int curADCPin = 0;
  int curMax = 1024;
  int curMin = 0;
  int curSpeed = 0;
  int curIRVal = 0;
  int maxIRVal = 0;
  int maxIRVal2 = 0;
  Adafruit_DCMotor *curMotor;
  bool done_flag;
  
  if(pMotorNum == 1) {
    curMotor = motor1;
    curADCPin = M1_ADC_PIN;
    curMax = M1_MAX;
    curMin = M1_MIN;
    curSpeed = M1_SPEED_3;
  } else if(pMotorNum == 2) {
    curMotor = motor2;
    curADCPin = M2_ADC_PIN;
    curMax = M2_MAX;
    curMin = M2_MIN;
    curSpeed = M2_SPEED_3;
  } else if(pMotorNum == 3) {
    curMotor = motor3;
    curADCPin = M3_ADC_PIN;
    curMax = M3_MAX;
    curMin = M3_MIN;
    curSpeed = M3_SPEED_3;
  } else if(pMotorNum == 4) {
    curMotor = motor4;
    curADCPin = M4_ADC_PIN;
    curMax = M4_MAX;
    curMin = M4_MIN;
    curSpeed = M4_SPEED_3;
  } else {
    // Invalid Motor Number
    Serial.println("b");
    return -1;
  }
  
  if (pLow > curMax || pLow < curMin || pHigh > curMax || pHigh < curMin) {
    // Invalid Number Range
    Serial.println("b");
    return -1;
  }

  curADCVal = analogRead(curADCPin);

  curMotor->setSpeed(curSpeed);

  done_flag = false;

  // Move motor until it reaches position
  if (curADCVal > pHigh) {
    curMotor->run(FORWARD);
    
    while(done_flag != true) {
      curIRVal = analogRead(IR_ADC_PIN);
      maxIRVal = max(curIRVal, maxIRVal);
      curADCVal = analogRead(curADCPin);
      // Motor has reach position
      if (curADCVal <= pHigh)
          done_flag = true;
    }
  } else {
    curMotor->run(BACKWARD);
    while(done_flag != true) {
      curIRVal = analogRead(IR_ADC_PIN);
      maxIRVal = max(curIRVal, maxIRVal);
      curADCVal = analogRead(curADCPin);
      // Motor has reach position
      if (curADCVal >= pHigh)
          done_flag = true;
    }
  }

  // Release motors
  curMotor->run(RELEASE);

  delay(500); //delay for half a second to let the motor settle

  curIRVal = analogRead(IR_ADC_PIN);
  delay(10);  //allow the read to finish


  //Sweep in reverse order and search for the max IR value
  curADCVal = analogRead(curADCPin);

  done_flag = false;

  // Move motor until it reaches position
  if (curADCVal > pLow) {
    curMotor->run(FORWARD);
    
    while(done_flag != true) {
      curIRVal = analogRead(IR_ADC_PIN);
      maxIRVal2 = max(curIRVal, maxIRVal2);
      curADCVal = analogRead(curADCPin);
      // Motor has reach position
      if (curADCVal <= pLow) {
        done_flag = true;
      } else if(curIRVal >= (maxIRVal-8)) {
        done_flag = true;
        curMotor->run(RELEASE);
        return curIRVal;
        
      }
    }
  } else {
    curMotor->run(BACKWARD);
    while(done_flag != true) {
      curIRVal = analogRead(IR_ADC_PIN);
      maxIRVal2 = max(curIRVal, maxIRVal2);
      curADCVal = analogRead(curADCPin);
      // Motor has reach position
      if (curADCVal >= pLow) {
        done_flag = true;
      } else if(curIRVal >= (maxIRVal-8)) {
        done_flag = true;
        curMotor->run(RELEASE);
        return curIRVal;
      }
    }
  }

  // Release motors
  curMotor->run(RELEASE);

  delay(500); //delay for half a second to let the motor settle

  curIRVal = analogRead(IR_ADC_PIN);
  delay(10);  //allow the read to finish

  //Sweep in reverse order a final time to try and find the max IR value
  curADCVal = analogRead(curADCPin);

  done_flag = false;

  // Move motor until it reaches position
  if (curADCVal > pHigh) {
    curMotor->run(FORWARD);
    
    while(done_flag != true) {
      curIRVal = analogRead(IR_ADC_PIN);
      curADCVal = analogRead(curADCPin);
      // Motor has reach position
      if (curADCVal <= pHigh) {
        done_flag = true;
      } else if(curIRVal >= (maxIRVal-8) || curIRVal >= (maxIRVal2-8)) {
        done_flag = true;
        curMotor->run(RELEASE);
        return curIRVal;
        
      }
    }
  } else {
    curMotor->run(BACKWARD);
    while(done_flag != true) {
      curIRVal = analogRead(IR_ADC_PIN);
      curADCVal = analogRead(curADCPin);
      // Motor has reach position
      if (curADCVal >= pHigh) {
        done_flag = true;
      } else if(curIRVal >= (maxIRVal-8) || curIRVal >= (maxIRVal2-8)) {
        done_flag = true;
        curMotor->run(RELEASE);
        return curIRVal;
      }
    }
  }

  // Release motors
  curMotor->run(RELEASE);

  delay(500); //delay for half a second to let the motor settle

  curIRVal = analogRead(IR_ADC_PIN);
  delay(10);  //allow the read to finish







  
  return curIRVal;
}
















bool isClawOpen() {
  int ADCVal = analogRead(M1_ADC_PIN);
  delay(10);
  if(ADCVal > 350) {
    return true;
  } else {
    return false;
  }
}

bool openClaw() {
  while(!isClawOpen()) {
    moveMotorTo(1, M1_RESET);
  }
  return true;
}

bool closeClaw() {
  moveMotorTo(1, M1_CLOSE);
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
  ADCVal = analogRead(IR_ADC_PIN);
  delay(10);
  Serial.print("IR: ");
  Serial.println(ADCVal);  //for debugging
}

void IRTest() {
  while(1) {
    int IRValue = analogRead(4);
    Serial.println(IRValue);  //for debugging
    delay(100);
  }
}

bool checkIRRange(int lower, int higher) {
  int goodCount = 0; 
  int badCount = 0;
  int IRValue = 0;
  
  for(int i = 0; i < 10; i++) {
    IRValue = analogRead(4);
    delay(10);
    if(IRValue >= lower && IRValue <= higher) {
      goodCount++;
    } else {
      badCount++;
    }
  }

  if(goodCount >= 5) {
    return true;
  } else {
    return false;
  }
}

int getIrSignal() {
  int i;
  uint8_t ir_signal;
  uint8_t res;
  
  // Nyquist
  i = 0;
  ir_signal = 0;
  while(2*D_SIZE - 1 > i++) {
	  if(ir_signal == target) { // Success
		  return 1;
 		}

		bit_rd1 = !digitalRead(7);
		WAIT_HALF();
		bit_rd2 = !digitalRead(7);
		ir_signal = (ir_signal << 1) | ( (bit_rd1 == bit_rd2) ? bit_rd2 : bit_rd1);
		WAIT_HALF();
  }
  
  // Fail (this time)
  return 0;
}

