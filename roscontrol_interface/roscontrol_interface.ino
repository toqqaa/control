// PID motor position control.
// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <PID_v1.h>
#include "CytronMotorDriver.h"
#include <util/atomic.h>
//#include "ros_node.h"
#include "mpu6050.h"
#define COMMAND_RATE 20 //hz

// Configure the motor driver.
CytronMD motor_left(PWM_PWM, 10, 11);   // PWM 1A = Pin 10, PWM 1B = Pin 11.
CytronMD motor_right(PWM_PWM, 8, 9);   // PWM 2A = Pin 8, PWM 2B = Pin 9.

//****** Left Motor Encoder Pins **********
#define ENCLA 3 // YELLOW
#define ENCLB 2 // GREEN


//****** Right Motor Encoder Pins **********
#define ENCRA 18 // YELLOW
#define ENCRB 19 // GREEN

unsigned long lastInterruptTimeR = 0;  // variable to store the time of the last interrupt
unsigned long lastInterruptTimeL = 0;  // variable to store the time of the last interrupt


float count_per_rev_lm = 228.0;// <======== set this number according to your motor (count per rev) for left motor

float count_per_rev_rm = 224.0;// <======== set this number according to your motor (count per rev) for right motor

unsigned long lastTime, now;

//LEFT MOTOR PARAMTERS

volatile int posi_lm = 0;
double LM_kp = .04, LM_ki = 1.3 , LM_kd = 0.00000;          // modify for optimal performance



volatile long LM_encoderPos = 0, LM_last_pos = 0, LM_lastpos = 0;

//RIGHT MOTOR PARAMTERS

volatile int posi_rm = 0;

double RM_kp = .04, RM_ki = 1.3 , RM_kd = 0.00000;          // modify for optimal performance


volatile long RM_encoderPos = 0, RM_last_pos = 0, RM_lastpos = 0;

PID LMPID(&LM_input, &LM_output, &LM_setpoint, LM_kp, LM_ki, LM_kd, DIRECT);
PID RMPID(&RM_input, &RM_output, &RM_setpoint, RM_kp, RM_ki, RM_kd, DIRECT);

void setup() {
  pinMode(ENCLA, INPUT_PULLUP);
  pinMode(ENCLB, INPUT_PULLUP);
  pinMode(ENCRA, INPUT_PULLUP);
  pinMode(ENCRB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCLA), readEncoder_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCRA), readEncoder_right, RISING);

  LMPID.SetMode(AUTOMATIC);
  LMPID.SetSampleTime(1);
  LMPID.SetOutputLimits(-255, 255);

  RMPID.SetMode(AUTOMATIC);
  RMPID.SetSampleTime(1);
  RMPID.SetOutputLimits(-255, 255);
  ros_setup();
  setupIMU();
  //  Serial.begin (57600);

}

void loop() {
  static bool imu_is_initialized;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_control_time = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Read the position in an atomic block to avoid a potential misread if the interrupt coincides with this code running
    unsigned long interruptTimeL = millis();  // get the current time
    if (interruptTimeL - lastInterruptTimeL > 8) {  // check if enough time has elapsed since the last interrupt
      LM_encoderPos = posi_lm;
      lastInterruptTimeL = interruptTimeL;
    }
    unsigned long interruptTimeR = millis();  // get the current time
    if (interruptTimeR - lastInterruptTimeR > 8) {  // check if enough time has elapsed since the last interrupt
      RM_encoderPos = posi_rm;
      lastInterruptTimeR = interruptTimeR;
    }
  }
  now = millis();
  int timeChange = (now - lastTime);
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {

    LM_pose_cahnge = (360.0 * (LM_encoderPos - LM_lastpos)) / count_per_rev_lm; //change in position in degrees of the wheel
    LM_input = (360.0 * 1000 * (LM_encoderPos - LM_last_pos)) / (count_per_rev_lm * (now - lastTime));

    RM_pose_cahnge = (360.0 * (RM_encoderPos - RM_lastpos)) / count_per_rev_rm; //change in position in degrees of the wheel
    RM_input = (360.0 * 1000 * (RM_encoderPos - RM_last_pos)) / (count_per_rev_rm * (now - lastTime));
    joint_state.data_length = 2;
    joint_state.data[0] = LM_pose_cahnge;
    joint_state.data[1] = RM_pose_cahnge;

    lastTime = now;
    LM_last_pos = LM_encoderPos;
    RM_last_pos = RM_encoderPos;
  }

  LMPID.Compute();                                    // calculate new output
  RMPID.Compute();                                    // calculate new output

  motor_left.setSpeed(LM_output);  // drive L298N H-Bridge module
  motor_right.setSpeed(RM_output);

  //this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU is connected
    if (!imu_is_initialized)
    {
      imu_is_initialized = true;

      if (imu_is_initialized)
        nh.loginfo("IMU Initialized");
      else
        nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    }
    else
    {
      publishIMU();
    }
    prev_imu_time = millis();
  }


  //  Serial.print(LM_output);
  //  Serial.print(" ");
  //  Serial.print(RM_output);
  //  Serial.println();






  ros_loop();

  delay(5);

}



void readEncoder_left() {

  int b = digitalRead(ENCLB);

  if (b > 0) {
    posi_lm ++;

  }
  else {
    posi_lm --;

  }

}

void readEncoder_right() {

  int b = digitalRead(ENCRB);

    if (b > 0) {
      posi_rm ++;

    }
    else {
      posi_rm --;

    }
   
 
}
