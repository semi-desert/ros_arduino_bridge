/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
  
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined ADAFRUIT_MOTOR_SHIELD_V2
  /* Wrap the motor driver initialization */
  void initMotorController() {  
    AFMS.begin();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) {
      if (spd < 0) {
         myLeftMotor->run(BACKWARD);
         spd = -spd;
      }
      else myLeftMotor->run(FORWARD);
      myLeftMotor->setSpeed(spd);
    }
    else {
      if (spd < 0) {
         myRightMotor->run(BACKWARD);
         spd = -spd;
      }
      else myRightMotor->run(FORWARD);
      myRightMotor->setSpeed(spd);
    }
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined ARDUINO_MOTOR_SHIELD_R3
  /* Wrap the motor driver initialization */
  void initMotorController() {
    /* If the brake function is enabled */
    #ifdef USE_ARDUINO_MOTOR_SHIELD_R3_BRAKE
      pinMode(LEFT_MOTOR_PIN_BRAKE, OUTPUT);   // Initiates Brake Channel A pin
      pinMode(RIGHT_MOTOR_PIN_BRAKE, OUTPUT);  // Initiates Brake Channel B pin
    #endif
    
    // Set up Channel A - LEFT
    pinMode(LEFT_MOTOR_PIN_DIR, OUTPUT);   // Initiates Motor Channel A pin

    // Set up Channel B - RIGHT
    pinMode(RIGHT_MOTOR_PIN_DIR, OUTPUT); // Initiates Motor Channel B pin
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) {
      if (spd < 0) digitalWrite(LEFT_MOTOR_PIN_DIR, LOW);
      else digitalWrite(LEFT_MOTOR_PIN_DIR, HIGH);

      #ifdef USE_ARDUINO_MOTOR_SHIELD_R3_BRAKE
        if (spd == 0) digitalWrite(LEFT_MOTOR_PIN_BRAKE, HIGH);
        else digitalWrite(LEFT_MOTOR_PIN_BRAKE, LOW);
      #endif

      analogWrite(LEFT_MOTOR_PIN_SPEED, spd);
    }
    else {
      if (spd < 0) digitalWrite(RIGHT_MOTOR_PIN_DIR, LOW);
      else digitalWrite(RIGHT_MOTOR_PIN_DIR, HIGH);

      #ifdef USE_ARDUINO_MOTOR_SHIELD_R3_BRAKE
        if (spd == 0) digitalWrite(RIGHT_MOTOR_PIN_BRAKE, HIGH);
        else digitalWrite(RIGHT_MOTOR_PIN_BRAKE, LOW);
      #endif
      
      analogWrite(RIGHT_MOTOR_PIN_SPEED, spd);
    }
  }

  /* A convenience function for setting both motor speeds */
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

/* For testing only! */
#elif defined(NO_MOTOR_CONTROLLER)
  /* Wrap the motor driver initialization */
  void initMotorController() { }
  
  /* A convenience function for setting both motor speeds */
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) {}
    else {}
  }
#elif defined TB6612_MOTOR_DRIVER
  void initMotorController(){
    //Control the direction of motor A, (AIN1, AIN2)=(1, 0) is forward rotation, (AIN1, AIN2)=(0, 1) is reversed
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    // CONTROL THE DIRECTION OF MOTOR B, (BIN1, BIN2)=(0, 1) IS FORWARD ROTATION, (BIN1, BIN2)=(1, 0) IS REVERSED
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);//A motor PWM
    pinMode(PWMB, OUTPUT);//B motor PWM
    pinMode(STBY, OUTPUT);//TB6612FNG enabled, set 0 to stop all motors, and set 1 to allow control of motors
    
    //Initialize the TB6612 motor drive module
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    digitalWrite(STBY, 1);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

  }

  void SetMotorPWM(int motor, int pwm, int dir)
  {
    if(motor==1 && dir==0)
    {
      digitalWrite(AIN1, 1);
      digitalWrite(AIN2, 0);
      analogWrite(PWMA, pwm);
    }
    else if(motor==1 && dir==1)
    {
      digitalWrite(AIN1, 0);
      digitalWrite(AIN2, 1);
      analogWrite(PWMA, pwm);
    }
    else if(motor==2 && dir==0)
    {
      digitalWrite(BIN1, 0);
      digitalWrite(BIN2, 1);
      analogWrite(PWMB, pwm);
    }
    else if(motor==2 && dir==1)
    {
      digitalWrite(BIN1, 1);
      digitalWrite(BIN2, 0);
      analogWrite(PWMB, pwm);
    }
  }

  void setMotorSpeed(int i, int spd){
    unsigned char reverse = 0;

    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;

    if (i == LEFT) { 
      SetMotorPWM(1, spd, reverse);
    } else /*if (i == RIGHT) //no need for condition*/ {
      SetMotorPWM(2, spd, 1-reverse);
    }
  }
  void setMotorSpeeds(int leftSpeed, int rightSpeed){
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  //#error A motor driver must be selected!
#endif

#endif

