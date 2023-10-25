#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

// vars to count the accumulated counts of the encoders
int movementCommand = 0;   // used to select the command
int movementParameter = 0;   // used to select distance or angle

int const countJump = 100;  // number of counts before jumping to next command in stage 0
int const maxDist = 100;   // max distance the robot should travel
int const maxAngle = 180; // max angle the robot can turn
int speed = 100; // max speed of the robot
float wheelCirc = 12.9; // circumference of the wheel (incl. the belt) to calculate the distance

// control the flow of the program. 0 input for command
//                                  1 input for movement parameter (distance or angle)
//                                  2 running the command.
int stage = 0;      

// control selection 1              0 Forward
//                                  1 Backward
//                                  2 turn
int chosenCommand = 0;

// variables for gyro
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

void setup() {
  Serial.begin(9600);
  turnSensorSetup();
  delay(100);
}

void loop() {
  delay(50);
  switch(stage){
    case 0: selectMovement();
    break;
    case 1: selectParameter();
    break;
    case 2: countdown();
    executeMovement();
    break;
  }
}

void selectMovement(){
  readEncodersMovement();
  if (movementCommand > countJump){
    bip();
    movementCommand = 0;
    chosenCommand++;
    if (chosenCommand>2) chosenCommand = 0; 
  }
  else if (movementCommand < -countJump){
    bop();
    movementCommand = 0;
    chosenCommand--;
    if (chosenCommand<0) chosenCommand = 2; 
  }
  LCDSelectMovement();

  if (buttonA.isPressed()){
    bip();
    stage = 1;
    buttonA.waitForButton();
  }
}

void LCDSelectMovement(){
  switch(chosenCommand){
    case 0:
      display.clear();
      display.print("Command>");
      display.gotoXY(0, 1);
      display.print("Forward");
      break;
    case 1:
      display.clear();
      display.print("Command>");
      display.gotoXY(0, 1);
      display.print("Backward");
      break;
    case 2:
      display.clear();
      display.print("Command>");
      display.gotoXY(0, 1);
      display.print("Turn");
      break;
  }
}

void selectParameter(){
  display.clear();
  readEncodersParameter();
  switch(chosenCommand){
    case 0: case 1: 
    if(movementParameter < 0){
      movementParameter = 0;
      bip();
    }
    else if(movementParameter > maxDist){
      movementParameter = maxDist;
      bop();
    }
    break;
    case 2: 
    if(movementParameter < -180){
      movementParameter = -180;
      bip();
    }
    else if(movementParameter > maxAngle){
      movementParameter = maxAngle;
      bop();
    }
    break;
  }
  LCDSelectParameter();
  if(buttonA.isPressed()){
    bip();
    stage = 2;
    buttonA.waitForRelease(); 
  }

  
}

void LCDSelectParameter(){
  if (chosenCommand == 2){
    display.clear();
    display.print("T Angle:");
    display.gotoXY(0, 1);
    display.print((String)movementParameter);
  }
  else {
    display.clear();
    display.print("Distance:");
    display.gotoXY(0, 1);
    display.print((String)movementParameter);
  }
}    

void executeMovement(){
  display.clear();
  switch(chosenCommand){
    case 0: // forward
      while (getDistance() < movementParameter){
        forward();
        display.clear();
        display.print("go");
        delay(100);
      }
      stop();
      break;
    case 1: // backward
      while (getDistance() > (-1) *movementParameter){
        backward();
        display.clear();
        display.print("go");
        delay(100);
      }
      stop();
      break;
    case 2: // turn
      turnByAngle();
      display.clear();
      delay(100);
      stop();
      break;
  }
  stop();
  // Reset of the values
  movementCommand = 0;
  movementParameter = 0;
  stage = 0;
  chosenCommand = 0;

}

void countdown(){
  display.clear();
  display.print("Count...3");
  display.gotoXY(0,1);
  display.print("Down...3");
  buzzer.playNote(NOTE_A(4),800,5);
  delay(1600);
  display.clear();
  display.print("Count..2");
  display.gotoXY(0,1);
  display.print("Down..2");
  buzzer.playNote(NOTE_A(4),800,5);
  delay(1600);
  display.clear();
  display.print("Count.1");
  display.gotoXY(0,1);
  display.print("Down.1");
  buzzer.playNote(NOTE_B(4),400,5);
  delay(800);
  display.clear();
  display.print("STEP");
  display.gotoXY(0,1);
  display.print("!ASIDE!");
  buzzer.playNote(NOTE_G(5),800,5);
  delay(1600);
  display.clear();
}

/*
Movement functions
*/
void stop(){
  motors.setSpeeds(0,0);
}

void forward(){
  motors.setSpeeds(speed, speed);
}

void backward(){
  motors.setSpeeds(-speed, -speed);
}

void turnByAngle(){
  turnSensorReset();
  Serial.println(getTurnAngleInDegrees());
  if (movementParameter>0){
     Serial.println(getTurnAngleInDegrees());
    motors.setSpeeds(-100,100);
     Serial.println(getTurnAngleInDegrees());
    while(getTurnAngleInDegrees() != movementParameter){
       Serial.println(getTurnAngleInDegrees());
      delay(10);
    }
  }
  else{
    Serial.println(getTurnAngleInDegrees());
    motors.setSpeeds(100,-100);
    Serial.println(getTurnAngleInDegrees());
    while(getTurnAngleInDegrees()!=(360+movementParameter)){
      Serial.println(getTurnAngleInDegrees());
      delay(10);
      Serial.println(getTurnAngleInDegrees());
    }
  }
}

/*
Encoder functions
*/

void resetEncoders(){
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void readEncodersMovement(){
//  movementCommand = TODO;
   movementCommand = movementCommand + encoders.getCountsRight();
  resetEncoders();
}

void readEncodersParameter(){
//  movementParameter = TODO;
  movementParameter = movementParameter + encoders.getCountsLeft();
  resetEncoders();
}

float getDistance(){
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL/900.0 * wheelCirc;
  float distanceR = countsR/900.0 * wheelCirc;

  return (distanceL + distanceR)/2;
}

/*
Sound functions
*/

void bip(){
  buzzer.playFrequency(440, 200, 1);
  delay(100);
  
}
void bop(){
  buzzer.playFrequency(240, 200, 1);
  delay(100);
}

/* 
Gyro setup and convenience functions '
*/
void turnSensorSetup(){
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

uint32_t getTurnAngleInDegrees(){
  turnSensorUpdate();
  // do some math and pointer magic to turn angle in seconds to angle in degree
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}
