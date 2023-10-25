#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuMenu.h>

Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];
int s1, s2, s3;

int speed =100;
float wheelCirc = 12.5; // change to the circumference that you measured for your robot
long randNumber;
float targetAngle = 90.0;     // Ønsket drejevinkel i grader

uint32_t turnAngle = 0;

int16_t turnRate;

int16_t gyroOffset;

uint16_t gyroLastUpdate = 0;

void setup() {
  // put your setup code here, to run once:
Wire.begin();
Serial.begin(9600);
turnSensorSetup();
delay(500);
turnSensorReset();
display.clear();
randomSeed(analogRead(0));

}

int32_t getTurnAngleInDegrees(){

}

void turnSensorSetup(){
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  ledYellow(1);

  delay(500);

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
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    display.gotoXY(0, 0);
  // do some math and pointer magic to turn angle in seconds to angle in degrees
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));
  }
  display.clear();
}

void turnSensorReset(){
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate(){
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  int32_t d = (int32_t)turnRate * dt;

  turnAngle += (int64_t)d * 14680064 / 17578125;
}


void forward(){
  motors.setSpeeds(speed, speed);
}


float getDistance(){

  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  
  float distanceL = countsL /900.0 * wheelCirc;
  float distanceR = countsR /900.0 * wheelCirc;
  return (distanceL + distanceR)/2; 
}



void loop() {
  
  int32_t turnDegrees = getTurnAngleInDegrees();
  Serial.println("Degree: " + (String)turnDegrees);

  float distance = getDistance();

  display.clear();
  display.gotoXY(0, 0);
  display.print("Distance:");
  display.gotoXY(0, 1);
  display.print(distance); // Erstat 'distance' med den faktiske afstand

imu.read();


  if (distance<20){
    forward();
  }
  else{
    while (turnAngle<90){
      motors.setSpeeds(-speed,speed);
      delay(100);
      turnSensorUpdate();
    }

    motors.setSpeeds(0,0);
    delay(200);
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
 
    }
    
  }
 


  






 
 




