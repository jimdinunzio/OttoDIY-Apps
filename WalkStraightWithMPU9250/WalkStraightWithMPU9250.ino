/* OttoDIY Walking Straight and Returning
 *  Copyright (C) 2019 by Jim DiNunzio MIT License
 *  date: 10/5/2019
 *  
 *  This module makes use of a MPU9250 board installed in a OttoDIY. It uses the magnetometer to get the robot's heading and
 *  stick to it walking for a certain number of paces and then turning around and coming back (roughly) to the point it started
 *  
 *  Tweaking of of the numbers will be required to make this work on any individual OttoDIY. 
 */

/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS false         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

//#define LOWER_BOUND_CYCLES_BEFORE_TURN 18
//#define UPPER_BOUND_CYCLES_BEFORE_TURN 23
//#define CYCLES_END_OF_RUN 42

#define LOWER_BOUND_CYCLES_BEFORE_TURN 36
#define UPPER_BOUND_CYCLES_BEFORE_TURN 41
#define CYCLES_END_OF_RUN 78

#define RIGHT_STEP_DEGREES 25
#define LEFT_STEP_DEGREES 20
#define PAUSE_FOR_DIR_MS 100

// Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

volatile bool newData = false;
bool newMagData = false;

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

//----------------------------------------------------------------
//-- Otto basic firmware v2 adapted from Zowi (ottodiy.com)
//-- CC BY SA
//-- 04 December 2016
//-----------------------------------------------------------------
//-- Otto will avoid obstacles with this code!
//-----------------------------------------------------------------
#include <Servo.h> 
#include <Oscillator.h>
#include <US.h>
#include <Otto.h>
Otto Otto;  //This is Otto!
//---------------------------------------------------------
//-- First step: Make sure the pins for servos are in the right position
/*
         --------------- 
        |     O   O     |
        |---------------|
YR 3==> |               | <== YL 2
         --------------- 
            ||     ||
RR 5==>   -----   ------  <== RL 4
         |-----   ------|
*/
  #define PIN_YL 6 //servo[2]
  #define PIN_YR 3 //servo[3]
  #define PIN_RL 4 //servo[4]
  #define PIN_RR 5 //servo[5]
///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
//-- Movement parameters
int T=1000;              //Initial duration of movement
int moveId=0;            //Number of movement
int moveSize=15;         //Asociated with the height of some movements
//---------------------------------------------------------
bool obstacleDetected = false;


void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  while(!Serial){};

  //Set the servo pins
  Otto.init(PIN_YL,PIN_YR,PIN_RL,PIN_RR,true);
  //Otto.setYieldCb(readMyIMU);
  Otto.sing(S_connection);
  Otto.home();

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x73, HEX);
  
  if (c == 0x73) // WHO_AM_I should always be 0x73
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }
    
    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    Otto.sing(S_confused); //Otto wake up!
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    
    // For my OTTO installation JCD 8/9/2019
//    myIMU.magBias[0] = 813;
//    myIMU.magBias[1] = -81;
//    myIMU.magBias[2] = 2015;
//    myIMU.magScale[0] = 0.92;
//    myIMU.magScale[1] = 0.82;
//    myIMU.magScale[2] = 1.44;

    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    //delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  // ready to start
  Otto.sing(S_happy); // a happy Otto :)
  delay(3000);
  Otto.sing(S_mode1);
}

#define START_SAMPLING_COUNT 50
#define TARGET_HEADING_SAMPLE_COUNT 50
int targetHeadingCount = TARGET_HEADING_SAMPLE_COUNT;
int initialCount = 0;

float targetHeading;

void updateMyIMU()
{
  newData = true;
  Serial.println("interrupt");
}

void updateAHRS()
{
  // AHRS
  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;

  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  myIMU.yaw  -= 11.75; // my house is 33.65 N x 117.87 W
  myIMU.roll *= RAD_TO_DEG;
}

void readMyIMU()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
//  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
//                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
//                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
//    myIMU.delt_t = millis() - myIMU.count;
//    if (myIMU.delt_t > 500)
//    {
//      if(SerialDebug)
//      {
//        // Print acceleration values in milligs!
//        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
//        Serial.print(" mg ");
//        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
//        Serial.print(" mg ");
//        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
//        Serial.println(" mg ");
//  
//        // Print gyro values in degree/sec
//        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
//        Serial.println(" degrees/sec");
//  
//        // Print mag values in degree/sec
//        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
//        Serial.print(" mG ");
//        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
//        Serial.print(" mG ");
//        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
//        Serial.println(" mG");
//  
//        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
//        // Temperature in degrees Centigrade
//        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
//        // Print temperature in degrees Centigrade
//        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
//        Serial.println(" degrees C");
//      }
//
//      myIMU.count = millis();
//      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
//    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  { 
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
   
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        updateAHRS();
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }
  
      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print("Yaw: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);
  
        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

float getHeading()
{
  return 180.0*atan2(myIMU.my, myIMU.mx)/PI;
}

enum Direction {
  D_FWD = 0,
  D_RGT = 1,
  D_LFT = 2
};

void halt()
{
  while (1)
  {
  }
}

int cycleCount = 0;
uint32_t lastPrintTime = 0;
uint32_t lastPauseForDir = 0;
int direction;

void loop()
{
  readMyIMU();
  
  float heading = getHeading();
  if (targetHeadingCount > 0){
    if (initialCount > START_SAMPLING_COUNT) {
      targetHeading += heading;
      targetHeadingCount--;
      if (targetHeadingCount == 0)
        targetHeading /= TARGET_HEADING_SAMPLE_COUNT; 
      else { 
        delay(25);
        return;
      }
    }
    else
    {
      initialCount++;
      delay(25);
      return;
    }
  }

  bool printLog = false;
  
  if (millis() - lastPauseForDir < PAUSE_FOR_DIR_MS)
    return;

  bool countSteps = true;
  // If at end of half lap,turn around and go opposite way
  if (cycleCount >= LOWER_BOUND_CYCLES_BEFORE_TURN && cycleCount <= UPPER_BOUND_CYCLES_BEFORE_TURN) 
  {
    cycleCount = UPPER_BOUND_CYCLES_BEFORE_TURN + 1;
    if (targetHeading < 0)
      targetHeading += 180;
    else 
      targetHeading -= 180;
    countSteps = false;
    if (printLog) 
    {
      Serial.print("Turning around");
    }
    Otto.sing(S_happy);
  } 
  else if (abs(targetHeading - heading) < 10 && cycleCount >= CYCLES_END_OF_RUN)
  {
    if (printLog)
      Serial.println("End of run");
    Otto.home();
    Otto.sing(S_superHappy);
    delay(2000);
    Otto.sing(S_disconnection);
    halt();
  }

  // compute delta angle (to turn) between heading and target heading properly handling angle discontinuity at -180/180 degrees.
  int result = int(targetHeading - heading + 360.0) % 360;
  if (result > 180.0)
    result -= 360.0;
   
  if (printLog)
  {
    Serial.print("Heading = ");
    Serial.print(heading, 2);
    Serial.print(" Target heading = ");
    Serial.print(targetHeading,2);
    Serial.print(" Delta angle = ");
    Serial.println(result);
    lastPrintTime = millis();
  }

  if (abs(result) <= 10) // if only 10 degrees off of target heading, go straight.
  {
    if (printLog)
      Serial.println("Going Forward");
    Otto.walk(3,1000,FORWARD);
    if (direction != D_FWD)
      Otto._tone(note_G5,100,0);
    direction = D_FWD;
    if (countSteps)
      cycleCount += 3;
  }
  else if (result < -10) // if more than 10 degrees to left of heading, go right
  {
    if (printLog)
      Serial.println("Going Right");
    if (direction != D_RGT)
      Otto._tone(note_E6,100,0); //D6
    direction = D_RGT;
    int rightSteps = max(1, (-result + RIGHT_STEP_DEGREES / 2) / RIGHT_STEP_DEGREES);
    Otto.turn(rightSteps,1000,RIGHT);
    if (countSteps)
      cycleCount += rightSteps;
  }
  else // result > 10 // if more than 10 degrees to right of heading, go left.
  {
    if (printLog)
      Serial.println("Going Left");
    if (direction != D_LFT)
      Otto._tone(note_D7,100,0);  //G6
    direction = D_LFT;
    int leftSteps = 1 + max(1, (result + LEFT_STEP_DEGREES / 2) / LEFT_STEP_DEGREES);    
    Otto.turn(leftSteps,1000,LEFT);
    if (countSteps)
      cycleCount += leftSteps;
  }

  Otto.home();
  lastPauseForDir = millis();
}
