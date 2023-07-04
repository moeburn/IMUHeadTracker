#include <Joystick.h> //include USB input libraries
#include <Mouse.h>
#include <Keyboard.h>
#include <HID.h>

#include <Wire.h> //include basic Arduino interfacing libraries
#include "I2Cdev.h"
#include <EEPROM.h>

#include "RTIMUSettings.h" //include IMU libraries
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"


RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
RTMath mathobj;

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  20                         // interval between pose displays in milliseconds

//  SERIAL_PORT_SPEED defines the baud speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

#define JOYSTICK 1


unsigned long lastDisplay;  //declare a bunch of things for stuff
unsigned long zeroTime = 2000;
bool zeroed = false;
unsigned long lastRate;
int sampleCount;

//JOYSTICK STUFF


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,   //declare the USB joystick with settings that only enable two X,Y axes with 10 bit precision each
  JOYSTICK_TYPE_MULTI_AXIS, 32, 0,
  true, true, false, false, false, false,
  true, true, false, false, false); 

// Set to true to test "Auto Send" mode or false to test "Manual Send" mode.
const bool testAutoSendMode = false;

const unsigned long gcCycleDelta = 1000; //declare more things that I forget why
const unsigned long gcAnalogDelta = 25;
const unsigned long gcButtonDelta = 500;
unsigned long gNextTime = 0;
unsigned int gCurrentStep = 0;
int xAxis;
int yAxis;
int xoffset;
int yoffset;
float pitch1;
float yaw1;

void setup() //Do this on device powerup
{
    int errcode;
    if (!JOYSTICK) {Serial.begin(SERIAL_PORT_SPEED);}  //if joystick isn't present yet, start the serial port
    Wire.begin(); //start the wire interfacing
    imu = RTIMU::createIMU(&settings);                        // create the imu object
    //all serial port debugging code has been commented out as it is no longer needed:
    //Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName()); 
    if ((errcode = imu->IMUInit()) < 0) {
        //Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    //if (imu->getCalibrationValid())
        //Serial.println("Using compass calibration");
    //else
        //Serial.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();  //set the two timekeeping variables to equal the current time
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.  0.02 default 
    
    fusion.setSlerpPower(0.005); //0.005 is perfect jan072022-moeburn
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);

    //JOYSTICK STUFF
  if (JOYSTICK) {
        Joystick.setXAxisRange(-32766, 32766); //set X and Y axes to maximum 10 bit resolution
        Joystick.setYAxisRange(-32766, 32766);
        Joystick.setZAxisRange(-127, 127); //we're not really ussing these 3 other axes but I'm not touching that code
        Joystick.setThrottleRange(0, 245);
        Joystick.setRudderRange(0, 255);
        
        if (testAutoSendMode)
        {
          Joystick.begin();
        }
        else
        {
          Joystick.begin(false);
        }

  }
}

void loop() //Do this stuff while the device is powered on, after booting up
{  
    unsigned long now = millis();
    unsigned long delta;
    int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp()); //do the thing that gets the stuff
        sampleCount++; //increase the variable that tracks the number of samples we have taken
        //all serial port debugging code has been commented out:
        /*if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.println(", gyro bias valid");
            else
                Serial.println(", calculating gyro bias");
        
            sampleCount = 0;
            lastRate = now;
        }*/
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {  //Do this every DISPLAY_INTERVAL (20ms)
            lastDisplay = now; //reset timer
            //this is debugging code, commented out:
//          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
            if (!JOYSTICK) {
            RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fuse the output
            Serial.println();
            }
        }
    }


    if (JOYSTICK) { //If Joystick is working properly, convert IMU radians output to Joystick-compatible integers
          pitch1 = ((RTVector3&)fusion.getFusionPose()).y(); //pitch1 is IMU library output in radians
          pitch1 *= RTMATH_RAD_TO_DEGREE; //convert that to degrees
          pitch1 += 90; //do a bunch of math to convert 360 values into 65532 values
          pitch1 = pitch1 / 180;
          pitch1 *= 65532;
          pitch1 -= 32766;
          yAxis = pitch1 - yoffset; //subtract the zeroing offset
      
          yaw1 = ((RTVector3&)fusion.getFusionPose()).z(); //do all that again but for the other axis
          yaw1 *= RTMATH_RAD_TO_DEGREE;
          yaw1 += 90;
          yaw1 = yaw1 / 180;
          yaw1 *= 65532;
          yaw1 -= 32766;
          xAxis = yaw1 - xoffset;

          if (!zeroed && (millis() > 2000)) { //if device has been powered on for more than 2 seconds and we haven't zeroed yet
          xoffset = xAxis; //set the offset values to the current x,y values
          yoffset = yAxis;
          zeroed = true;  //we've zeroed now, don't do this again
          }
      
          Joystick.setXAxis(xAxis); //tell the USB library code to actually send that data we just calculated
          Joystick.setYAxis(yAxis);
          
          if (testAutoSendMode == false) {
            Joystick.sendState();
          }
    }

}

//that's all folks
