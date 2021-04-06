
// HOLD DOWN CODE
// Author:  Jonathan Palafoutas
// Date:    2020-09-10

// ===========================================================
// COLOR KEY:
// yellow           - testing microSD card
// yellow + red     - microSD card failed or is not present
// yellow + red(1)  - failure to open file before renaming
// yellow + red(2)  - failure to rename file
// yellow + red(3)  - faillure to open file before writing header
// yellow + green   - microSD card initialized successfully!

// blue             - testing MPU6050
// blue + red       - MPU6050 failed or is not present
// blue + green     - MPU6050 initialized successfully!

// blue + yellow    - calibrating gyroscope

// green            - system is currently recording
// ===========================================================





// ===========================================================
// include libraries

#include "I2Cdev.h"
#include "math.h"
#include "MPU6050.h"
#include "SdFat.h"
#include "Servo.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ===========================================================
// declare global variables

// calibration
int cal_sam = 5000; // number of samples for calibration 
float gx_cal = 0; // calibtration sum terms
float gy_cal = 0;
float gz_cal = 0;
float gxcs, gycs, gzcs; //cs - calibrated and scaled (rad/s)

// IMU calculations
int16_t ax, ay, az, gx, gy, gz;
float pitch, roll, yaw;
float pitch_prev = 0.0;
float roll_prev = 0.0;
float yaw_prev = 0.0;
float pitchD, yawD;
float pitchI = 0.0, yawI = 0.0;

// PID
//float P_pitch = 5.0;
//float I_pitch = 0.0;
//float D_pitch = 0.0;
//float P_yaw = -5.0;
//float I_yaw = -0.0;
//float D_yaw = -0.0;

float P_pitch = 0.01930;
float I_pitch = 0.00113;
float D_pitch = 0.07350;
float P_yaw = -0.00942;
float I_yaw = -0.00055;
float D_yaw = -0.03588;

// pins
const int CS = 10; // chip select pin for microSD card
const int LED_g = 8; // LED pins
const int LED_b = 7;  
const int LED_r = 6;
const int LED_y = 5;
const int servox_pin = 2; // servo pins
const int servoy_pin = 4;

// printing (microSD card)
char data[185]; // for printing data
char filename[] = "data.txt"; // do not use numbers
char seconame[8]; // for renaming files
char header[185];
char header1[] = "time (us)";
char header2[] = "dtime (s)";
char header3[] = "ax";
char header4[] = "ay";
char header5[] = "az";
char header6[] = "gxcs (deg/s)";
char header7[] = "gycs (deg/s)";
char header8[] = "gzcs (deg/s)";
char header9[] = "roll (deg)";
char header10[] = "pitch (deg)";
char header11[] = "yaw (deg)";
char header12[] = "servoy (deg)";
char header13[] = "servox (deg)";
int i; // for file number
int count = 1; // for recording every nth sample
int n = 10;

// servos
float alpha; // motor direction on the pitch-yaw plane
float Beta_max = 15.0; // maxaccelgyrom gimbal angle (deg)
float posx = 0.0; // servo position
float posy = 0.0;
float posxi = 97.0; // initial servo position (deg)
float posyi = 85.0;

// time
float delta_t;  // current - previous (s)
int delayMS = 500; // standard delay (ms)
unsigned long current; // current time value (us)
unsigned long previous = 0; // previous time value (us)
unsigned long RecordingTime; // time at writing to SD card (us)

// unit conversion
float DEG2RAD = 0.017453292519943;
float RAD2DEG = 57.295779513082;

// quaternions
float qw=1.0, qx=0.0, qy=0.0, qz=0.0; // previous quaternions
float qwn=1.0, qxn=0.0, qyn=0.0, qzn=0.0; // current quaternion
float qwn_dot, qxn_dot, qyn_dot, qzn_dot; // current quaternion derivative

// ===========================================================
// initialize objects (or whatever they are called)
MPU6050 accelgyro;
Servo servox; // yaw servo
Servo servoy; // pitch servo
SdFat sd;
SdFile myFile;

void setup() {
  // ===========================================================
  // begin serial activity
  //Serial.begin(38400);
  
  // ===========================================================
  // initialize servos
  servox.attach(servox_pin);
  servoy.attach(servoy_pin);
  
  servox.write(posx + posxi);
  servoy.write(posy + posyi);
  
  // initialize LEDs
  pinMode(LED_g, OUTPUT);
  pinMode(LED_b, OUTPUT);
  pinMode(LED_r, OUTPUT);
  pinMode(LED_y, OUTPUT);

  // ===========================================================
  // connect to microSD card and indicate if this was successful
  delay(delayMS);
  digitalWrite(LED_y, HIGH);
  delay(delayMS);

  // create header character array before opening file
  sprintf(header, "%15s %15s %10s %10s %10s %15s %15s %15s %15s"
  "%15s %15s %15s %15s", header1, header2, header3, header4,
  header5, header6, header7, header8, header9, header10, header11,
  header12, header13);

  // see if the card is present and can be initialized:
  if (!sd.begin(CS, SPI_FULL_SPEED)) {
    digitalWrite(LED_r, HIGH);
    while (true); // do nothing
  }

  // card initialization did not fail (yet)!

  // determine if DATA.TXT already exists and rename it if it does
  if (sd.exists(filename))
  {
    for (i = 1; i < 1000; i++)
    {
      sprintf(seconame, "DATA%i.TXT", i);
      if (!sd.exists(seconame)) 
      {
        break;
      }
    }

    // open file that already exists
    if (!myFile.open(filename, O_WRITE | O_CREAT)) {
      // ERROR, BLINK RED ONCE AND HOLD
      digitalWrite(LED_r, HIGH);
      delay(delayMS);
      digitalWrite(LED_r, LOW);
      delay(delayMS);
      digitalWrite(LED_r, HIGH);
      while (true);
    }

    // rename the file with a number
    if (!myFile.rename(sd.vwd(), seconame))
    {
      // ERROR, BLINK RED TWICE AND HOLD
      digitalWrite(LED_r, HIGH);
      delay(delayMS);
      digitalWrite(LED_r, LOW);
      delay(delayMS);
      digitalWrite(LED_r, HIGH);
      delay(delayMS);
      digitalWrite(LED_r, LOW);
      delay(delayMS);
      digitalWrite(LED_r, HIGH);
      while (true);
    }
    
    myFile.close();
  }
  
  // write header to microSD card
  if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END))
  {
    // ERROR, BLINK RED THREE TIMES AND HOLD
    // at the header writing stage
    digitalWrite(LED_r, HIGH);
    delay(delayMS);
    digitalWrite(LED_r, LOW);
    delay(delayMS);
    digitalWrite(LED_r, HIGH);
    delay(delayMS);
    digitalWrite(LED_r, LOW);
    delay(delayMS);
    digitalWrite(LED_r, HIGH);
    delay(delayMS);
    digitalWrite(LED_r, LOW);
    delay(delayMS);
    digitalWrite(LED_r, HIGH);
    while (true); // do nothing
  }

  myFile.println(header);
  myFile.close();

  // done initializing microSD card!
  digitalWrite(LED_g, HIGH);
  delay(delayMS);
  digitalWrite(LED_y, LOW);
  digitalWrite(LED_g, LOW);
  delay(delayMS);

  // ===========================================================
  // connect to MPU6050
  digitalWrite(LED_b, HIGH);
  delay(delayMS);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();

  // indicate if this was successful
  if (accelgyro.testConnection())
  {
    // MPU6050 is online
    digitalWrite(LED_g, HIGH);
    delay(delayMS);
    // turn off
    digitalWrite(LED_b, LOW);
    digitalWrite(LED_g, LOW);
    delay(delayMS);
  }
  else
  {
    // failed to initalize MPU6050
    digitalWrite(LED_r, HIGH);
    while (true); // do nothing
  }

  // ===========================================================
  // calibrate MPU6050 gyroscope
  delay(5000);
  digitalWrite(LED_b, HIGH);
  digitalWrite(LED_y, HIGH);
  for (int cal_int = 0; cal_int < cal_sam ; cal_int ++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gx_cal += gx;
    gy_cal += gy;
    gz_cal += gz;

    delay(3);
  }

  gx_cal /= cal_sam;                                                       //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gy_cal /= cal_sam;                                                       //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gz_cal /= cal_sam;

  // finished calibrating gyroscope
  digitalWrite(LED_b, LOW);
  digitalWrite(LED_y, LOW);

  // turn on green LED to indicate that system is on and recording
  digitalWrite(LED_g, HIGH);

  // don't forget to convert Beta_max to radians!
  Beta_max = Beta_max*DEG2RAD;
}

void loop() {
  // ===========================================================
  // accelgyro calculations
  
  // time step calculations
  current = micros();
  delta_t = (current - previous)*0.000001;
  previous = current;

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


  gxcs = DEG2RAD*(gx - gx_cal)/131;
  gycs = DEG2RAD*(gy - gy_cal)/131;
  gzcs = DEG2RAD*(gz - gz_cal)/131;

  // calculate quaternion derivative
  qwn_dot = 0.5*((-gxcs*qx) + (-gycs*qy) + (-gzcs*qz));
  qxn_dot = 0.5*( (gxcs*qw) +  (gzcs*qy) + (-gycs*qz));
  qyn_dot = 0.5*( (gycs*qw) + (-gzcs*qx) +  (gxcs*qz));
  qzn_dot = 0.5*( (gzcs*qw) +  (gycs*qx) + (-gxcs*qy));

  // integrate quaternion derivative
  qwn += qwn_dot*delta_t;
  qxn += qxn_dot*delta_t;
  qyn += qyn_dot*delta_t;
  qzn += qzn_dot*delta_t;

  // set 'previous' quaternion to be 'next' quaternion
  qw = qwn;
  qx = qxn;
  qy = qyn;
  qz = qzn;
  
  // convert to pitch roll yaw wrt inertial frame
  pitch = atan2((2*qxn*qwn) - (2*qyn*qzn), 1 - (2*qxn*qxn) - (2*qzn*qzn));
  roll = atan2((2*qyn*qwn) - (2*qxn*qzn), 1 - (2*qyn*qyn) - (2*qzn*qzn));
  yaw = asin(2*qxn*qyn + 2*qzn*qwn);

  // ===========================================================
  // PID term calculations
  
  pitchD = (pitch - pitch_prev)/delta_t;
  yawD = (yaw - yaw_prev)/delta_t;
  
  pitchI += (pitch - pitch_prev)*delta_t;
  yawI += (yaw - yaw_prev)*delta_t;

  pitch_prev = pitch;
  yaw_prev = yaw;

  // ===========================================================
  // servomotor instructions
  
  posx = (P_yaw*yaw) + (I_yaw*yawI) + (D_yaw*yawD);
  posy = (P_pitch*pitch) + (I_pitch*pitchI) + (D_pitch*pitchD);

  if (sq(sin(Beta_max)) < (sq(sin(posx)) + (sq(sin(posy))*sq(cos(posx)))))
  {
    alpha = atan2(sin(posy)*cos(posx), sin(posx));

    posx = asin(sin(Beta_max)*cos(alpha));
    posy = asin(sin(Beta_max)*sin(alpha)/cos(posx));
  }

  posy = posy*RAD2DEG;
  posx = posx*RAD2DEG;

  servox.write(posxi + posx);
  servoy.write(posyi + posy);

  // ===========================================================
  // write to microSD card

  gxcs = gxcs*RAD2DEG;
  gycs = gycs*RAD2DEG;
  gzcs = gzcs*RAD2DEG;
  roll = roll*RAD2DEG;
  pitch = pitch*RAD2DEG;
  yaw = yaw*RAD2DEG;
    
//  sprintf(data, "%10lu %15d %15d %15d %15f %15f %15f %15f %15f"
//  "%15f %15f %15f", current, ax, ay, az, gxcs, gycs, gzcs,
//  roll, pitch, yaw, posy, posx);

  RecordingTime = micros();
  
  sprintf(data, "%15lu %15.8f %10d %10d %10d %15f %15f %15f %15f %15f"
  "%15f %15f %15f", RecordingTime, delta_t, ax, ay, az, gxcs, gycs, gzcs,
  roll, pitch, yaw, posy, posx);

  //Serial.println(data);

  if (count == n)
  {
    count = 0;
    
    if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END))
    {
      digitalWrite(LED_g, LOW);
      digitalWrite(LED_r, HIGH);
      while (true); // do nothing
    }

    myFile.println(data);
    myFile.close();
  }
  else
  {
    count += 1;
  }

}
