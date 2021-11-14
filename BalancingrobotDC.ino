/*
  Balancing robot with 6V DC motors
  L293D IC as motor driver (powered by 5V pin of arduino)
  MPU6050 module (powered by 5V pin of arduino)
  Arduino nano V3 (5V), powered via a 9V battery to Vin
  2x Lipo batteries in series
  step down buck converter (Lipo to DC motors). Note measure the voltage at the motors since the L293D had a large voltage drop
*/

#include <Wire.h>
//#include "AltSoftSerial.h"  // to be used later on for communication with bluetooth module HC-05

#define L_ENABLE 3  // enable pin of L293D IC for left motor, PWM pin
#define L_DIR1 2
#define L_DIR2 4
#define R_ENABLE 6  // enable pin of L293D IC for blue motor, PWM pin
#define R_DIR1 5
#define R_DIR2 7
#define GREENLED 13 // light for indication of leaning backward
#define BLUELED 12  // light for indication of leaning forward

#define FORWARDS 1
#define BACKWARDS 0
#define NEUTRAL 2

#define PRINTINTERVAL 1000 // interval for printing debug information when serial debug is on
uint32_t print_interval;

float X_correction = 1.07;  // the maximum value of the X acceleration of the used mpu6050 is 1.07, use this correction to get a maximum of 1
float Z_correction = 0.85;  // the maximum value of the Z acceleration of the used mpu6050 is 0.85, use this correction to get a maximum of 1

#define MEASUREINTERVAL 5  // interval for reading the MPU06050 in ms. Must be longer than the delay used in the low pass filter of the MPU6050
float elapsed_time;
uint32_t prev_measure_time_us, measure_time_us, elapsed_time_us;

float balance_angle = -13.2;  // the measured angle when the robot is in balance. This depends on the robot and the center of mass
float zero_deviation = 0.3;   // If the deviation is below this value, the robot is in balance
boolean in_control = false;   // the robot starts off balance

float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read
float Acc_X, Acc_Z;                    // corrected values
float accAngle;

const int GyroY_error = 6;        // When the MPU6050 is at rest, measure the raw values, the GyroY value is 6 at rest, so correct for this error
                                  //https://www.i2cdevlib.com/forums/topic/91-how-to-decide-gyro-and-accelerometer-offsett/
                                  //https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/

float angle_rad, angle_deg = 100;                 // the robot starts flat, angle is about 100 deg
float angle_dev, prev_angle_dev, angle_dev_sum;
#define MAX_DEVIATION_DEG 35                      // when the deviation is larger then this angle, the motors stop.
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read
float GyrAngle;

float P_value = 90;         // P of the PID control, reacts proportional to deviation.
float I_value = 0.001;      // I of the PID control, integral value, reacts to added deviation
float D_value = 250;        // D of the PID control, derrivative, reacts to difference between current and previous deviation

float speed, prev_speed;
int state, prev_state;

const int min_speed = 40;  // minimum speed (PWM) value at which the wheels start to rotate

#define SERIAL_DEBUG 0    // 0: no Serial communication. 1: Serial communication for debugging. Note: slows down the Arduino!!

void setup() {
  if (SERIAL_DEBUG)
  {
    Serial.begin(9600);
    Serial.println("setup start");
  }
  setpinmodes();
  initgyro();

  Serial.println("setup end");
  print_interval = millis();
  measure_time_us = micros(); // we measure microseconds
}

void loop() {
  prev_state = state;
  if (micros() > measure_time_us + MEASUREINTERVAL * 1000)  // the mpu6050 is read every measureinterval (5000us) and the speed is set
  {
    prev_measure_time_us = measure_time_us;
    measure_time_us = micros();
    elapsed_time_us = measure_time_us - prev_measure_time_us;
    elapsed_time = elapsed_time_us / 1000.0;

    //read MPU6050
    //long temp1 = micros();
    getAccXZGyrY();             // see function
    //long temp2 = micros();
    //int temp3 = temp2 - temp1;
    //Serial.print("Measurement takes: ");  // debug info to measure the time it takes to read the mpu6050
    //Serial.println(temp3);

    // calculate speed via PID
    // first formula I used
    //speed = abs(angle_dev) * P_value + (abs(angle_dev) - abs(prev_angle_dev)) * D_value;// + angle_dev_sum * I_value;
    // second formula I used
    //speed = abs(angle_dev) * P_value + (abs(angle_dev - prev_angle_dev)) * D_value; // + angle_dev_sum * I_value;
    // third and best formula I used, without I value
    //speed = 30+ abs(angle_dev * P_value + (angle_dev - prev_angle_dev) * D_value);// + angle_dev_sum * I_value;
    // current formula, with I value. Since the motors start to turn at about 40, add 30 to reach this speed faster
    speed = 30 + abs(angle_dev * P_value + (angle_dev - prev_angle_dev) * D_value + angle_dev_sum * I_value);
    // maximum value for PWM signal in analogWrite function
    if (speed > 255)
      speed = 255;
    if (speed < min_speed)
      speed = min_speed;
  }
  if (abs(balance_angle - angle_deg) > MAX_DEVIATION_DEG) // stop the motor if the robot tipped over.
  {
    in_control = false;
    digitalWrite(GREENLED, HIGH);
    digitalWrite(BLUELED, HIGH);
    angle_dev_sum = 0;
    setspeed(0);
  }
  else
  {
    in_control = true;
    digitalWrite(GREENLED, LOW);
    digitalWrite(BLUELED, LOW);
  }
  if (in_control)
  {
    if (abs(angle_dev) < zero_deviation) //robot is in balance when the deviation of the angle is small
    {
      speed = 0;
      setspeed(speed);
      angle_dev_sum = 0;
      state = NEUTRAL;
      digitalWrite(BLUELED, LOW);
      digitalWrite(GREENLED, LOW);
    }
    else // not in balance
    {
      if ((angle_deg < balance_angle))
      {
        state = FORWARDS;
        digitalWrite(BLUELED, HIGH);
        digitalWrite(GREENLED, LOW);
        leftdir(FORWARDS);
        rightdir(FORWARDS);
        setspeed(speed);
      }
      else // leaning backwards
      {
        state = BACKWARDS;
        digitalWrite(BLUELED, LOW);
        digitalWrite(GREENLED, HIGH);
        leftdir(BACKWARDS);
        rightdir(BACKWARDS);
        setspeed(speed);
      }
    }
  }
  if (SERIAL_DEBUG && millis() > print_interval + PRINTINTERVAL)  // print debug information when the Serial monitor is initialized
  {
    Serial.println("\nGyr_rawY\tGyrAngle: ");
    Serial.print(Gyr_rawY);
    Serial.print("\t\t");
    Serial.println(GyrAngle);
    Serial.print("\n");
    Serial.println("\t\t\tAcc_X\tAcc_Z\tangle_deg\tangle_dev\tangle_dev_sum");
    Serial.print("\t\t\t");
    Serial.print(Acc_X);
    Serial.print("\t");
    Serial.print(Acc_Z);
    Serial.print("\t");
    Serial.print(angle_deg);
    Serial.print("\t");
    Serial.print(angle_dev);
    Serial.print("\t");
    Serial.println(angle_dev_sum);
    //Serial.print("elapsed_time: ");
    //Serial.println(elapsed_time, 6);
    Serial.print("elapsed_time_us: ");
    Serial.println(elapsed_time_us);
    Serial.print("speed: ");
    Serial.println(speed);

    print_interval = millis();
  }
}

void leftdir(boolean direction)
{
  if (direction)
  {
    digitalWrite(L_DIR1, HIGH);
    digitalWrite(L_DIR2, LOW);
  }
  else
  {
    digitalWrite(L_DIR2, HIGH);
    digitalWrite(L_DIR1, LOW);
  }
}

void neutral(void)
{
  digitalWrite(L_DIR1, LOW);
  digitalWrite(L_DIR2, LOW);
  digitalWrite(R_DIR1, LOW);
  digitalWrite(R_DIR2, LOW);
}

void rightdir(boolean direction)
{
  if (direction)
  {
    digitalWrite(R_DIR1, HIGH);
    digitalWrite(R_DIR2, LOW);
  }
  else
  {
    digitalWrite(R_DIR2, HIGH);
    digitalWrite(R_DIR1, LOW);
  }
}

void setspeed(int speedvalue)
{
  analogWrite(L_ENABLE, speedvalue);
  analogWrite(R_ENABLE, speedvalue);
}

void setpinmodes(void)
{
  pinMode(L_ENABLE, OUTPUT);
  pinMode(R_ENABLE, OUTPUT);
  pinMode(L_DIR1, OUTPUT);
  pinMode(L_DIR2, OUTPUT);
  pinMode(R_DIR1, OUTPUT);
  pinMode(R_DIR2, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  digitalWrite(L_DIR1, LOW);
  digitalWrite(L_DIR2, LOW);
  digitalWrite(R_DIR1, LOW);
  digitalWrite(R_DIR2, LOW);
  digitalWrite(L_ENABLE, LOW);
  digitalWrite(R_ENABLE, LOW);
  digitalWrite(GREENLED, LOW);
  digitalWrite(BLUELED, LOW);
}

void initgyro(void)
{
  Wire.begin();                           //begin the wire comunication
  TWBR = 12;                              //set the I2C frequency to 400kHz
  //at 400kHz the measurement takes about 990us, at 100kHz 2100us.
  //https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config                           //https://mjwhite8119.github.io/Robots/mpu6050
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000 deg/s full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // low pass filter  https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // write to address 26 of the register
  Wire.write(0x03); // options here are 0x00 which is off, and 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
  Wire.endTransmission(true); // 0x06 being the highest filter setting

}

void getAccXZGyrY(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;  //each value needs two registres
  Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;  //the 4096 value comes from the acceleration sensitivity of 8g
  Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;  //see https://mjwhite8119.github.io/Robots/mpu6050
  Acc_X = Acc_rawX / X_correction;
  Acc_Z = Acc_rawZ / Z_correction;

  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);          //We ask for just 4 registers, the Z-axis is not relevant
                                            //This is due to the orientation of the MPU6050 on my robot,                                            
  Gyr_rawX = Wire.read() << 8 | Wire.read();  //The X-axis in my robot is relevant when measuring the steering (not used yet)
  Gyr_rawY = Wire.read() << 8 | Wire.read();  //The Z-axis in my robot is most relevant, this is the tilting angle of the robot

  //Serial.print("\nGyr_rawY (uncorrected): ");
  //Serial.println(Gyr_rawY / 32.8);

  // we measure the angles based on the Gyro en based on the Accelerometer separately and combine them later on in the complementary filter
  // calculate the angle based on Gyro
  Gyr_rawY = ((Gyr_rawY - GyroY_error) / 32.8);           //the 32.8 value comes from the acceleration sensitivity of 1000 deg/s
  //Serial.print("\nGyr_rawY (corrected): ");
  //Serial.println(Gyr_rawY);
  float GyrAngle_incr = Gyr_rawY * elapsed_time / 1000.0; // the change in the angle is calculated from the rotation speed and the time
  //GyrAngle += Gyr_rawY * elapsed_time / 1000.0;
  GyrAngle += GyrAngle_incr;                              // the increment during the measurement interval is added to the current angle
  
  // calculate the angle based on Accelerometer using both axis   
  accAngle = atan2(Acc_Z, Acc_X) * RAD_TO_DEG; // https://www.instructables.com/Arduino-Self-Balancing-Robot-1/

  //calculate angle based on Acc angle and Gyro angle, via the complementary filter see links
  //https://bayesianadventures.wordpress.com/2013/10/20/gyroscopes-accelerometers-and-the-complementary-filter/
  //https://d1.amobbs.com/bbs_upload782111/files_44/ourdev_665531S2JZG6.pdf
  
  //angle_deg = 0.98 * (angle_deg + GyrAngle_incr) + 0.02 * accAngle; //measure interval 10 ms
  //angle_deg = 0.962 * (angle_deg + GyrAngle_incr) + 0.038 * accAngle; //measure interval 20 ms
  angle_deg = 0.99 * (angle_deg + GyrAngle_incr) + 0.01 * accAngle; //measure interval 5 ms
  //angle_deg = accAngle; // this was a first try, way to simple
  prev_angle_dev = angle_dev;             // the prev_angle_dev is used in the D-part of the PID
  angle_dev = balance_angle - angle_deg;  // this is an important value and used the P-part of the PID
  angle_dev_sum += angle_dev;             // the angle_dev_sum is used in the I-part of the PID

  /*
    Serial.println("rawX \t rawZ: ");
    Serial.print(Acc_rawX);
    Serial.print("\t");
    Serial.println(Acc_rawZ);
    Serial.println("corrected values");
    Serial.println("X \t Z ");
    Serial.print(Acc_rawX);
    Serial.print("\t");
    Serial.println(Acc_rawZ);
  */
}
