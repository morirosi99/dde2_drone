// main contains the entire sensor reading and calculation of the raw values
#include <Arduino.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "motors.h"

motors::Motors m;

//Declaring some global variables
boolean motorsON = false;                                              // Set true to turn motors on
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

int gyro_x_Error, gyro_y_Error, gyro_z_Error;

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable, [deg/s], 8 bit
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable, [deg/s], 8 bit
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable, [deg/s], 8 bit
}

void setup() {                                                         // Setup() calculates the offset of the gyro (x,y,z) which are used for calibration
  Wire.begin();
  Serial.begin(115200);

  if (motorsON){
    m.attach();                                                          // Attach the motors
    m.startup();                                                         // Power the motors, should not start the motors because min speed is 1050   
  }
                                                           
  setup_mpu_6050_registers();

    for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;
  
  delay(10000);                                                        // Wait 10 sec
  
  if (motorsON){
    m.setThrust(1050);                                                 // Start the motors
  }                                                 
}

int gyro_roll;
int gyro_pitch;
int gyro_yaw;
float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;
float roll_level_adjust, pitch_level_adjust;
int counter = 1;
double roll_cum = 0;
double pitch_cum = 0;

void loop(){
  read_mpu_6050_data();  
  
  gyro_roll = gyro_x;                                                       //Read the raw acc and gyro data from the MPU-6050
  gyro_pitch = gyro_y; 
  gyro_yaw = gyro_z;     

  gyro_roll -= gyro_x_cal;                                                  // Add the offsets values of void setup()
  gyro_pitch -= gyro_y_cal; 
  gyro_yaw -= gyro_z_cal;


  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
 
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radian
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }

  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }

   
  // MANUAL Calibration (instead of void setup calibration): Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  // angle_pitch_acc -= 8.18;                                                  //Accelerometer calibration value for pitch.
  // angle_roll_acc -= 4.6;                                                    //Accelerometer calibration value for roll.

  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
 
  roll_level_adjust = angle_roll * 15 + 224.88;                             //Calculate the roll angle correction
  pitch_level_adjust = angle_pitch * 15 - 92.81;                            //Calculate the pitch angle correction
 
  Serial.print(roll_level_adjust);
  Serial.print(";  ");  
  Serial.print(pitch_level_adjust);
  Serial.print(";  ");
  Serial.print(gyro_yaw_input);
  Serial.print("\n");

  if (motorsON){
    //CHECK and UNDERSTAND functionality first
    //m.setByPID(pitch, roll, yaw)                                          // Sets the thrust of the motors according to the calculated error                                                               
  }

  counter++;
}

