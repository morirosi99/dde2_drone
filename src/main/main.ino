// main contains the entire sensor reading and calculation of the raw values
#include <Arduino.h>
#include "Wire.h"
#include "motors.h"
#include "pid.h"

motors::Motors m;
pid::PIDCalc calcPID(&m);
unsigned long timer = 0;

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_x_cal, acc_y_cal, acc_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll,angle_yaw;
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
  setup_mpu_6050_registers();

  Serial.println("Calibration of the Gyro");

    for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;
  acc_x_cal /= 2000;
  acc_y_cal /= 2000;
  acc_z_cal /= 2000;

  Serial.println("Gyro Calibrated");
  delay(500);  

  Serial.println("Start Motors");
  delay(500);
  
  m.attach();
  m.startup();
  Serial.println("");

  delay(500);

  m.setThrust(1080);
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
  acc_x -= acc_x_cal;
  acc_y -= acc_y_cal;
  acc_z -= (acc_z_cal + 4096);
  


  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  double pt1 = 0.7;
  gyro_roll_input = (gyro_roll_input * pt1) + ((gyro_roll / 65.5) * (1-pt1));   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * pt1) + ((gyro_pitch / 65.5) * (1-pt1));//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * pt1) + ((gyro_yaw / 65.5) * (1-pt1));      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
  angle_yaw += gyro_yaw * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radian
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
   
  // MANUAL Calibration (instead of void setup calibration): Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0; //-0.85;                                                  //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0; //-1.4;                                                    //Accelerometer calibration value for roll.
  /*
  roll_cum = roll_cum + angle_roll_acc;
  pitch_cum = pitch_cum + angle_pitch_acc;
  Serial.print(pitch_cum / counter);
  Serial.print("; ");
  Serial.print(roll_cum / counter);
  Serial.print("\n"); */

  double alpha = 0.0007;
  angle_pitch = angle_pitch *(1-alpha) + angle_pitch_acc * alpha;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * (1-alpha) + angle_roll_acc * alpha;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch;                            //Calculate the pitch angle correction
  roll_level_adjust = angle_roll;                              //Calculate the roll angle correction
  
  /*
  Serial.print(roll_level_adjust);
  Serial.print(";  ");
  Serial.print(pitch_level_adjust);
  Serial.print(";  ");
  Serial.print(angle_yaw);
  Serial.print("\n"); 
  */

  calcPID.averageError(roll_level_adjust, pitch_level_adjust, angle_yaw);

  if ((millis()-timer)>10) {
        calcPID.computeAverageError();
        calcPID.updateInputs(roll_level_adjust, pitch_level_adjust, angle_yaw);
        calcPID.updatePID();
        calcPID.sendError();

        if (calcPID.getPID()){
//            m.setRandom();
//            delay(1000);
        }
        calcPID.computePID();

        m.setByPID(calcPID.pid_pitch.output, calcPID.pid_roll.output, calcPID.pid_yaw.output);
        timer = millis();
    }

  counter++;
}


