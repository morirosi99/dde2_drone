#include <Wire.h>
#include <Servo.h>

//Declaring some global variables
//gyro
int gyro_x, gyro_y, gyro_z;

//gyro calibration
float gyro_x_cal, gyro_y_cal, gyro_z_cal;

//accelerometer
float acc_x, acc_y, acc_z, acc_total_vector;

// error, integral, derivative
float error_pitch=0;
float integral_pitch=0;
float derivative_pitch=0;
float lastError_pitch=0;

float error_roll=0;
float integral_roll=0;
float derivative_roll=0;
float lastError_roll=0;

float error_yaw=0;
float integral_yaw=0;
float derivative_yaw=0;
float lastError_yaw=0;

// pid outputs
float output_pitch;
float output_roll;
float output_yaw;

// pid constants
float Ki=0;
float Kd=0;
float Kp=0;

// 
bool check;

int temperature;
float loop_timer;

float elapsedTime, time, timePrev;

float angle_pitch, angle_roll, angle_yaw;

Servo fr_prop;
Servo fl_prop;
Servo br_prop;
Servo bl_prop;

float thrust = 1150;

float fr_contrib, br_contrib, fl_contrib, bl_contrib;

float fr_speed, br_speed, fl_speed, bl_speed;



//int angle_pitch_buffer, angle_roll_buffer;
//boolean set_gyro_angles;
//float angle_roll_acc, angle_pitch_acc;
//float angle_pitch_output, angle_roll_output;

//Subroutine for reading the raw gyro and accelerometer data
void read_mpu_6050_data(){                                             
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

// Register MPU6050
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


void pid_update(){

  error_pitch = angle_pitch;
  integral_pitch += error_pitch; 
  derivative_pitch = (error_pitch - lastError_pitch)/elapsedTime;
  lastError_pitch = error_pitch;

  /*Serial.print(error_pitch);
  Serial.print(", ");
  Serial.print(integral_pitch);
  Serial.print(", ");
  Serial.println(derivative_pitch);*/

  error_roll = angle_roll;
  integral_roll += error_roll;
  derivative_roll = (error_roll - lastError_roll)/elapsedTime;
  lastError_roll = error_roll;

  error_yaw = angle_yaw;
  integral_yaw += error_yaw;
  derivative_yaw = (error_yaw - lastError_yaw)/elapsedTime;
  lastError_yaw = error_yaw;
}

void pid_output() {
  output_pitch = Kp * error_pitch + Ki * integral_pitch + Kd * derivative_pitch;
  output_roll = Kp * error_roll + Ki * integral_roll + Kd * derivative_roll;
  output_yaw = Kp * error_yaw + Ki * integral_yaw + Kd * derivative_yaw;

  // calculate the contribution to the ESCs
  fr_contrib = output_pitch - output_roll + output_yaw;
  fr_speed = check_max_min_speed(fr_contrib);


  fl_contrib = output_pitch + output_roll - output_yaw;
  fl_speed = check_max_min_speed(fl_contrib);

  br_contrib = -1*output_pitch - output_roll - output_yaw;
  br_speed = check_max_min_speed(br_contrib);

  bl_contrib = -1*output_pitch + output_roll + output_yaw;
  bl_speed = check_max_min_speed(bl_contrib);
}

void set_pid_constants() {

  Serial.println("Please set PID Constants (Kp;Ki;Kd)");
  check = true;

  while (check){
    if (Serial.available() > 0) {

      
      String input = Serial.readString();
      Serial.println();

      // Separate input
      int pos1 = input.indexOf(';');
      int pos2 = input.indexOf(';', pos1 + 1);

      // Extrahiere die einzelnen Werte
      String kpStr = input.substring(0, pos1);
      String kiStr = input.substring(pos1 + 1, pos2);
      String kdStr = input.substring(pos2 + 1);

      // Konvertiere die Werte in Floats
      Kp = kpStr.toFloat();
      Ki = kiStr.toFloat();
      Kd = kdStr.toFloat();

      Serial.println("Kp: " + String(Kp));
      Serial.println("Ki: " + String(Ki));
      Serial.println("Kd: " + String(Kd));
      Serial.println();

      check = false;
    }
  }
}

void motor_startup(){
  fr_prop.attach(5);
  fl_prop.attach(4);
  br_prop.attach(6);
  bl_prop.attach(7);


  fr_prop.writeMicroseconds(1015);
  fl_prop.writeMicroseconds(1015);
  br_prop.writeMicroseconds(1015);
  bl_prop.writeMicroseconds(1015);
  

  delay(2000);

  fr_prop.writeMicroseconds(thrust);
  fl_prop.writeMicroseconds(thrust);
  br_prop.writeMicroseconds(thrust);
  bl_prop.writeMicroseconds(thrust);

}

float check_max_min_speed(float contrib){
  
  float speed;


  if (thrust + contrib < 1100){
    speed = 1100;
  }
  else if (thrust + contrib > 1200){
    speed = 1200;
  }
  else {
    speed = thrust + contrib;
  }
  return speed;
}

void motor_set_thrust(float fr, float fl, float br, float bl){
  fr_prop.writeMicroseconds(fr);
  fl_prop.writeMicroseconds(fl);
  br_prop.writeMicroseconds(br);
  bl_prop.writeMicroseconds(bl);

  Serial.print(fr);
  Serial.print(", ");
  Serial.print(fl);
  Serial.print(", ");
  Serial.print(br);
  Serial.print(", ");
  Serial.println(bl);


}


// SETUP
void setup() {

  Wire.begin();                                                        //Start I2C as master
  
  Serial.begin(57600);                                                 //Use only for debugging

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  
  set_pid_constants();
 
  // Gyro Calibration
  Serial.println("Start Gyro Calibration");
  Serial.println();
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }

  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  
  Serial.println("Gyro Calibration Done");
  Serial.println();

  delay(1000); 

  //m.attach(); 
  //m.startup();
  //m.setThrust(1100);

  motor_startup();

  delay(3000);

  time = millis();  
  loop_timer = millis();                                           //Reset the loop timer
}

// LOOP
void loop(){
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;
  
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //angle_yaw = gyro_z * 0.0000611;
  angle_yaw = 0;                                
  /*Serial.print(angle_pitch);
  Serial.print(", ");
  Serial.print(angle_roll);
  Serial.print(", ");
  Serial.println(angle_yaw);*/

  pid_update();

  pid_output();
  
  motor_set_thrust(fr_speed, fl_speed, br_speed, bl_speed);

  
  /*
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  
  */
  while(millis() - loop_timer < 4);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = millis();                                               //Reset the loop timer
}

















