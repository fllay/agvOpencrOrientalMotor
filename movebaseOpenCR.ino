#include "ModbusBLVmotor.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


#include <RTOS.h>


#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "lino_velocities.h"
#include "geometry_msgs/Twist.h"
#include "lino_imu.h"
#include "Kinematics.h"

#include <IMU.h>

#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
#define MAX_RPM 100               // motor's maximum RPM
#define WHEEL_DIAMETER 0.150       // wheel's diameter in meters
//#define LR_WHEELS_DISTANCE 0.395  // distance between left and right wheels 0.800

//#define LR_WHEELS_DISTANCE 0.538  // distance between left and right wheels 0.800
#define LR_WHEELS_DISTANCE 0.30  // distance between left and right wheels 0.800

#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define USE_MPU9250_IMU

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8



#define COMMAND_RATE 30

cIMU    IMU;

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);


int led_state = 0;

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

int currentLeftWheelRPM;
int currentRightWheelRPM;
std_msgs::Int32 rpmLeft;
std_msgs::Int32 rpmRight;

void commandCallback(const geometry_msgs::Twist& cmd_msg);
void moveBase();


void waitForSerialLink(bool isConnected);


ros::NodeHandle nh;


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

ModbusBLVmotor md; 


ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

/*lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
*/

//***** IMU Pub declear ****

sensor_msgs::Imu imu;
ros::Publisher imu_pub("/imu/data", &imu);
sensor_msgs::MagneticField raw_data_mag;
ros::Publisher raw_mag_data_pub("/imu/mag", &raw_data_mag);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

ros::Publisher rpmLeft_pub("rpmLeft", &rpmLeft);

ros::Publisher rpmRight_pub("rpmRight", &rpmRight);



int led_pin = 13;
unsigned long prev_update_time_m1 = 0;
unsigned long prev_update_time_m2 = 0;

unsigned long pulseRPM1 = 0;
unsigned long pulseRPM2 = 0;

unsigned long prev_update_time_M1 = 0;
unsigned long prev_update_time_M2 = 0;

unsigned long prev_encoder_ticks_M1 = 0;
unsigned long prev_encoder_ticks_M2 = 0;
int counts_per_rev_ = 15;


osThreadId thread_command;
osThreadId thread_odom;
osThreadId thread_imu;


void m1Pulse(void){
  unsigned long current_time_m1 = micros();;
  unsigned long dt = current_time_m1 - prev_update_time_m1;
  prev_update_time_m1 = micros();
  float freq_m1 = 1/(dt*0.000001);
  pulseRPM1++;
  //Serial.print("m1 = ");
  //Serial.println(freq_m1);
}

void m2Pulse(void){
  unsigned long current_time_m2 = micros();;
  unsigned long dt = current_time_m2 - prev_update_time_m2;
  prev_update_time_m2 = micros();
  float freq_m2 = 1/(dt*0.000001);
  pulseRPM2++;
  //Serial.print("m2 = ");
  //Serial.println(freq_m2);  
}


int getRPM1(){
    unsigned long encoder_ticks = pulseRPM1;
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_update_time_M1;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000.0;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_M1;

    //calculate wheel's speed (RPM)

    prev_update_time_M1 = current_time;
    prev_encoder_ticks_M1 = encoder_ticks;
    
    return ((delta_ticks / counts_per_rev_) / dtm)/60; 
}

int getRPM2(){
    unsigned long encoder_ticks = pulseRPM2;
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_update_time_M2;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000.0;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_M2;

    //calculate wheel's speed (RPM)

    prev_update_time_M2 = current_time;
    prev_encoder_ticks_M2 = encoder_ticks;
    
    return ((delta_ticks / counts_per_rev_) / dtm)/60;    
    
}

void setLeftRPM(int rpm){
    if(rpm < 0){
        //M1_dir = 1;
        currentLeftWheelRPM = -1*getRPM1();
    } else {
        //M1_dir = 0;
        currentLeftWheelRPM = getRPM1();
    }
    
        

   md.setM1Speed(rpm);     
       
}

void setRightRPM(int rpm){
    
       if(rpm < 0){
        //M2_dir = 1;
        currentRightWheelRPM = -1*getRPM2();
    } else {
        //M2_dir = 0;
        currentRightWheelRPM = getRPM2();
    }

    
        

    md.setM2Speed(rpm);
}


static void Thread_command(void const *argument)
{
  (void) argument;
  static unsigned long prev_control_time = 0;

  for(;;){
      if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)){   
     
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    setLeftRPM(req_rpm.motor1);
    setRightRPM(req_rpm.motor2);
    digitalWrite(led_pin, (led_state) ? HIGH : LOW);
     led_state = !led_state;
    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
    prev_control_time = millis();
  }
  }

   nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
 
}


static void Thread_odom(void const *argument)
{
  (void) argument;
  static unsigned long prev_odom_time = 0;
  Kinematics::velocities current_vel;

  for(;;){
    if ((millis() - prev_odom_time) >= (1000 / COMMAND_RATE)){  
        int current_rpm1 = currentLeftWheelRPM; //rightWheel.getRPM();
    int current_rpm2 = currentRightWheelRPM; //leftWheel.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;
    rpmLeft.data = currentLeftWheelRPM;
    rpmRight.data = currentRightWheelRPM;
    
    rpmLeft_pub.publish(&rpmLeft);
    rpmRight_pub.publish(&rpmRight);

    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    //current_vel = kinematics.getVelocities(50, 50, 0, 0);
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
    prev_odom_time = millis();
    }

     nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
      
  }
 
}

static void Thread_imu(void const *argument)
{
  (void) argument;
   static unsigned long prev_imu_time = 0;
  
for(;;){
  if ((millis() - prev_imu_time) >= 1000/50){

          IMU.update();

           /* raw_imu_msg.linear_acceleration.x = IMU.SEN.accADC[0] * ACCEL_FACTOR;
            raw_imu_msg.linear_acceleration.y = IMU.SEN.accADC[1] * ACCEL_FACTOR;
            raw_imu_msg.linear_acceleration.z = IMU.SEN.accADC[2] * ACCEL_FACTOR;

           
            raw_imu_msg.angular_velocity.x = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
            raw_imu_msg.angular_velocity.y = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
            raw_imu_msg.angular_velocity.z = IMU.SEN.gyroADC[2] * GYRO_FACTOR;

            
            raw_imu_msg.magnetic_field.x = IMU.SEN.magADC[0] * MAG_FACTOR;
            raw_imu_msg.magnetic_field.y = IMU.SEN.magADC[1] * MAG_FACTOR;
            raw_imu_msg.magnetic_field.z = IMU.SEN.magADC[2] * MAG_FACTOR;


            raw_imu_pub.publish(&raw_imu_msg);*/


             imu.angular_velocity.x = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
            imu.angular_velocity.y = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
            imu.angular_velocity.z = IMU.SEN.gyroADC[2] * GYRO_FACTOR;
            imu.angular_velocity_covariance[0] = 0.02;
            imu.angular_velocity_covariance[1] = 0;
            imu.angular_velocity_covariance[2] = 0;
            imu.angular_velocity_covariance[3] = 0;
            imu.angular_velocity_covariance[4] = 0.02;
            imu.angular_velocity_covariance[5] = 0;
            imu.angular_velocity_covariance[6] = 0;
            imu.angular_velocity_covariance[7] = 0;
            imu.angular_velocity_covariance[8] = 0.02;

            imu.linear_acceleration.x = IMU.SEN.accADC[0] * ACCEL_FACTOR;
            imu.linear_acceleration.y = IMU.SEN.accADC[1] * ACCEL_FACTOR;
            imu.linear_acceleration.z = IMU.SEN.accADC[2] * ACCEL_FACTOR;

            imu.linear_acceleration_covariance[0] = 0.04;
            imu.linear_acceleration_covariance[1] = 0;
            imu.linear_acceleration_covariance[2] = 0;
            imu.linear_acceleration_covariance[3] = 0;
            imu.linear_acceleration_covariance[4] = 0.04;
            imu.linear_acceleration_covariance[5] = 0;
            imu.linear_acceleration_covariance[6] = 0;
            imu.linear_acceleration_covariance[7] = 0;
            imu.linear_acceleration_covariance[8] = 0.04;

            imu.orientation.w = IMU.quat[0];
            imu.orientation.x = IMU.quat[1];
            imu.orientation.y = IMU.quat[2];
            imu.orientation.z = IMU.quat[3];

            imu.orientation_covariance[0] = 0.0025;
            imu.orientation_covariance[1] = 0;
            imu.orientation_covariance[2] = 0;
            imu.orientation_covariance[3] = 0;
            imu.orientation_covariance[4] = 0.0025;
            imu.orientation_covariance[5] = 0;
            imu.orientation_covariance[6] = 0;
            imu.orientation_covariance[7] = 0;
            imu.orientation_covariance[8] = 0.0025;     
            imu.header.stamp    = rosNow();
            imu.header.frame_id = "imu_link"; //imu_frame_id;
            imu_pub.publish(&imu);

            raw_data_mag.magnetic_field.x = IMU.SEN.magADC[0] * MAG_FACTOR;
            raw_data_mag.magnetic_field.y = IMU.SEN.magADC[1] * MAG_FACTOR;
            raw_data_mag.magnetic_field.z = IMU.SEN.magADC[2] * MAG_FACTOR;

            raw_data_mag.magnetic_field_covariance[0] = 0.0048;
            raw_data_mag.magnetic_field_covariance[1] = 0;
            raw_data_mag.magnetic_field_covariance[2] = 0;
            raw_data_mag.magnetic_field_covariance[3] = 0;
            raw_data_mag.magnetic_field_covariance[4] = 0.0048;
            raw_data_mag.magnetic_field_covariance[5] = 0;
            raw_data_mag.magnetic_field_covariance[6] = 0;
            raw_data_mag.magnetic_field_covariance[7] = 0;
            raw_data_mag.magnetic_field_covariance[8] = 0.0048;
            raw_data_mag.header.stamp    = rosNow();
            raw_data_mag.header.frame_id = "imu_link  `"; //mag_frame_id;

            raw_mag_data_pub.publish(&raw_data_mag);






           prev_imu_time = millis();
        }
  }

   nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
 
}


void setup() {


   pinMode(led_pin, OUTPUT);
  pinMode(BDPIN_LED_USER_1, OUTPUT);


  
  // put your setup code here, to run once:
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  
  //nh.advertise(raw_imu_pub);


  nh.advertise(imu_pub);
  nh.advertise(raw_mag_data_pub);
  
  nh.advertise(rpmLeft_pub);
  nh.advertise(rpmRight_pub);


  

  
  //Serial.begin(115200); 
  //Serial.println("Start");
  pinMode(7, INPUT); //set Arduino Pin 2 as input with pull-down
  attachInterrupt(3, m1Pulse, RISING);


  pinMode(8, INPUT); //set Arduino Pin 2 as input with pull-down
  attachInterrupt(4, m2Pulse, RISING);
  
  pinMode(led_pin, OUTPUT);
  md.init();  
  IMU.begin();
  calibrationGyro();

     // define thread
  osThreadDef(THREAD_NAME_COMMAND, Thread_command, osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_NAME_ODOM,  Thread_odom,  osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_NAME_IMU,  Thread_imu,  osPriorityNormal, 0, 1024);

  // create thread
  thread_command = osThreadCreate(osThread(THREAD_NAME_COMMAND), NULL);
  thread_odom  = osThreadCreate(osThread(THREAD_NAME_ODOM), NULL);
  thread_imu  = osThreadCreate(osThread(THREAD_NAME_IMU), NULL);

  // start kernel
  osKernelStart();


}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/

void calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  //const uint8_t led_ros_connect = 3;

  IMU.SEN.gyro_cali_start();
  
  t_time   = millis();
  pre_time = millis();

  while(!IMU.SEN.gyro_cali_get_done())
  {
    IMU.update();

    if (millis()-pre_time > 20000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      //setLedToggle(led_ros_connect);
      digitalWrite(BDPIN_LED_USER_1, !digitalRead(BDPIN_LED_USER_1));
    }
  }

  digitalWrite(BDPIN_LED_USER_1,HIGH);

  
}


void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}


void loop() {
 
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_control_time = 0;
  //osDelay(100);  
  // put your main code here, to run repeatedly:
  //digitalWrite(led_pin, HIGH);  // set to as HIGH LED is turn-off
  //delay(200);                   // Wait for 0.1 second
  //digitalWrite(led_pin, LOW);   // set to as LOW LED is turn-on
  //delay(200);         // Wait for 0.1 second
  //md.setM1Speed(40);
  //delay(2000);  
  //md.setM2Speed(40);
  //delay(2000); 
  //md.setM2Speed(0); 
  //delay(2000); 
  

  //Serial.println("5555");
  //Serial2.print("66656");


        //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)){   
    //moveBase();
    //digitalWrite(led_pin, (led_state) ? HIGH : LOW);
     //led_state = !led_state;
    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
    prev_control_time = millis();
  }

  
  
  //updateGyroCali(nh.connected());
  
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

void stopBase()
{   
    
  md.setM1Speed(0);
  //delay(2000);  
  md.setM2Speed(0);
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void moveBase(){
    
   Kinematics::velocities current_vel;
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    setLeftRPM(req_rpm.motor1);
    setRightRPM(req_rpm.motor2);

    /*int current_rpm1 = currentLeftWheelRPM; //rightWheel.getRPM();
    int current_rpm2 = currentRightWheelRPM; //leftWheel.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;
    rpmLeft.data = currentLeftWheelRPM;
    rpmRight.data = currentRightWheelRPM;
    
    rpmLeft_pub.publish(&rpmLeft);
    rpmRight_pub.publish(&rpmRight);

    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    //current_vel = kinematics.getVelocities(50, 50, 0, 0);
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);*/
      

}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //char buffer[40];  
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;


    //moveBase();


    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
            
    g_prev_command_time = millis();
     
     //nh.spinOnce();

  // Wait the serial link time to process
  //waitForSerialLink(nh.connected());
}
