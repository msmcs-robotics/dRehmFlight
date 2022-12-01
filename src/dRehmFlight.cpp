// Original code by Nick Rhem: 
// https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino


//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
//#define USE_PWM_RX
#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX

static const int num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G


//========================================================================================================================//


#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators
#include <radiosetup.h> //Radio setup
#include <print_funcs.h> //Functions for printing data to serial monitor
#include <Ultrasonic.h> //Ultrasonic sensors
#include <vector> //Vectors for storing sensor data

#if defined USE_SBUS_RX
  #include "src/SBUS/SBUS.h"   //sBus interface
#endif

#if defined USE_DSM_RX
  #include "src/DSMRX/DSMRX.h"  
#endif

#if defined USE_MPU6050_I2C
  #include "MPU6050.h"
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "MPU9250.h"
  MPU9250 mpu9250(SPI2,36);
#else
  #error No MPU defined... 
#endif

//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif

//========================================================================================================================//
//                                                     PINOUTS                                                            //                           
//========================================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)

const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
const int ch6Pin = 22; //aux1 (free aux channel)
const int PPM_Pin = 23;

struct {
  
    //OneShot125 ESC pin outputs:
    const int m1Pin = 0;
    const int m2Pin = 1;
    const int m3Pin = 2;
    const int m4Pin = 3;
    const int m5Pin = 4;
    const int m6Pin = 5;

    //PWM servo or ESC outputs:
    const int servo1Pin = 6;
    const int servo2Pin = 7;
    const int servo3Pin = 8;
    const int servo4Pin = 9;
    const int servo5Pin = 10;
    const int servo6Pin = 11;
    const int servo7Pin = 12;

    // Ultrasonic sensors
    const int us1T = 1;
    const int us1E = 2;
    const int us2T = 3;
    const int us2E = 4;
    const int us3T = 5;
    const int us3E = 6;
    const int us4T = 7;
    const int us4E = 8; 

    // Servos for ultrasonic sensors
    const int us_s1 = 9;
    const int us_s2 = 10;
    const int us_s3 = 11;
    const int us_s4 = 12;

} pinout;

//Create servo objects to control a servo or ESC with PWM

PWMServo servo1; 
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

PWMServo us_s1;
PWMServo us_s2;
PWMServo us_s3;
PWMServo us_s4;

// Ultrasonic Sensor Objects

Ultrasonic us1(pinout.us1T, pinout.us1E);
Ultrasonic us2(pinout.us2T, pinout.us2E);
Ultrasonic us3(pinout.us3T, pinout.us3E);
Ultrasonic us4(pinout.us4T, pinout.us4E);

//========================================================================================================================//
//                                                     VARS                                                               //                           
//========================================================================================================================//

struct {
    
    // Servos
    const int min_PWM = 900;
    const int max_PWM = 2100;
    const float half_speed = max_PWM - (max_PWM - min_PWM)/2;

    // Ultrasonic sensors

    const int min_us_range = 10;
    const int max_us_range = 100;

} module_info;

struct {
  // Baud rate
  int baud = 9600;


  //Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
  struct {
    unsigned long channel_1_fs = 1000; //thro
    unsigned long channel_2_fs = 1500; //ail
    unsigned long channel_3_fs = 1500; //elev
    unsigned long channel_4_fs = 1500; //rudd
    unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
    unsigned long channel_6_fs = 2000; //aux1
  } failsafe_struct;

  //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
  struct {
    float B_madgwick = 0.04;  //Madgwick filter parameter
    float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    float B_mag = 1.0;        //Magnetometer LP filter parameter
  } filter_struct;

  //Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
  struct {
    float MagErrorX = 0.0;
    float MagErrorY = 0.0; 
    float MagErrorZ = 0.0;
    float MagScaleX = 1.0;
    float MagScaleY = 1.0;
    float MagScaleZ = 1.0;
  } magno_cal_struct;

  //IMU calibration parameters - calibrate IMU using imuf.calculate_IMU_error() in the void setup() to get these values, then comment out imuf.calculate_IMU_error()
  struct {
    float AccErrorX = 0.0;
    float AccErrorY = 0.0;
    float AccErrorZ = 0.0;
    float GyroErrorX = 0.0;
    float GyroErrorY= 0.0;
    float GyroErrorZ = 0.0;
  } imu_cal_struct;

  //Controller parameters (take note of defaults before modifying!): 
  struct {
    float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
    float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
    float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
    float maxYaw = 160.0;     //Max yaw rate in deg/sec

    float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
    float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
    float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on ctrlfuncs.controlANGLE2)
    float B_loop_roll = 0.9;      //Roll damping term for ctrlfuncs.controlANGLE2(), lower is more damping (must be between 0 to 1)
    float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
    float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
    float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on ctrlfuncs.controlANGLE2)
    float B_loop_pitch = 0.9;     //Pitch damping term for ctrlfuncs.controlANGLE2(), lower is more damping (must be between 0 to 1)

    float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
    float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
    float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
    float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
    float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
    float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    float Kp_yaw = 0.3;           //Yaw P-gain
    float Ki_yaw = 0.05;          //Yaw I-gain
    float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  } ctrl_params;

} usrVARS;

struct {
  //IMU:

  struct {
    float AccX, AccY, AccZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX, GyroY, GyroZ;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float MagX, MagY, MagZ;
    float MagX_prev, MagY_prev, MagZ_prev;
    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;
    float q0 = 1.0f; //Initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
  } IMU;

  //Normalized desired state:

  struct {
    float thro_des, roll_des, pitch_des, yaw_des;
    float roll_passthru, pitch_passthru, yaw_passthru;
  } nml_des_state;

  //Controller:
  struct {
    float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
    float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
    float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
  } ctrl;

  //Mixer
  struct {
    float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
    int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
    float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
    int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;
  } mixer;

} regVARS;

//GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

#if defined USE_SBUS_RX
  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
  DSM1024 DSM;
#endif

//========================================================================================================================//
//                                               Regular Classes                                                              //
//========================================================================================================================//


class HelperFuncs {
  public:

    float invSqrt(float x) {
      //Fast inverse sqrt for madgwick filter
      /*
      float halfx = 0.5f * x;
      float y = x;
      long i = *(long*)&y;
      i = 0x5f3759df - (i>>1);
      y = *(float*)&i;
      y = y * (1.5f - (halfx * y * y));
      y = y * (1.5f - (halfx * y * y));
      return y;
      */
      /*
      //alternate form:
      unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
      float tmp = *(float*)&i;
      float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
      return y;
      */
      return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
    }

    float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
      //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
      /*  
      *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
      *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in ctrlfuncs.controlMixer()
      *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
      *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
      *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
      *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
      *  
      */
      float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

      if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
        param = param + diffParam;
      }
      else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
        param = param - diffParam;
      }

      param = constrain(param, param_min, param_max); //Constrain param within max bounds
      
      return param;
    }

    float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
      //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
      /*  
      *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 
      *  and linearly fades that param variable up or down to the desired value. This function can be called in ctrlfuncs.controlMixer()
      *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 
      *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
      *  statements in order to fade controller gains, for example between the two dynamic configurations. 
      *  
      */
      if (param > param_des) { //Need to fade down to get to desired
        float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
        param = param - diffParam;
      }
      else if (param < param_des) { //Need to fade up to get to desired
        float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
        param = param + diffParam;
      }

      param = constrain(param, param_lower, param_upper); //Constrain param within max bounds
      
      return param;
    }

};

class ReadRadio {

  public:
  
    void scaleCommands() {
      //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
      /*
      * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
      * the mixer function are scaled to 0-180 for the servo library using standard PWM.
      * mX_command_PWM are updated here which are used to command the motors in motorfuncs.commandMotors(). sX_command_PWM are updated 
      * which are used to command the servos.
      */
      //Scaled to 125us - 250us for oneshot125 protocol
      regVARS.mixer.m1_command_PWM = regVARS.mixer.m1_command_scaled*125 + 125;
      regVARS.mixer.m2_command_PWM = regVARS.mixer.m2_command_scaled*125 + 125;
      regVARS.mixer.m3_command_PWM = regVARS.mixer.m3_command_scaled*125 + 125;
      regVARS.mixer.m4_command_PWM = regVARS.mixer.m4_command_scaled*125 + 125;
      regVARS.mixer.m5_command_PWM = regVARS.mixer.m5_command_scaled*125 + 125;
      regVARS.mixer.m6_command_PWM = regVARS.mixer.m6_command_scaled*125 + 125;
      //Constrain commands to motors within oneshot125 bounds
      regVARS.mixer.m1_command_PWM = constrain(regVARS.mixer.m1_command_PWM, 125, 250);
      regVARS.mixer.m2_command_PWM = constrain(regVARS.mixer.m2_command_PWM, 125, 250);
      regVARS.mixer.m3_command_PWM = constrain(regVARS.mixer.m3_command_PWM, 125, 250);
      regVARS.mixer.m4_command_PWM = constrain(regVARS.mixer.m4_command_PWM, 125, 250);
      regVARS.mixer.m5_command_PWM = constrain(regVARS.mixer.m5_command_PWM, 125, 250);
      regVARS.mixer.m6_command_PWM = constrain(regVARS.mixer.m6_command_PWM, 125, 250);

      //Scaled to 0-180 for servo library
      regVARS.mixer.s1_command_PWM = regVARS.mixer.s1_command_scaled*180;
      regVARS.mixer.s2_command_PWM = regVARS.mixer.s2_command_scaled*180;
      regVARS.mixer.s3_command_PWM = regVARS.mixer.s3_command_scaled*180;
      regVARS.mixer.s4_command_PWM = regVARS.mixer.s4_command_scaled*180;
      regVARS.mixer.s5_command_PWM = regVARS.mixer.s5_command_scaled*180;
      regVARS.mixer.s6_command_PWM = regVARS.mixer.s6_command_scaled*180;
      regVARS.mixer.s7_command_PWM = regVARS.mixer.s7_command_scaled*180;
      //Constrain commands to servos within servo library bounds
      regVARS.mixer.s1_command_PWM = constrain(regVARS.mixer.s1_command_PWM, 0, 180);
      regVARS.mixer.s2_command_PWM = constrain(regVARS.mixer.s2_command_PWM, 0, 180);
      regVARS.mixer.s3_command_PWM = constrain(regVARS.mixer.s3_command_PWM, 0, 180);
      regVARS.mixer.s4_command_PWM = constrain(regVARS.mixer.s4_command_PWM, 0, 180);
      regVARS.mixer.s5_command_PWM = constrain(regVARS.mixer.s5_command_PWM, 0, 180);
      regVARS.mixer.s6_command_PWM = constrain(regVARS.mixer.s6_command_PWM, 0, 180);
      regVARS.mixer.s7_command_PWM = constrain(regVARS.mixer.s7_command_PWM, 0, 180);

    }

    void getCommands() {
      //DESCRIPTION: Get raw PWM values for every channel from the radio
      /*
      * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
      * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
      * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
      * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
      */

      #if defined USE_PPM_RX || defined USE_PWM_RX
        channel_1_pwm = getRadioPWM(1);
        channel_2_pwm = getRadioPWM(2);
        channel_3_pwm = getRadioPWM(3);
        channel_4_pwm = getRadioPWM(4);
        channel_5_pwm = getRadioPWM(5);
        channel_6_pwm = getRadioPWM(6);
        
      #elif defined USE_SBUS_RX
        if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
        {
          //sBus scaling below is for Taranis-Plus and X4R-SB
          float scale = 0.615;  
          float bias  = 895.0; 
          channel_1_pwm = sbusChannels[0] * scale + bias;
          channel_2_pwm = sbusChannels[1] * scale + bias;
          channel_3_pwm = sbusChannels[2] * scale + bias;
          channel_4_pwm = sbusChannels[3] * scale + bias;
          channel_5_pwm = sbusChannels[4] * scale + bias;
          channel_6_pwm = sbusChannels[5] * scale + bias; 
        }

      #elif defined USE_DSM_RX
        if (DSM.timedOut(micros())) {
            //Serial.println("*** DSM RX TIMED OUT ***");
        }
        else if (DSM.gotNewFrame()) {
            uint16_t values[num_DSM_channels];
            DSM.getChannelValues(values, num_DSM_channels);

            channel_1_pwm = values[0];
            channel_2_pwm = values[1];
            channel_3_pwm = values[2];
            channel_4_pwm = values[3];
            channel_5_pwm = values[4];
            channel_6_pwm = values[5];
        }
      #endif
      
      //Low-pass the critical commands and update previous values
      float b = 0.7; //Lower=slower, higher=noiser
      channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
      channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
      channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
      channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
      channel_1_pwm_prev = channel_1_pwm;
      channel_2_pwm_prev = channel_2_pwm;
      channel_3_pwm_prev = channel_3_pwm;
      channel_4_pwm_prev = channel_4_pwm;
    }

    void failSafe() {
      //DESCRIPTION: If radio gives garbage values, set all commands to default values
      /*
      * Radio connection failsafe used to check if the readradio.getCommands() function is returning acceptable pwm values. If any of 
      * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
      * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
      * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
      * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
      */
      unsigned minVal = 800;
      unsigned maxVal = 2200;
      int check1 = 0;
      int check2 = 0;
      int check3 = 0;
      int check4 = 0;
      int check5 = 0;
      int check6 = 0;

      //Triggers for failure criteria
      if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
      if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
      if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
      if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
      if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
      if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

      //If any failures, set to default failsafe values
      if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
        channel_1_pwm = usrVARS.failsafe_struct.channel_1_fs;
        channel_2_pwm = usrVARS.failsafe_struct.channel_2_fs;
        channel_3_pwm = usrVARS.failsafe_struct.channel_3_fs;
        channel_4_pwm = usrVARS.failsafe_struct.channel_4_fs;
        channel_5_pwm = usrVARS.failsafe_struct.channel_5_fs;
        channel_6_pwm = usrVARS.failsafe_struct.channel_6_fs;
      }
    }

    void getDesState() {
      //DESCRIPTION: Normalizes desired control values to appropriate values
      /*
      * Updates the desired state variables regVARS.nml_des_state.thro_des, regVARS.nml_des_state.roll_des, regVARS.nml_des_state.pitch_des, and regVARS.nml_des_state.yaw_des. These are computed by using the raw
      * RC pwm commands and scaling them to be within our limits defined in setup. regVARS.nml_des_state.thro_des stays within 0 to 1 range.
      * regVARS.nml_des_state.roll_des and regVARS.nml_des_state.pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
      * (rate mode). regVARS.nml_des_state.yaw_des is scaled to be within max yaw in degrees/sec. Also creates regVARS.nml_des_state.roll_passthru, regVARS.nml_des_state.pitch_passthru, and
      * regVARS.nml_des_state.yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in ctrlfuncs.controlMixer().
      */
      regVARS.nml_des_state.thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
      regVARS.nml_des_state.roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
      regVARS.nml_des_state.pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
      regVARS.nml_des_state.yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
      regVARS.nml_des_state.roll_passthru = regVARS.nml_des_state.roll_des/2.0; //Between -0.5 and 0.5
      regVARS.nml_des_state.pitch_passthru = regVARS.nml_des_state.pitch_des/2.0; //Between -0.5 and 0.5
      regVARS.nml_des_state.yaw_passthru = regVARS.nml_des_state.yaw_des/2.0; //Between -0.5 and 0.5
      
      //Constrain within normalized bounds
      regVARS.nml_des_state.thro_des = constrain(regVARS.nml_des_state.thro_des, 0.0, 1.0); //Between 0 and 1
      regVARS.nml_des_state.roll_des = constrain(regVARS.nml_des_state.roll_des, -1.0, 1.0)*usrVARS.ctrl_params.maxRoll; //Between -usrVARS.ctrl_params.maxRoll and +usrVARS.ctrl_params.maxRoll
      regVARS.nml_des_state.pitch_des = constrain(regVARS.nml_des_state.pitch_des, -1.0, 1.0)*usrVARS.ctrl_params.maxPitch; //Between -usrVARS.ctrl_params.maxPitch and +usrVARS.ctrl_params.maxPitch
      regVARS.nml_des_state.yaw_des = constrain(regVARS.nml_des_state.yaw_des, -1.0, 1.0)*usrVARS.ctrl_params.maxYaw; //Between -usrVARS.ctrl_params.maxYaw and +usrVARS.ctrl_params.maxYaw
      regVARS.nml_des_state.roll_passthru = constrain(regVARS.nml_des_state.roll_passthru, -0.5, 0.5);
      regVARS.nml_des_state.pitch_passthru = constrain(regVARS.nml_des_state.pitch_passthru, -0.5, 0.5);
      regVARS.nml_des_state.yaw_passthru = constrain(regVARS.nml_des_state.yaw_passthru, -0.5, 0.5);
    }

    void throttleCut() {
      //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
      /*
      * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      * minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function 
      * called before motorfuncs.commandMotors() is called so that the last thing checked is if the user is giving permission to command
      * the motors to anything other than minimum value. Safety first. 
      */
      if (channel_5_pwm > 1500) {
        regVARS.mixer.m1_command_PWM = 120;
        regVARS.mixer.m2_command_PWM = 120;
        regVARS.mixer.m3_command_PWM = 120;
        regVARS.mixer.m4_command_PWM = 120;
        regVARS.mixer.m5_command_PWM = 120;
        regVARS.mixer.m6_command_PWM = 120;
        
        //Uncomment if using servo PWM variables to control motor ESCs
        //regVARS.mixer.s1_command_PWM = 0;
        //regVARS.mixer.s2_command_PWM = 0;
        //regVARS.mixer.s3_command_PWM = 0;
        //regVARS.mixer.s4_command_PWM = 0;
        //regVARS.mixer.s5_command_PWM = 0;
        //regVARS.mixer.s6_command_PWM = 0;
        //regVARS.mixer.s7_command_PWM = 0;
      }
    }

};
class CtrlFuncs {
  
  ReadRadio readradio;

  public:
  
    void controlMixer() {
      //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
      /*
      * Takes regVARS.ctrl.roll_PID, regVARS.ctrl.pitch_PID, and regVARS.ctrl.yaw_PID computed from the PID controller and appropriately mixes them for the desired
      * vehicle configuration. For example on a quadcopter, the left two motors should have +regVARS.ctrl.roll_PID while the right two motors
      * should have -regVARS.ctrl.roll_PID. Front two should have -regVARS.ctrl.pitch_PID and the back two should have +regVARS.ctrl.pitch_PID etc... every motor has
      * normalized (0 to 1) regVARS.nml_des_state.thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
      * regVARS.nml_des_state.roll_passthru, regVARS.nml_des_state.pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in readradio.scaleCommands() 
      * in preparation to be sent to the motor ESCs and servos.
      * 
      *Relevant variables:
      *regVARS.nml_des_state.thro_des - direct thottle control
      *regVARS.ctrl.roll_PID, regVARS.ctrl.pitch_PID, regVARS.ctrl.yaw_PID - stabilized axis variables
      *regVARS.nml_des_state.roll_passthru, regVARS.nml_des_state.pitch_passthru, regVARS.nml_des_state.yaw_passthru - direct unstabilized command passthrough
      *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
      */
      
      //Quad mixing - EXAMPLE
      regVARS.mixer.m1_command_scaled = regVARS.nml_des_state.thro_des - regVARS.ctrl.pitch_PID + regVARS.ctrl.roll_PID + regVARS.ctrl.yaw_PID; //Front Left
      regVARS.mixer.m2_command_scaled = regVARS.nml_des_state.thro_des - regVARS.ctrl.pitch_PID - regVARS.ctrl.roll_PID - regVARS.ctrl.yaw_PID; //Front Right
      regVARS.mixer.m3_command_scaled = regVARS.nml_des_state.thro_des + regVARS.ctrl.pitch_PID - regVARS.ctrl.roll_PID + regVARS.ctrl.yaw_PID; //Back Right
      regVARS.mixer.m4_command_scaled = regVARS.nml_des_state.thro_des + regVARS.ctrl.pitch_PID + regVARS.ctrl.roll_PID - regVARS.ctrl.yaw_PID; //Back Left
      regVARS.mixer.m5_command_scaled = 0;
      regVARS.mixer.m6_command_scaled = 0;

      //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
      regVARS.mixer.s1_command_scaled = 0;
      regVARS.mixer.s2_command_scaled = 0;
      regVARS.mixer.s3_command_scaled = 0;
      regVARS.mixer.s4_command_scaled = 0;
      regVARS.mixer.s5_command_scaled = 0;
      regVARS.mixer.s6_command_scaled = 0;
      regVARS.mixer.s7_command_scaled = 0;
    
    }

    void controlANGLE() {

      //DESCRIPTION: Computes control commands based on state error (angle)
      /*
      * Basic PID control to stablize on angle setpoint based on desired states regVARS.nml_des_state.roll_des, regVARS.nml_des_state.pitch_des, and regVARS.nml_des_state.yaw_des computed in 
      * readradio.getDesState(). Error is simply the desired state minus the actual state (ex. regVARS.nml_des_state.roll_des - regVARS.IMU.roll_IMU). Two safety features
      * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
      * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
      * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
      * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
      * terms will always start from 0 on takeoff. This function updates the variables regVARS.ctrl.roll_PID, regVARS.ctrl.pitch_PID, and regVARS.ctrl.yaw_PID which
      * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in ctrlfuncs.controlMixer().
      */
      
      //Roll
      regVARS.ctrl.error_roll = regVARS.nml_des_state.roll_des - regVARS.IMU.roll_IMU;
      regVARS.ctrl.integral_roll = regVARS.ctrl.integral_roll_prev + regVARS.ctrl.error_roll*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_roll = 0;
      }
      regVARS.ctrl.integral_roll = constrain(regVARS.ctrl.integral_roll, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_roll = regVARS.IMU.GyroX;
      regVARS.ctrl.roll_PID = 0.01*(usrVARS.ctrl_params.Kp_roll_angle*regVARS.ctrl.error_roll + usrVARS.ctrl_params.Ki_roll_angle*regVARS.ctrl.integral_roll - usrVARS.ctrl_params.Kd_roll_angle*regVARS.ctrl.derivative_roll); //Scaled by .01 to bring within -1 to 1 range

      //Pitch
      regVARS.ctrl.error_pitch = regVARS.nml_des_state.pitch_des - regVARS.IMU.pitch_IMU;
      regVARS.ctrl.integral_pitch = regVARS.ctrl.integral_pitch_prev + regVARS.ctrl.error_pitch*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_pitch = 0;
      }
      regVARS.ctrl.integral_pitch = constrain(regVARS.ctrl.integral_pitch, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_pitch = regVARS.IMU.GyroY;
      regVARS.ctrl.pitch_PID = .01*(usrVARS.ctrl_params.Kp_pitch_angle*regVARS.ctrl.error_pitch + usrVARS.ctrl_params.Ki_pitch_angle*regVARS.ctrl.integral_pitch - usrVARS.ctrl_params.Kd_pitch_angle*regVARS.ctrl.derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

      //Yaw, stablize on rate from regVARS.IMU.GyroZ
      regVARS.ctrl.error_yaw = regVARS.nml_des_state.yaw_des - regVARS.IMU.GyroZ;
      regVARS.ctrl.integral_yaw = regVARS.ctrl.integral_yaw_prev + regVARS.ctrl.error_yaw*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_yaw = 0;
      }
      regVARS.ctrl.integral_yaw = constrain(regVARS.ctrl.integral_yaw, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_yaw = (regVARS.ctrl.error_yaw - regVARS.ctrl.error_yaw_prev)/dt; 
      regVARS.ctrl.yaw_PID = .01*(usrVARS.ctrl_params.Kp_yaw*regVARS.ctrl.error_yaw + usrVARS.ctrl_params.Ki_yaw*regVARS.ctrl.integral_yaw + usrVARS.ctrl_params.Kd_yaw*regVARS.ctrl.derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

      //Update roll variables
      regVARS.ctrl.integral_roll_prev = regVARS.ctrl.integral_roll;
      //Update pitch variables
      regVARS.ctrl.integral_pitch_prev = regVARS.ctrl.integral_pitch;
      //Update yaw variables
      regVARS.ctrl.error_yaw_prev = regVARS.ctrl.error_yaw;
      regVARS.ctrl.integral_yaw_prev = regVARS.ctrl.integral_yaw;
    }

    void controlANGLE2() {
      //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
      /*
      * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
      * See the documentation for tuning this regVARS.ctrl.
      */
      //Outer loop - PID on angle
      float roll_des_ol, pitch_des_ol;
      
      //Roll
      regVARS.ctrl.error_roll = regVARS.nml_des_state.roll_des - regVARS.IMU.roll_IMU;
      regVARS.ctrl.integral_roll_ol = regVARS.ctrl.integral_roll_prev_ol + regVARS.ctrl.error_roll*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_roll_ol = 0;
      }
      regVARS.ctrl.integral_roll_ol = constrain(regVARS.ctrl.integral_roll_ol, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_roll = (regVARS.IMU.roll_IMU - regVARS.IMU.roll_IMU_prev)/dt; 
      roll_des_ol = usrVARS.ctrl_params.Kp_roll_angle*regVARS.ctrl.error_roll + usrVARS.ctrl_params.Ki_roll_angle*regVARS.ctrl.integral_roll_ol;// - usrVARS.ctrl_params.Kd_roll_angle*regVARS.ctrl.derivative_roll;

      //Pitch
      regVARS.ctrl.error_pitch = regVARS.nml_des_state.pitch_des - regVARS.IMU.pitch_IMU;
      regVARS.ctrl.integral_pitch_ol = regVARS.ctrl.integral_pitch_prev_ol + regVARS.ctrl.error_pitch*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_pitch_ol = 0;
      }
      regVARS.ctrl.integral_pitch_ol = constrain(regVARS.ctrl.integral_pitch_ol, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_pitch = (regVARS.IMU.pitch_IMU - regVARS.IMU.pitch_IMU_prev)/dt;
      pitch_des_ol = usrVARS.ctrl_params.Kp_pitch_angle*regVARS.ctrl.error_pitch + usrVARS.ctrl_params.Ki_pitch_angle*regVARS.ctrl.integral_pitch_ol;// - usrVARS.ctrl_params.Kd_pitch_angle*regVARS.ctrl.derivative_pitch;

      //Apply loop gain, constrain, and LP filter for artificial damping
      float Kl = 30.0;
      roll_des_ol = Kl*roll_des_ol;
      pitch_des_ol = Kl*pitch_des_ol;
      roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
      pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
      roll_des_ol = (1.0 - usrVARS.ctrl_params.B_loop_roll)*regVARS.ctrl.roll_des_prev + usrVARS.ctrl_params.B_loop_roll*roll_des_ol;
      pitch_des_ol = (1.0 - usrVARS.ctrl_params.B_loop_pitch)*regVARS.ctrl.pitch_des_prev + usrVARS.ctrl_params.B_loop_pitch*pitch_des_ol;

      //Inner loop - PID on rate
      //Roll
      regVARS.ctrl.error_roll = roll_des_ol - regVARS.IMU.GyroX;
      regVARS.ctrl.integral_roll_il = regVARS.ctrl.integral_roll_prev_il + regVARS.ctrl.error_roll*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_roll_il = 0;
      }
      regVARS.ctrl.integral_roll_il = constrain(regVARS.ctrl.integral_roll_il, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_roll = (regVARS.ctrl.error_roll - regVARS.ctrl.error_roll_prev)/dt; 
      regVARS.ctrl.roll_PID = .01*(usrVARS.ctrl_params.Kp_roll_rate*regVARS.ctrl.error_roll + usrVARS.ctrl_params.Ki_roll_rate*regVARS.ctrl.integral_roll_il + usrVARS.ctrl_params.Kd_roll_rate*regVARS.ctrl.derivative_roll); //Scaled by .01 to bring within -1 to 1 range

      //Pitch
      regVARS.ctrl.error_pitch = pitch_des_ol - regVARS.IMU.GyroY;
      regVARS.ctrl.integral_pitch_il = regVARS.ctrl.integral_pitch_prev_il + regVARS.ctrl.error_pitch*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_pitch_il = 0;
      }
      regVARS.ctrl.integral_pitch_il = constrain(regVARS.ctrl.integral_pitch_il, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_pitch = (regVARS.ctrl.error_pitch - regVARS.ctrl.error_pitch_prev)/dt; 
      regVARS.ctrl.pitch_PID = .01*(usrVARS.ctrl_params.Kp_pitch_rate*regVARS.ctrl.error_pitch + usrVARS.ctrl_params.Ki_pitch_rate*regVARS.ctrl.integral_pitch_il + usrVARS.ctrl_params.Kd_pitch_rate*regVARS.ctrl.derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
      
      //Yaw
      regVARS.ctrl.error_yaw = regVARS.nml_des_state.yaw_des - regVARS.IMU.GyroZ;
      regVARS.ctrl.integral_yaw = regVARS.ctrl.integral_yaw_prev + regVARS.ctrl.error_yaw*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_yaw = 0;
      }
      regVARS.ctrl.integral_yaw = constrain(regVARS.ctrl.integral_yaw, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_yaw = (regVARS.ctrl.error_yaw - regVARS.ctrl.error_yaw_prev)/dt; 
      regVARS.ctrl.yaw_PID = .01*(usrVARS.ctrl_params.Kp_yaw*regVARS.ctrl.error_yaw + usrVARS.ctrl_params.Ki_yaw*regVARS.ctrl.integral_yaw + usrVARS.ctrl_params.Kd_yaw*regVARS.ctrl.derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
      
      //Update roll variables
      regVARS.ctrl.integral_roll_prev_ol = regVARS.ctrl.integral_roll_ol;
      regVARS.ctrl.integral_roll_prev_il = regVARS.ctrl.integral_roll_il;
      regVARS.ctrl.error_roll_prev = regVARS.ctrl.error_roll;
      regVARS.IMU.roll_IMU_prev = regVARS.IMU.roll_IMU;
      regVARS.ctrl.roll_des_prev = roll_des_ol;
      //Update pitch variables
      regVARS.ctrl.integral_pitch_prev_ol = regVARS.ctrl.integral_pitch_ol;
      regVARS.ctrl.integral_pitch_prev_il = regVARS.ctrl.integral_pitch_il;
      regVARS.ctrl.error_pitch_prev = regVARS.ctrl.error_pitch;
      regVARS.IMU.pitch_IMU_prev = regVARS.IMU.pitch_IMU;
      regVARS.ctrl.pitch_des_prev = pitch_des_ol;
      //Update yaw variables
      regVARS.ctrl.error_yaw_prev = regVARS.ctrl.error_yaw;
      regVARS.ctrl.integral_yaw_prev = regVARS.ctrl.integral_yaw;

    }

    void controlRATE() {
      //DESCRIPTION: Computes control commands based on state error (rate)
      /*
      * See explanation for ctrlfuncs.controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
      */
      //Roll
      regVARS.ctrl.error_roll = regVARS.nml_des_state.roll_des - regVARS.IMU.GyroX;
      regVARS.ctrl.integral_roll = regVARS.ctrl.integral_roll_prev + regVARS.ctrl.error_roll*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_roll = 0;
      }
      regVARS.ctrl.integral_roll = constrain(regVARS.ctrl.integral_roll, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_roll = (regVARS.ctrl.error_roll - regVARS.ctrl.error_roll_prev)/dt; 
      regVARS.ctrl.roll_PID = .01*(usrVARS.ctrl_params.Kp_roll_rate*regVARS.ctrl.error_roll + usrVARS.ctrl_params.Ki_roll_rate*regVARS.ctrl.integral_roll + usrVARS.ctrl_params.Kd_roll_rate*regVARS.ctrl.derivative_roll); //Scaled by .01 to bring within -1 to 1 range

      //Pitch
      regVARS.ctrl.error_pitch = regVARS.nml_des_state.pitch_des - regVARS.IMU.GyroY;
      regVARS.ctrl.integral_pitch = regVARS.ctrl.integral_pitch_prev + regVARS.ctrl.error_pitch*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_pitch = 0;
      }
      regVARS.ctrl.integral_pitch = constrain(regVARS.ctrl.integral_pitch, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_pitch = (regVARS.ctrl.error_pitch - regVARS.ctrl.error_pitch_prev)/dt; 
      regVARS.ctrl.pitch_PID = .01*(usrVARS.ctrl_params.Kp_pitch_rate*regVARS.ctrl.error_pitch + usrVARS.ctrl_params.Ki_pitch_rate*regVARS.ctrl.integral_pitch + usrVARS.ctrl_params.Kd_pitch_rate*regVARS.ctrl.derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

      //Yaw, stablize on rate from regVARS.IMU.GyroZ
      regVARS.ctrl.error_yaw = regVARS.nml_des_state.yaw_des - regVARS.IMU.GyroZ;
      regVARS.ctrl.integral_yaw = regVARS.ctrl.integral_yaw_prev + regVARS.ctrl.error_yaw*dt;
      if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        regVARS.ctrl.integral_yaw = 0;
      }
      regVARS.ctrl.integral_yaw = constrain(regVARS.ctrl.integral_yaw, -usrVARS.ctrl_params.i_limit, usrVARS.ctrl_params.i_limit); //Saturate integrator to prevent unsafe buildup
      regVARS.ctrl.derivative_yaw = (regVARS.ctrl.error_yaw - regVARS.ctrl.error_yaw_prev)/dt; 
      regVARS.ctrl.yaw_PID = .01*(usrVARS.ctrl_params.Kp_yaw*regVARS.ctrl.error_yaw + usrVARS.ctrl_params.Ki_yaw*regVARS.ctrl.integral_yaw + usrVARS.ctrl_params.Kd_yaw*regVARS.ctrl.derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

      //Update roll variables
      regVARS.ctrl.error_roll_prev = regVARS.ctrl.error_roll;
      regVARS.ctrl.integral_roll_prev = regVARS.ctrl.integral_roll;
      regVARS.IMU.GyroX_prev = regVARS.IMU.GyroX;
      //Update pitch variables
      regVARS.ctrl.error_pitch_prev = regVARS.ctrl.error_pitch;
      regVARS.ctrl.integral_pitch_prev = regVARS.ctrl.integral_pitch;
      regVARS.IMU.GyroY_prev = regVARS.IMU.GyroY;
      //Update yaw variables
      regVARS.ctrl.error_yaw_prev = regVARS.ctrl.error_yaw;
      regVARS.ctrl.integral_yaw_prev = regVARS.ctrl.integral_yaw;
    }

};

class MotorFuncs {
  
  ReadRadio readradio;

  public:
  
    void commandMotors() {
      //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
      /*
      * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
      * sent are mX_command_PWM, computed in readradio.scaleCommands(). This may be replaced by something more efficient in the future.
      */
      int wentLow = 0;
      int pulseStart, timer;
      int flagM1 = 0;
      int flagM2 = 0;
      int flagM3 = 0;
      int flagM4 = 0;
      int flagM5 = 0;
      int flagM6 = 0;
      
      //Write all motor pins high
      digitalWrite(pinout.m1Pin, HIGH);
      digitalWrite(pinout.m2Pin, HIGH);
      digitalWrite(pinout.m3Pin, HIGH);
      digitalWrite(pinout.m4Pin, HIGH);
      digitalWrite(pinout.m5Pin, HIGH);
      digitalWrite(pinout.m6Pin, HIGH);
      pulseStart = micros();

      //Write each motor pin low as correct pulse length is reached
      while (wentLow < 6 ) { //Keep going until final (6th) pulse is finished, then done
        timer = micros();
        if ((regVARS.mixer.m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
          digitalWrite(pinout.m1Pin, LOW);
          wentLow = wentLow + 1;
          flagM1 = 1;
        }
        if ((regVARS.mixer.m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
          digitalWrite(pinout.m2Pin, LOW);
          wentLow = wentLow + 1;
          flagM2 = 1;
        }
        if ((regVARS.mixer.m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
          digitalWrite(pinout.m3Pin, LOW);
          wentLow = wentLow + 1;
          flagM3 = 1;
        }
        if ((regVARS.mixer.m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
          digitalWrite(pinout.m4Pin, LOW);
          wentLow = wentLow + 1;
          flagM4 = 1;
        } 
        if ((regVARS.mixer.m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
          digitalWrite(pinout.m5Pin, LOW);
          wentLow = wentLow + 1;
          flagM5 = 1;
        } 
        if ((regVARS.mixer.m6_command_PWM <= timer - pulseStart) && (flagM6==0)) {
          digitalWrite(pinout.m6Pin, LOW);
          wentLow = wentLow + 1;
          flagM6 = 1;
        } 
      }
    }

    void armMotors() {
      //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
      /*  
      *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
      *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
      *  for other processes that sometimes prevent motors from arming.
      */
      for (int i = 0; i <= 50; i++) {
        commandMotors();
        delay(2);
      }
    }

    void switchRollYaw(int reverseRoll, int reverseYaw) {
      //DESCRIPTION: Switches regVARS.nml_des_state.roll_des and regVARS.nml_des_state.yaw_des variables for tailsitter-type configurations
      /*
      * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
      * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not 
      * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis. 
      * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the 
      * IMU tilted 90 degrees from default level).
      */
      float switch_holder;

      switch_holder = regVARS.nml_des_state.yaw_des;
      regVARS.nml_des_state.yaw_des = reverseYaw*regVARS.nml_des_state.roll_des;
      regVARS.nml_des_state.roll_des = reverseRoll*switch_holder;
    }

};

class IMUFuncs {
  
  public:

    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
      HelperFuncs helper;

      //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
      /*
      * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
      * available (for example when using the recommended MPU6050 IMU for the default setup).
      */
      float recipNorm;
      float s0, s1, s2, s3;
      float qDot1, qDot2, qDot3, qDot4;
      float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

      //Convert gyroscope degrees/sec to radians/sec
      gx *= 0.0174533f;
      gy *= 0.0174533f;
      gz *= 0.0174533f;

      //Rate of change of quaternion from gyroscope
      qDot1 = 0.5f * (-regVARS.IMU.q1 * gx - regVARS.IMU.q2 * gy - regVARS.IMU.q3 * gz);
      qDot2 = 0.5f * (regVARS.IMU.q0 * gx + regVARS.IMU.q2 * gz - regVARS.IMU.q3 * gy);
      qDot3 = 0.5f * (regVARS.IMU.q0 * gy - regVARS.IMU.q1 * gz + regVARS.IMU.q3 * gx);
      qDot4 = 0.5f * (regVARS.IMU.q0 * gz + regVARS.IMU.q1 * gy - regVARS.IMU.q2 * gx);

      //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
      if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        //Normalise accelerometer measurement
        recipNorm = helper.invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * regVARS.IMU.q0;
        _2q1 = 2.0f * regVARS.IMU.q1;
        _2q2 = 2.0f * regVARS.IMU.q2;
        _2q3 = 2.0f * regVARS.IMU.q3;
        _4q0 = 4.0f * regVARS.IMU.q0;
        _4q1 = 4.0f * regVARS.IMU.q1;
        _4q2 = 4.0f * regVARS.IMU.q2;
        _8q1 = 8.0f * regVARS.IMU.q1;
        _8q2 = 8.0f * regVARS.IMU.q2;
        q0q0 = regVARS.IMU.q0 * regVARS.IMU.q0;
        q1q1 = regVARS.IMU.q1 * regVARS.IMU.q1;
        q2q2 = regVARS.IMU.q2 * regVARS.IMU.q2;
        q3q3 = regVARS.IMU.q3 * regVARS.IMU.q3;

        //Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * regVARS.IMU.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * regVARS.IMU.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * regVARS.IMU.q3 - _2q1 * ax + 4.0f * q2q2 * regVARS.IMU.q3 - _2q2 * ay;
        recipNorm = helper.invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= usrVARS.filter_struct.B_madgwick * s0;
        qDot2 -= usrVARS.filter_struct.B_madgwick * s1;
        qDot3 -= usrVARS.filter_struct.B_madgwick * s2;
        qDot4 -= usrVARS.filter_struct.B_madgwick * s3;
      }

      //Integrate rate of change of quaternion to yield quaternion
      regVARS.IMU.q0 += qDot1 * invSampleFreq;
      regVARS.IMU.q1 += qDot2 * invSampleFreq;
      regVARS.IMU.q2 += qDot3 * invSampleFreq;
      regVARS.IMU.q3 += qDot4 * invSampleFreq;

      //Normalise quaternion
      recipNorm = helper.invSqrt(regVARS.IMU.q0 * regVARS.IMU.q0 + regVARS.IMU.q1 * regVARS.IMU.q1 + regVARS.IMU.q2 * regVARS.IMU.q2 + regVARS.IMU.q3 * regVARS.IMU.q3);
      regVARS.IMU.q0 *= recipNorm;
      regVARS.IMU.q1 *= recipNorm;
      regVARS.IMU.q2 *= recipNorm;
      regVARS.IMU.q3 *= recipNorm;

      //Compute angles
      regVARS.IMU.roll_IMU = atan2(regVARS.IMU.q0*regVARS.IMU.q1 + regVARS.IMU.q2*regVARS.IMU.q3, 0.5f - regVARS.IMU.q1*regVARS.IMU.q1 - regVARS.IMU.q2*regVARS.IMU.q2)*57.29577951; //degrees
      regVARS.IMU.pitch_IMU = -asin(-2.0f * (regVARS.IMU.q1*regVARS.IMU.q3 - regVARS.IMU.q0*regVARS.IMU.q2))*57.29577951; //degrees
      regVARS.IMU.yaw_IMU = -atan2(regVARS.IMU.q1*regVARS.IMU.q2 + regVARS.IMU.q0*regVARS.IMU.q3, 0.5f - regVARS.IMU.q2*regVARS.IMU.q2 - regVARS.IMU.q3*regVARS.IMU.q3)*57.29577951; //degrees
    }

    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
      HelperFuncs helper;
      IMUFuncs imuf;
      //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
      /*
      * This function fuses the accelerometer gyro, and magnetometer readings regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ, regVARS.IMU.GyroX, regVARS.IMU.GyroY, regVARS.IMU.GyroZ, regVARS.IMU.MagX, regVARS.IMU.MagY, and regVARS.IMU.MagZ for attitude estimation.
      * Don't worry about the math. There is a tunable parameter usrVARS.filter_struct.B_madgwick in the user specified variable section which basically
      * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
      * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the regVARS.IMU.roll_IMU,
      * regVARS.IMU.pitch_IMU, and regVARS.IMU.yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls imuf.Madgwick6DOF() instead.
      */
      float recipNorm;
      float s0, s1, s2, s3;
      float qDot1, qDot2, qDot3, qDot4;
      float hx, hy;
      float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;


      //use 6DOF algorithm if MPU6050 is being used
      #if defined USE_MPU6050_I2C 
        imuf.Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
        return;
      #endif
      
      //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
      if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        imuf.Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
        return;
      }

      //Convert gyroscope degrees/sec to radians/sec
      gx *= 0.0174533f;
      gy *= 0.0174533f;
      gz *= 0.0174533f;

      //Rate of change of quaternion from gyroscope
      qDot1 = 0.5f * (-regVARS.IMU.q1 * gx - regVARS.IMU.q2 * gy - regVARS.IMU.q3 * gz);
      qDot2 = 0.5f * (regVARS.IMU.q0 * gx + regVARS.IMU.q2 * gz - regVARS.IMU.q3 * gy);
      qDot3 = 0.5f * (regVARS.IMU.q0 * gy - regVARS.IMU.q1 * gz + regVARS.IMU.q3 * gx);
      qDot4 = 0.5f * (regVARS.IMU.q0 * gz + regVARS.IMU.q1 * gy - regVARS.IMU.q2 * gx);

      //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
      if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        //Normalise accelerometer measurement
        recipNorm = helper.invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Normalise magnetometer measurement
        recipNorm = helper.invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * regVARS.IMU.q0 * mx;
        _2q0my = 2.0f * regVARS.IMU.q0 * my;
        _2q0mz = 2.0f * regVARS.IMU.q0 * mz;
        _2q1mx = 2.0f * regVARS.IMU.q1 * mx;
        _2q0 = 2.0f * regVARS.IMU.q0;
        _2q1 = 2.0f * regVARS.IMU.q1;
        _2q2 = 2.0f * regVARS.IMU.q2;
        _2q3 = 2.0f * regVARS.IMU.q3;
        _2q0q2 = 2.0f * regVARS.IMU.q0 * regVARS.IMU.q2;
        _2q2q3 = 2.0f * regVARS.IMU.q2 * regVARS.IMU.q3;
        q0q0 = regVARS.IMU.q0 * regVARS.IMU.q0;
        q0q1 = regVARS.IMU.q0 * regVARS.IMU.q1;
        q0q2 = regVARS.IMU.q0 * regVARS.IMU.q2;
        q0q3 = regVARS.IMU.q0 * regVARS.IMU.q3;
        q1q1 = regVARS.IMU.q1 * regVARS.IMU.q1;
        q1q2 = regVARS.IMU.q1 * regVARS.IMU.q2;
        q1q3 = regVARS.IMU.q1 * regVARS.IMU.q3;
        q2q2 = regVARS.IMU.q2 * regVARS.IMU.q2;
        q2q3 = regVARS.IMU.q2 * regVARS.IMU.q3;
        q3q3 = regVARS.IMU.q3 * regVARS.IMU.q3;

        //Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * regVARS.IMU.q3 + _2q0mz * regVARS.IMU.q2 + mx * q1q1 + _2q1 * my * regVARS.IMU.q2 + _2q1 * mz * regVARS.IMU.q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * regVARS.IMU.q3 + my * q0q0 - _2q0mz * regVARS.IMU.q1 + _2q1mx * regVARS.IMU.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * regVARS.IMU.q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * regVARS.IMU.q2 + _2q0my * regVARS.IMU.q1 + mz * q0q0 + _2q1mx * regVARS.IMU.q3 - mz * q1q1 + _2q2 * my * regVARS.IMU.q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        //Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * regVARS.IMU.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * regVARS.IMU.q3 + _2bz * regVARS.IMU.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * regVARS.IMU.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * regVARS.IMU.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * regVARS.IMU.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * regVARS.IMU.q2 + _2bz * regVARS.IMU.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * regVARS.IMU.q3 - _4bz * regVARS.IMU.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * regVARS.IMU.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * regVARS.IMU.q2 - _2bz * regVARS.IMU.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * regVARS.IMU.q1 + _2bz * regVARS.IMU.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * regVARS.IMU.q0 - _4bz * regVARS.IMU.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * regVARS.IMU.q3 + _2bz * regVARS.IMU.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * regVARS.IMU.q0 + _2bz * regVARS.IMU.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * regVARS.IMU.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = helper.invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= usrVARS.filter_struct.B_madgwick * s0;
        qDot2 -= usrVARS.filter_struct.B_madgwick * s1;
        qDot3 -= usrVARS.filter_struct.B_madgwick * s2;
        qDot4 -= usrVARS.filter_struct.B_madgwick * s3;
      }

      //Integrate rate of change of quaternion to yield quaternion
      regVARS.IMU.q0 += qDot1 * invSampleFreq;
      regVARS.IMU.q1 += qDot2 * invSampleFreq;
      regVARS.IMU.q2 += qDot3 * invSampleFreq;
      regVARS.IMU.q3 += qDot4 * invSampleFreq;

      //Normalize quaternion
      recipNorm = helper.invSqrt(regVARS.IMU.q0 * regVARS.IMU.q0 + regVARS.IMU.q1 * regVARS.IMU.q1 + regVARS.IMU.q2 * regVARS.IMU.q2 + regVARS.IMU.q3 * regVARS.IMU.q3);
      regVARS.IMU.q0 *= recipNorm;
      regVARS.IMU.q1 *= recipNorm;
      regVARS.IMU.q2 *= recipNorm;
      regVARS.IMU.q3 *= recipNorm;
      
      //compute angles - NWU
      regVARS.IMU.roll_IMU = atan2(regVARS.IMU.q0*regVARS.IMU.q1 + regVARS.IMU.q2*regVARS.IMU.q3, 0.5f - regVARS.IMU.q1*regVARS.IMU.q1 - regVARS.IMU.q2*regVARS.IMU.q2)*57.29577951; //degrees
      regVARS.IMU.pitch_IMU = -asin(-2.0f * (regVARS.IMU.q1*regVARS.IMU.q3 - regVARS.IMU.q0*regVARS.IMU.q2))*57.29577951; //degrees
      regVARS.IMU.yaw_IMU = -atan2(regVARS.IMU.q1*regVARS.IMU.q2 + regVARS.IMU.q0*regVARS.IMU.q3, 0.5f - regVARS.IMU.q2*regVARS.IMU.q2 - regVARS.IMU.q3*regVARS.IMU.q3)*57.29577951; //degrees
    }

    void IMUinit() {
      //DESCRIPTION: Initialize IMU
      /*
      * Don't worry about how this works.
      */
      #if defined USE_MPU6050_I2C
        Wire.begin();
        Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
        
        mpu6050.initialize();
        
        if (mpu6050.testConnection() == false) {
          Serial.println("MPU6050 initialization unsuccessful");
          Serial.println("Check MPU6050 wiring or try cycling power");
          while(1) {}
        }

        //From the reset state all registers should be 0x00, so we should be at
        //max sample rate with digital low pass filter(s) off.  All we need to
        //do is set the desired fullscale ranges
        mpu6050.setFullScaleGyroRange(GYRO_SCALE);
        mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
        
      #elif defined USE_MPU9250_SPI
        int status = mpu9250.begin();    

        if (status < 0) {
          Serial.println("MPU9250 initialization unsuccessful");
          Serial.println("Check MPU9250 wiring or try cycling power");
          Serial.print("Status: ");
          Serial.println(status);
          while(1) {}
        }

        //From the reset state all registers should be 0x00, so we should be at
        //max sample rate with digital low pass filter(s) off.  All we need to
        //do is set the desired fullscale ranges
        mpu9250.setGyroRange(GYRO_SCALE);
        mpu9250.setAccelRange(ACCEL_SCALE);
        mpu9250.setMagCalX(usrVARS.magno_cal_struct.MagErrorX, usrVARS.magno_cal_struct.MagScaleX);
        mpu9250.setMagCalY(usrVARS.magno_cal_struct.MagErrorY, usrVARS.magno_cal_struct.MagScaleY);
        mpu9250.setMagCalZ(usrVARS.magno_cal_struct.MagErrorZ, usrVARS.magno_cal_struct.MagScaleZ);
        mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
      #endif
    }

    void getIMUdata() {
      //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
      /*
      * Reads accelerometer, gyro, and magnetometer data from IMU as regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ, regVARS.IMU.GyroX, regVARS.IMU.GyroY, regVARS.IMU.GyroZ, regVARS.IMU.MagX, regVARS.IMU.MagY, regVARS.IMU.MagZ. 
      * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
      * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
      * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
      * the readings. The filter parameters B_gyro and usrVARS.filter_struct.B_accel are set to be good for a 2kHz loop rate. Finally,
      * the constant errors found in imuf.calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
      */
     
      int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

      #if defined USE_MPU6050_I2C
        mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
      #elif defined USE_MPU9250_SPI
        mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
      #endif

    //Accelerometer
      regVARS.IMU.AccX = AcX / ACCEL_SCALE_FACTOR; //G's
      regVARS.IMU.AccY = AcY / ACCEL_SCALE_FACTOR;
      regVARS.IMU.AccZ = AcZ / ACCEL_SCALE_FACTOR;
      //Correct the outputs with the calculated error values
      regVARS.IMU.AccX = regVARS.IMU.AccX - usrVARS.imu_cal_struct.AccErrorX;
      regVARS.IMU.AccY = regVARS.IMU.AccY - usrVARS.imu_cal_struct.AccErrorY;
      regVARS.IMU.AccZ = regVARS.IMU.AccZ - usrVARS.imu_cal_struct.AccErrorZ;
      //LP filter accelerometer data
      regVARS.IMU.AccX = (1.0 - usrVARS.filter_struct.B_accel)*regVARS.IMU.AccX_prev + usrVARS.filter_struct.B_accel*regVARS.IMU.AccX;
      regVARS.IMU.AccY = (1.0 - usrVARS.filter_struct.B_accel)*regVARS.IMU.AccY_prev + usrVARS.filter_struct.B_accel*regVARS.IMU.AccY;
      regVARS.IMU.AccZ = (1.0 - usrVARS.filter_struct.B_accel)*regVARS.IMU.AccZ_prev + usrVARS.filter_struct.B_accel*regVARS.IMU.AccZ;
      regVARS.IMU.AccX_prev = regVARS.IMU.AccX;
      regVARS.IMU.AccY_prev = regVARS.IMU.AccY;
      regVARS.IMU.AccZ_prev = regVARS.IMU.AccZ;

      //Gyro
      regVARS.IMU.GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
      regVARS.IMU.GyroY = GyY / GYRO_SCALE_FACTOR;
      regVARS.IMU.GyroZ = GyZ / GYRO_SCALE_FACTOR;
      //Correct the outputs with the calculated error values
      regVARS.IMU.GyroX = regVARS.IMU.GyroX - usrVARS.imu_cal_struct.GyroErrorX;
      regVARS.IMU.GyroY = regVARS.IMU.GyroY - usrVARS.imu_cal_struct.GyroErrorY;
      regVARS.IMU.GyroZ = regVARS.IMU.GyroZ - usrVARS.imu_cal_struct.GyroErrorZ;
      //LP filter gyro data
      regVARS.IMU.GyroX = (1.0 - usrVARS.filter_struct.B_gyro)*regVARS.IMU.GyroX_prev + usrVARS.filter_struct.B_gyro*regVARS.IMU.GyroX;
      regVARS.IMU.GyroY = (1.0 - usrVARS.filter_struct.B_gyro)*regVARS.IMU.GyroY_prev + usrVARS.filter_struct.B_gyro*regVARS.IMU.GyroY;
      regVARS.IMU.GyroZ = (1.0 - usrVARS.filter_struct.B_gyro)*regVARS.IMU.GyroZ_prev + usrVARS.filter_struct.B_gyro*regVARS.IMU.GyroZ;
      regVARS.IMU.GyroX_prev = regVARS.IMU.GyroX;
      regVARS.IMU.GyroY_prev = regVARS.IMU.GyroY;
      regVARS.IMU.GyroZ_prev = regVARS.IMU.GyroZ;

      //Magnetometer
      regVARS.IMU.MagX = MgX/6.0; //uT
      regVARS.IMU.MagY = MgY/6.0;
      regVARS.IMU.MagZ = MgZ/6.0;
      //Correct the outputs with the calculated error values
      regVARS.IMU.MagX = (regVARS.IMU.MagX - usrVARS.magno_cal_struct.MagErrorX)*usrVARS.magno_cal_struct.MagScaleX;
      regVARS.IMU.MagY = (regVARS.IMU.MagY - usrVARS.magno_cal_struct.MagErrorY)*usrVARS.magno_cal_struct.MagScaleY;
      regVARS.IMU.MagZ = (regVARS.IMU.MagZ - usrVARS.magno_cal_struct.MagErrorZ)*usrVARS.magno_cal_struct.MagScaleZ;
      //LP filter magnetometer data
      regVARS.IMU.MagX = (1.0 - usrVARS.filter_struct.B_mag)*regVARS.IMU.MagX_prev + usrVARS.filter_struct.B_mag*regVARS.IMU.MagX;
      regVARS.IMU.MagY = (1.0 - usrVARS.filter_struct.B_mag)*regVARS.IMU.MagY_prev + usrVARS.filter_struct.B_mag*regVARS.IMU.MagY;
      regVARS.IMU.MagZ = (1.0 - usrVARS.filter_struct.B_mag)*regVARS.IMU.MagZ_prev + usrVARS.filter_struct.B_mag*regVARS.IMU.MagZ;
      regVARS.IMU.MagX_prev = regVARS.IMU.MagX;
      regVARS.IMU.MagY_prev = regVARS.IMU.MagY;
      regVARS.IMU.MagZ_prev = regVARS.IMU.MagZ;
    }

    void calculate_IMU_error() {
      //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
      /*
      * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
      * accelerometer values regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ, regVARS.IMU.GyroX, regVARS.IMU.GyroY, regVARS.IMU.GyroZ in imuf.getIMUdata(). This eliminates drift in the
      * measurement. 
      */
      int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
      usrVARS.imu_cal_struct.AccErrorX = 0.0;
      usrVARS.imu_cal_struct.AccErrorY = 0.0;
      usrVARS.imu_cal_struct.AccErrorZ = 0.0;
      usrVARS.imu_cal_struct.GyroErrorX = 0.0;
      usrVARS.imu_cal_struct.GyroErrorY= 0.0;
      usrVARS.imu_cal_struct.GyroErrorZ = 0.0;
      
      //Read IMU values 12000 times
      int c = 0;
      while (c < 12000) {
        #if defined USE_MPU6050_I2C
          mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
        #elif defined USE_MPU9250_SPI
          mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
        #endif
        
        regVARS.IMU.AccX  = AcX / ACCEL_SCALE_FACTOR;
        regVARS.IMU.AccY  = AcY / ACCEL_SCALE_FACTOR;
        regVARS.IMU.AccZ  = AcZ / ACCEL_SCALE_FACTOR;
        regVARS.IMU.GyroX = GyX / GYRO_SCALE_FACTOR;
        regVARS.IMU.GyroY = GyY / GYRO_SCALE_FACTOR;
        regVARS.IMU.GyroZ = GyZ / GYRO_SCALE_FACTOR;
        
        //Sum all readings
        usrVARS.imu_cal_struct.AccErrorX  = usrVARS.imu_cal_struct.AccErrorX + regVARS.IMU.AccX;
        usrVARS.imu_cal_struct.AccErrorY  = usrVARS.imu_cal_struct.AccErrorY + regVARS.IMU.AccY;
        usrVARS.imu_cal_struct.AccErrorZ  = usrVARS.imu_cal_struct.AccErrorZ + regVARS.IMU.AccZ;
        usrVARS.imu_cal_struct.GyroErrorX = usrVARS.imu_cal_struct.GyroErrorX + regVARS.IMU.GyroX;
        usrVARS.imu_cal_struct.GyroErrorY = usrVARS.imu_cal_struct.GyroErrorY + regVARS.IMU.GyroY;
        usrVARS.imu_cal_struct.GyroErrorZ = usrVARS.imu_cal_struct.GyroErrorZ + regVARS.IMU.GyroZ;
        c++;
      }
      //Divide the sum by 12000 to get the error value
      usrVARS.imu_cal_struct.AccErrorX  = usrVARS.imu_cal_struct.AccErrorX / c;
      usrVARS.imu_cal_struct.AccErrorY  = usrVARS.imu_cal_struct.AccErrorY / c;
      usrVARS.imu_cal_struct.AccErrorZ  = usrVARS.imu_cal_struct.AccErrorZ / c - 1.0;
      usrVARS.imu_cal_struct.GyroErrorX = usrVARS.imu_cal_struct.GyroErrorX / c;
      usrVARS.imu_cal_struct.GyroErrorY = usrVARS.imu_cal_struct.GyroErrorY / c;
      usrVARS.imu_cal_struct.GyroErrorZ = usrVARS.imu_cal_struct.GyroErrorZ / c;

      Serial.print("float usrVARS.imu_cal_struct.AccErrorX = ");
      Serial.print(usrVARS.imu_cal_struct.AccErrorX);
      Serial.println(";");
      Serial.print("float usrVARS.imu_cal_struct.AccErrorY = ");
      Serial.print(usrVARS.imu_cal_struct.AccErrorY);
      Serial.println(";");
      Serial.print("float usrVARS.imu_cal_struct.AccErrorZ = ");
      Serial.print(usrVARS.imu_cal_struct.AccErrorZ);
      Serial.println(";");
      
      Serial.print("float usrVARS.imu_cal_struct.GyroErrorX = ");
      Serial.print(usrVARS.imu_cal_struct.GyroErrorX);
      Serial.println(";");
      Serial.print("float usrVARS.imu_cal_struct.GyroErrorY = ");
      Serial.print(usrVARS.imu_cal_struct.GyroErrorY);
      Serial.println(";");
      Serial.print("float usrVARS.imu_cal_struct.GyroErrorZ = ");
      Serial.print(usrVARS.imu_cal_struct.GyroErrorZ);
      Serial.println(";");

      Serial.println("Paste these values in user specified variables section and comment out imuf.calculate_IMU_error() in void setup.");
    }

};

class SetupFuncs {

  public:
  
    void loopBlink() {
      //DESCRIPTION: Blink LED on board to indicate main loop is running
      /*
      * It looks cool.
      */
      if (current_time - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
        
        if (blinkAlternate == 1) {
          blinkAlternate = 0;
          blink_delay = 100000;
          }
        else if (blinkAlternate == 0) {
          blinkAlternate = 1;
          blink_delay = 2000000;
          }
      }
    }

    void setupBlink(int numBlinks,int upTime, int downTime) {
      //DESCRIPTION: Simple function to make LED on board blink as desired
      for (int j = 1; j<= numBlinks; j++) {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
      }
    }

    void loopRate(int freq) {
      //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
      /*
      * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
      * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
      * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
      * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
      * and remain above 2kHz, without needing to retune all of our filtering parameters.
      */
      float invFreq = 1.0/freq*1000000.0;
      unsigned long checker = micros();
      
      //Sit in loop until appropriate time has passed
      while (invFreq > (checker - current_time)) {
        checker = micros();
      }
    }

    void calibrateMagnetometer() {
      #if defined USE_MPU9250_SPI 
        float success;
        Serial.println("Beginning magnetometer calibration in");
        Serial.println("3...");
        delay(1000);
        Serial.println("2...");
        delay(1000);
        Serial.println("1...");
        delay(1000);
        Serial.println("Rotate the IMU about all axes until complete.");
        Serial.println(" ");
        success = mpu9250.calibrateMag();
        if(success) {
          Serial.println("Calibration Successful!");
          Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
          Serial.print("float MagErrorX = ");
          Serial.print(mpu9250.getMagBiasX_uT());
          Serial.println(";");
          Serial.print("float MagErrorY = ");
          Serial.print(mpu9250.getMagBiasY_uT());
          Serial.println(";");
          Serial.print("float MagErrorZ = ");
          Serial.print(mpu9250.getMagBiasZ_uT());
          Serial.println(";");
          Serial.print("float MagScaleX = ");
          Serial.print(mpu9250.getMagScaleFactorX());
          Serial.println(";");
          Serial.print("float MagScaleY = ");
          Serial.print(mpu9250.getMagScaleFactorY());
          Serial.println(";");
          Serial.print("float MagScaleZ = ");
          Serial.print(mpu9250.getMagScaleFactorZ());
          Serial.println(";");
          Serial.println(" ");
          Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
        }
        else {
          Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
        }
      
        while(1); //Halt code so it won't enter main loop until this function commented out
      #endif
      Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
      while(1); //Halt code so it won't enter main loop until this function commented out
    }

    void calibrateESCs() {

      IMUFuncs imuf;
      SetupFuncs setup;
      MotorFuncs motorfuncs;
      ReadRadio readradio;

      //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
      /*  
      *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
      *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
      *  uncommented when performing an ESC calibration.
      */

      while (true) {
          
          prev_time = current_time;      
          current_time = micros();      
          dt = (current_time - prev_time)/1000000.0;
        
          digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

          readradio.getCommands(); //Pulls current available radio commands
          readradio.failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
          readradio.getDesState(); //Convert raw commands to normalized values based on saturated control limits
          imuf.getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
          imuf.Madgwick(regVARS.IMU.GyroX, -regVARS.IMU.GyroY, -regVARS.IMU.GyroZ, -regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ, regVARS.IMU.MagY, -regVARS.IMU.MagX, regVARS.IMU.MagZ, dt); //Updates regVARS.IMU.roll_IMU, regVARS.IMU.pitch_IMU, and regVARS.IMU.yaw_IMU (degrees)
          readradio.getDesState(); //Convert raw commands to normalized values based on saturated control limits
          
          regVARS.mixer.m1_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.m2_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.m3_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.m4_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.m5_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.m6_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s1_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s2_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s3_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s4_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s5_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s6_command_scaled = regVARS.nml_des_state.thro_des;
          regVARS.mixer.s7_command_scaled = regVARS.nml_des_state.thro_des;
          readradio.scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
        
          //readradio.throttleCut(); //Directly sets motor commands to low based on state of ch5
          
          servo1.write(regVARS.mixer.s1_command_PWM); 
          servo2.write(regVARS.mixer.s2_command_PWM);
          servo3.write(regVARS.mixer.s3_command_PWM);
          servo4.write(regVARS.mixer.s4_command_PWM);
          servo5.write(regVARS.mixer.s5_command_PWM);
          servo6.write(regVARS.mixer.s6_command_PWM);
          servo7.write(regVARS.mixer.s7_command_PWM);
          motorfuncs.commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
          
          //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
          
          setup.loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
      }
    }

};

class MiscFuncs {
  
  public:
  
    void calibrateAttitude() {
      SetupFuncs setup;
      IMUFuncs imuf;

      //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
      //Assuming vehicle is powered up on level surface!
      /*
      * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
      * to boot. 
      */
      //Warm up IMU and madgwick filter in simulated main loop
      for (int i = 0; i <= 10000; i++) {
        prev_time = current_time;      
        current_time = micros();      
        dt = (current_time - prev_time)/1000000.0; 
        imuf.getIMUdata();
        imuf.Madgwick(regVARS.IMU.GyroX, -regVARS.IMU.GyroY, -regVARS.IMU.GyroZ, -regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ, regVARS.IMU.MagY, -regVARS.IMU.MagX, regVARS.IMU.MagZ, dt);
        setup.loopRate(2000); //do not exceed 2000Hz
        }
    }

};

//========================================================================================================================//
//                                               Headless Classes                                                              //
//========================================================================================================================//
class Headless_Misc {
    public:

        void failsafe() {
            // Stop Motors
        }

        float scaleCommands (int direction, float multiplier) {
        // Scale Values from 0 to 1 to 0 to 255

        float speed;
        return speed;
}
};

class Headless_Sentry {
    
    // sec1 is the front sector
    // sec2 is the right sector
    // sec3 is the rear sector
    // sec4 is the left sector
    
    public:

        float get_sec_avg_and_multiplier(std::vector<float> sector) {
            using std::vector;

            // verify the average of all the readings in a sector
            vector<float> verified_sec;
            float sec_count;
            for (int i = 0; i < sector.size(); i++) {
                if (boundaries(sector[i])) {
                    sec_count += sector[i];
                    // insert failsafe here
                }
            }

            // the average of the sector is used to determine which direction the drone should move
            float sec_avg = sec_count / sector.size();
            
            // the multiplier is the percentage of the max range that the average is
            // this is used to determine how fast the drone should move
            float sec_mul = sec_avg / module_info.max_us_range; 
            
            return sec_avg, sec_mul;
        }

        float swivel(int direction) {
            // direction is either 1 or -1, or 0 for stop
            // create a vector for each sector
            
            std::vector<float> sec1;
            std::vector<float> sec2;
            std::vector<float> sec3;
            std::vector<float> sec4;


            if (direction == 1) {
                for (int i = 0; i < 180; i+=18) {
                    us_s1.write(i);
                    sec1.push_back(us1.Ranging(CM));
                    us_s2.write(i);
                    sec2.push_back(us2.Ranging(CM));
                    us_s3.write(i);
                    sec3.push_back(us3.Ranging(CM));
                    us_s4.write(i);
                    sec4.push_back(us4.Ranging(CM));
                    delay(10);
                }
            } else if (direction == -1) {
                for (int i = 180; i > 0; i-=18) {
                    us_s1.write(i);
                    sec1.push_back(us1.Ranging(CM));
                    us_s2.write(i);
                    sec2.push_back(us2.Ranging(CM));
                    us_s3.write(i);
                    sec3.push_back(us3.Ranging(CM));
                    us_s4.write(i);
                    sec4.push_back(us4.Ranging(CM));
                    delay(10);
                }
            } else {
                // stop
                us_s1.write(90);
                us_s2.write(90);
                us_s3.write(90);
                us_s4.write(90);
            }
            
            float avg1, mul1 = get_sec_avg_and_multiplier(sec1);
            float avg2, mul2 = get_sec_avg_and_multiplier(sec2);
            float avg3, mul3 = get_sec_avg_and_multiplier(sec3);
            float avg4, mul4 = get_sec_avg_and_multiplier(sec4); 
            return avg1, mul1, avg2, mul2, avg3, mul3, avg4, mul4;
        }

        bool boundaries(int value) {
            // check if the value is within the module_info.min_us_range and module_info.max_us_range
            if (value > module_info.min_us_range && value < module_info.max_us_range) {
                return true;
            } else {
                return false;
            }
        }

        float decide_move(float sec1, float sec2, float sec3, float sec4) {
            if (boundaries(sec1) && boundaries(sec2) && boundaries(sec3) && boundaries(sec4)) {
                // move toward the sector with the highest average (most space)
                if (sec1 > sec2 && sec1 > sec3 && sec1 > sec4) {
                    // move forward
                    return 1;
                } else if (sec2 > sec1 && sec2 > sec3 && sec2 > sec4) {
                    // move right
                    return 2;
                } else if (sec3 > sec1 && sec3 > sec2 && sec3 > sec4) {
                    // move backward
                    return 3;
                } else if (sec4 > sec1 && sec4 > sec2 && sec4 > sec3) {
                    // move left
                    return 4;
                } else {
                    // stop
                    return 0;
                }
            } else if (boundaries(sec1) && boundaries(sec2) && boundaries(sec3) && !boundaries(sec4)) {
                // sec4 is out of range
                // move right
                return 2;
            } else if (boundaries(sec1) && boundaries(sec2) && !boundaries(sec3) && boundaries(sec4)) {
                // sec3 is out of range
                // move forward
                return 1;
            } else if (boundaries(sec1) && !boundaries(sec2) && boundaries(sec3) && boundaries(sec4)) {
                // sec2 is out of range
                // move left
                return 3;
            } else if (!boundaries(sec1) && boundaries(sec2) && boundaries(sec3) && boundaries(sec4)) {
                // sec1 is out of range
                // move backward
                return 4;


            }
        }

        float quick_init () {
          if (us_s1.read() >= 160) {
                float sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul = swivel(1);
                return sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul;
            } else {
                float sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul = swivel(-1);
                return sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul;
            }
        }

        float exec () {

            // get position of servo to swivel to empty space
            // swivel the servo & get the average of each sector
            
            float sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul = quick_init();

            // decide which direction to move based on the averages
            int move = decide_move(sec1, sec2, sec3, sec4);
            
            // return the direction, and the corresponding multiplier
            if (move == 1) {
                return 1, sec1_mul;
            } else if (move == 2) {
                return 2, sec2_mul;
            } else if (move == 3) {
                return 3, sec3_mul;
            } else if (move == 4) {
                return 4, sec4_mul;
            } else {
                return 0, 0;
            }
        }
};

class Headless_Pre_Defined_Moves {
    public:
        // This might change after trial and error, but...

        //servo1 is front right
        //servo2 is back right
        //servo3 is back left
        //servo4 is front left

        //spin specified motors 10% faster to move in a direction
        float movement_multiplier = 1.1;
        
        void forward(float speed) {
            // move forward
            // back motors spin faster
            servo1.write(speed);
            servo2.write(speed * movement_multiplier);
            servo3.write(speed * movement_multiplier);
            servo4.write(speed);
        }

        void backward(float speed) {
            // move backward
            // front motors spin faster
            servo1.write(speed * movement_multiplier);
            servo2.write(speed);
            servo3.write(speed);
            servo4.write(speed * movement_multiplier);
        }

        void left(float speed) {
            // move left
            // right motors spin faster
            servo1.write(speed * movement_multiplier);
            servo2.write(speed * movement_multiplier);
            servo3.write(speed);
            servo4.write(speed);
        }

        void right(float speed) {
            // move right
            // left motors spin faster
            servo1.write(speed);
            servo2.write(speed);
            servo3.write(speed * movement_multiplier);
            servo4.write(speed * movement_multiplier);
        }

        void hover() {
            // hover
            // write half PWM speed
            servo1.write(module_info.half_speed);
            servo2.write(module_info.half_speed);
            servo3.write(module_info.half_speed);
            servo4.write(module_info.half_speed);
        }
};

//========================================================================================================================//
//                                                   Setup                                                                //
//========================================================================================================================//


void setup() {
  IMUFuncs imuf;
  MotorFuncs motorfuncs;

  Serial.begin(usrVARS.baud); //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 
  pinMode(pinout.m1Pin, OUTPUT);
  pinMode(pinout.m2Pin, OUTPUT);
  pinMode(pinout.m3Pin, OUTPUT);
  pinMode(pinout.m4Pin, OUTPUT);
  pinMode(pinout.m5Pin, OUTPUT);
  pinMode(pinout.m6Pin, OUTPUT);

  // Regular Motors
  servo1.attach(pinout.servo1Pin, module_info.min_PWM, module_info.max_PWM); //Pin, min PWM value, max PWM value
  servo2.attach(pinout.servo2Pin, module_info.min_PWM, module_info.max_PWM);
  servo3.attach(pinout.servo3Pin, module_info.min_PWM, module_info.max_PWM);
  servo4.attach(pinout.servo4Pin, module_info.min_PWM, module_info.max_PWM);
  servo5.attach(pinout.servo5Pin, module_info.min_PWM, module_info.max_PWM);
  servo6.attach(pinout.servo6Pin, module_info.min_PWM, module_info.max_PWM);
  servo7.attach(pinout.servo7Pin, module_info.min_PWM, module_info.max_PWM);

  // Ultrasonic Sensor Servos
  us_s1.attach(pinout.us_s1);
  us_s2.attach(pinout.us_s2);
  us_s3.attach(pinout.us_s3);
  us_s4.attach(pinout.us_s4);

  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup(ch1Pin, ch2Pin, ch3Pin, ch4Pin, ch5Pin, ch6Pin, PPM_Pin);
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = usrVARS.failsafe_struct.channel_1_fs;
  channel_2_pwm = usrVARS.failsafe_struct.channel_2_fs;
  channel_3_pwm = usrVARS.failsafe_struct.channel_3_fs;
  channel_4_pwm = usrVARS.failsafe_struct.channel_4_fs;
  channel_5_pwm = usrVARS.failsafe_struct.channel_5_fs;
  channel_6_pwm = usrVARS.failsafe_struct.channel_6_fs;

  //Initialize IMU communication
  imuf.IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //imuf.calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  
  delay(5);

  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //Arm OneShot125 motors
  regVARS.mixer.m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
  regVARS.mixer.m2_command_PWM = 125;
  regVARS.mixer.m3_command_PWM = 125;
  regVARS.mixer.m4_command_PWM = 125;
  regVARS.mixer.m5_command_PWM = 125;
  regVARS.mixer.m6_command_PWM = 125;
  motorfuncs.armMotors(); //Loop over commandMotors() until ESCs happily arm

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}


//========================================================================================================================//
//                                                    Main                                                                //
//========================================================================================================================//


void loop() {
  
  SetupFuncs setup;
  IMUFuncs imuf;
  CtrlFuncs ctrlfuncs;
  ReadRadio readradio;
  MotorFuncs motorfuncs;

  Headless_Misc hm;
  Headless_Sentry hs;
  Headless_Pre_Defined_Moves hpd;
  
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  setup.loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  //printRadioData(current_time, print_counter, channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm);     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState(current_time, print_counter, regVARS.nml_des_state.thro_des, regVARS.nml_des_state.roll_des, regVARS.nml_des_state.pitch_des, regVARS.nml_des_state.yaw_des);  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  printGyroData(current_time, print_counter, regVARS.IMU.GyroX, regVARS.IMU.GyroY, regVARS.IMU.GyroZ);      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData(current_time, print_counter, regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ);     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printMagData(current_time, print_counter, regVARS.IMU.MagX, regVARS.IMU.MagY, regVARS.IMU.MagZ);       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw(current_time, print_counter, regVARS.IMU.roll_IMU, regVARS.IMU.pitch_IMU, regVARS.IMU.yaw_IMU);  //Prints roll, pitch, and yaw angles in degrees from imuf.Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput(current_time, print_counter, regVARS.ctrl.roll_PID, regVARS.ctrl.pitch_PID, regVARS.ctrl.yaw_PID);     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands(current_time, print_counter, regVARS.mixer.m1_command_PWM, regVARS.mixer.m2_command_PWM, regVARS.mixer.m3_command_PWM, regVARS.mixer.m4_command_PWM, regVARS.mixer.m5_command_PWM, regVARS.mixer.m6_command_PWM); //Prints the values being written to the motors (expected: 120 to 250)
  //printServoCommands(current_time, print_counter, regVARS.mixer.s1_command_PWM, regVARS.mixer.s2_command_PWM, regVARS.mixer.s3_command_PWM, regVARS.mixer.s4_command_PWM, regVARS.mixer.s5_command_PWM, regVARS.mixer.s6_command_PWM, regVARS.mixer.s7_command_PWM); //Prints the values being written to the servos (expected: 0 to 180)
  //printLoopRate(current_time, print_counter, dt);      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

  //Get vehicle state
  imuf.getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  imuf.Madgwick(regVARS.IMU.GyroX, -regVARS.IMU.GyroY, -regVARS.IMU.GyroZ, -regVARS.IMU.AccX, regVARS.IMU.AccY, regVARS.IMU.AccZ, regVARS.IMU.MagY, -regVARS.IMU.MagX, regVARS.IMU.MagZ, dt); //Updates regVARS.IMU.roll_IMU, regVARS.IMU.pitch_IMU, and regVARS.IMU.yaw_IMU angle estimates (degrees)

  //Compute desired state
  readradio.getDesState(); //Convert raw commands to normalized values based on saturated control limits
  
  //PID Controller - SELECT ONE:
  ctrlfuncs.controlANGLE(); //Stabilize on angle setpoint
  //ctrlfuncs.controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  //controlRATE(); //Stabilize on rate setpoint

  //Actuator mixing and scaling to PWM values
  ctrlfuncs.controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  readradio.scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  readradio.throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  motorfuncs.commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(regVARS.mixer.s1_command_PWM); //Writes PWM value to servo object
  servo2.write(regVARS.mixer.s2_command_PWM);
  servo3.write(regVARS.mixer.s3_command_PWM);
  servo4.write(regVARS.mixer.s4_command_PWM);
  servo5.write(regVARS.mixer.s5_command_PWM);
  servo6.write(regVARS.mixer.s6_command_PWM);
  servo7.write(regVARS.mixer.s7_command_PWM);
    
  //Get vehicle commands for next loop iteration
  readradio.getCommands(); //Pulls current available radio commands
  readradio.failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  setup.loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}