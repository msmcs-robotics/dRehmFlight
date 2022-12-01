#ifndef print_funcs_h
#define print_funcs_h

#include <Arduino.h>

extern float dt;
extern unsigned long current_time, prev_time;
extern unsigned long print_counter, serial_counter;
extern unsigned long blink_counter, blink_delay;
extern bool blinkAlternate;

extern int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;
extern int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float MagX, MagY, MagZ;
extern float AccX, AccY, AccZ;
extern float GyroX, GyroY, GyroZ;
extern float thro_des, roll_des, pitch_des, yaw_des;
extern unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;

void printRadioData(unsigned long current_time, unsigned long print_counter, unsigned long channel_1_pwm, unsigned long channel_2_pwm, unsigned long channel_3_pwm, unsigned long channel_4_pwm, unsigned long channel_5_pwm, unsigned long channel_6_pwm);

void printDesiredState(unsigned long current_time, unsigned long print_counter, float thro_des, float roll_des, float pitch_des, float yaw_des);

void printGyroData(unsigned long current_time, unsigned long print_counter, float GyroX, float GyroY, float GyroZ);

void printAccelData(unsigned long current_time, unsigned long print_counter, float AccX, float AccY, float AccZ);

void printMagData(unsigned long current_time, unsigned long print_counter, float MagX, float MagY, float MagZ);

void printRollPitchYaw(unsigned long current_time, unsigned long print_counter, float roll_IMU, float pitch_IMU, float yaw_IMU);

void printPIDoutput(unsigned long current_time, unsigned long print_counter, float roll_PID, float pitch_PID, float yaw_PID);

void printMotorCommands(unsigned long current_time, unsigned long print_counter, int m1_command_PWM, int m2_command_PWM, int m3_command_PWM, int m4_command_PWM, int m5_command_PWM, int m6_command_PWM);

void printServoCommands(unsigned long current_time, unsigned long print_counter, int s1_command_PWM, int s2_command_PWM, int s3_command_PWM, int s4_command_PWM, int s5_command_PWM, int s6_command_PWM, int s7_command_PWM);

void printLoopRate(unsigned long current_time, unsigned long print_counter, float dt);

#endif