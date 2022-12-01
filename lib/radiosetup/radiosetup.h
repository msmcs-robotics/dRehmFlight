#ifndef radiosetup_h
#define radiosetup_h

#include <Arduino.h>

extern const int ch1Pin; //throttle
extern const int ch2Pin; //ail
extern const int ch3Pin; //ele
extern const int ch4Pin; //rudd
extern const int ch5Pin; //gear (throttle cut)
extern const int ch6Pin; //aux1 (free aux channel)
extern const int PPM_Pin;

unsigned long getRadioPWM(int ch_num);

void serialEvent3(void);

void getPPM();

void getCh1();

void getCh2();

void getCh3();

void getCh4();

void getCh5();

void getCh6();

void radioSetup(int ch1Pin, int ch2Pin, int ch3Pin, int ch4Pin, int ch5Pin, int ch6Pin, int PPM_pin);

#endif