//#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/servo_control.h>
#include <rosserial_arduino/command.h>
#include "motor.h"

//motor driver standby pins
#define STDNBY 6

//motor encoder
#define FULLTOUR 1200
#define ENCODERLEFT_PIN 19
#define ENCODERRIGHT_PIN 18

//led pins
#define LEDPIN 26
#define ONBOARD_LED 13
#define PW_LED 12
#define blinkLed1 27
#define blinkLed2 29
#define blinkTimes 3

#define DEF_SERVO_HOR 90
#define DEF_SERVO_VER 65

//servos
#define USE_SERVOS

#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"

#define SERVO_HOR_PIN	10
#define SERVO_VER_PIN	11

#endif

//neopixel
#define PIN 26
#define NUMPIXELS      16
