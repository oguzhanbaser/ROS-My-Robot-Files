#if (ARDUINO >= 100)
#include <Arduino.h>
#else
//#include <WProgram.h>
#endif

//#define USE_SERVOS
//#define MAKE_TEST
#define MAKE_MOTOR_TEST 0
#define USE_MPU6050


//servo
#ifdef USE_SERVOS
#define SERVO_HOR_PIN	10
#define SERVO_VER_PIN	11

#define DEF_SERVO_HOR 90
#define DEF_SERVO_VER 65
#endif

//encoder
#define FORWARD true
#define BACKWARD false
#define LEFT_ENCODER_PIN 19
#define RIGHT_ENCODER_PIN 18

#define ENCODERINTERVAL 1000 / 60
#define MPUINTERVAL 10
#define FULLTOUR 1200
#define RADIUS	3.5
#define PERIMETER 2 * 3.14 * RADIUS / FULLTOUR//cm

//neopixel
#define PIN 26
#define NUMPIXELS      16

//led
//#define USE_LDR
#define blinkLed1 27
#define blinkLed2 29
#define blinkTimes 3
#define LEDPIN 26
#define ONBOARD_LED 13
#define PW_LED 12
#define PW_LED_WAIT 1000

//motor
#define STDNBY 6
#define MOTOR_RIGHT_PWM 3
#define MOTOR_RIGHT_DIR1 5
#define MOTOR_RIGHT_DIR2 4
#define MOTOR_LEFT_PWM 9
#define MOTOR_LEFT_DIR1 8
#define MOTOR_LEFT_DIR2 7
#define MOTOR_LEFT_RIGHT_INV 1

//sensors
#define NUM_SAMPLES 3
#define DECIMAL_POINTS 3

//voltage sensor
#define RES1 1000000
#define RES2 220000
#define V_SENSOR_PIN A3

//current sensor
#define C_SENSOR_PIN A1
#define C_SENSOR_SAMPLE 10
#define ACSOFFSEET 2500
#define MVPERAMP 185

//temperature sensor
#define T_SENSOR1_PIN A2
#define T_SENSOR2_PIN A0

#ifdef USE_LDR
#define LDR_PIN A4
#endif

#ifdef USE_MPU6050
//#define MPU6050_YPR
#define MPU6050_Q
//mpu6050 interrupt pin
#define MPU6050_INT 10
#endif

/****
 * EEPROM Configuration
 *
 * */

//EEPROM Adress
#define MOTOR_ADRESS 0
#define LDR_ADRESS	 1

