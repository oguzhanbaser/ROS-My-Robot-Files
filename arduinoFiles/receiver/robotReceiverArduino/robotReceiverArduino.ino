/*
 Name:		robotReceiverArduino.ino
 Created:	9/3/2016 9:40:50 PM
 Author:	baser
*/

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/servo_control.h>
#include <rosserial_arduino/led_control.h>
#include <rosserial_arduino/constant.h>

#include "motor.h"

//neopixel
#define PIN 26
#define NUMPIXELS      16

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

Servo servoHor, servoVer;

class NewHardware : public ArduinoHardware
{
public:
	NewHardware() : ArduinoHardware(&Serial3, 57600) {};
};

ros::NodeHandle_<NewHardware>  nh;

double measureTemp(int pin, int numSamples = 3);
double measureVoltage(int pin, int numSamples = 3);

int blueVal = 0, redVal = 0, greenVal = 0;
int MAXHIZ = 170;

motor solMotor(3, 4, 5);
motor sagMotor(9, 7, 8);

#define STDNBY 6
#define LEDPIN 26
#define ONBOARD_LED 13

#define DEF_SERVO_HOR 90
#define DEF_SERVO_VER 65

unsigned long lastTime = 0, lastTime2 = 0;

#define blinkLed1 27
#define blinkLed2 29
#define blinkTimes 3

bool blinkStatusLed1 = false;
bool blinkStatusLed2 = true;

bool manuelControl = true, ledStatus = true;
bool redStatus = false, greenStatus = false, blueStatus = false;
int statServo = 0;

std_msgs::String str_msg;
std_msgs::String str_msgDebug;

rosserial_arduino::servo_control servo_msg;

ros::Publisher pub_temp("sensors", &str_msg);
ros::Publisher pub_debug("debug", &str_msgDebug);
ros::Publisher pub_servo_feedBack("servo_feedBack", &servo_msg);


void controlMotors(const std_msgs::Int16& cmd_msg) {
	int hiz = cmd_msg.data;

	if (hiz > MAXHIZ) hiz = MAXHIZ;
	if (hiz < -MAXHIZ) hiz = -MAXHIZ;

	//autoControl for opencv
	if (!manuelControl)
	{
		digitalWrite(ONBOARD_LED, HIGH);
		digitalWrite(STDNBY, HIGH);
		sagMotor.duzAyarla();
		solMotor.duzAyarla();

		if (hiz > 0)
		{
			sagMotor.hizAyarla(MAXHIZ - hiz);
			solMotor.hizAyarla(MAXHIZ);
		}
		else {
			solMotor.hizAyarla(MAXHIZ + hiz);
			sagMotor.hizAyarla(MAXHIZ);
		}
	}
	else {
		digitalWrite(ONBOARD_LED, LOW);
	}

	//1 sn boyunca data okunmamýþsa motoru durdur
	lastTime = millis();
}
//int valX = 0, valY = 0;

void servoControl(const rosserial_arduino::servo_control& cmd_msg)
{
	//servo switch i aktifken web kontrolü aktif
	if (statServo)
	{
		servo_msg = cmd_msg;

		servoHor.write(servo_msg.angle_x);
		servoVer.write(servo_msg.angle_y);
		delay(15);
	}
}

ros::Subscriber<std_msgs::Int16> sub("robotControl", controlMotors);
ros::Subscriber<rosserial_arduino::servo_control> subServo("servoControl", servoControl);

void dataRead(HardwareSerial *ser)
{
	char c = ser->read();
	int val1, val2, newMaxHiz;
	int valX, valY;


	switch (c)
	{

	//ilk karakter okunmuþsa parse baþla
	case '#':
		manuelControl = true;

		val1 = ser->parseInt();
		val2 = ser->parseInt();
		valX = ser->parseInt();
		valY = ser->parseInt();
		statServo = ser->parseInt();
		newMaxHiz = ser->parseInt();

		if (newMaxHiz != MAXHIZ)
		{
			MAXHIZ = newMaxHiz;
			EEPROM.write(0, MAXHIZ);
		}


		hizAyarla(val1 - 512, val2 - 512);
		//servo switch i aktif deðilken kumanda kontrolü aktif
		if (!statServo)
		{
			servo_msg.angle_x = valX;
			servo_msg.angle_y = valY;

			servoHor.write(servo_msg.angle_x);
			servoVer.write(servo_msg.angle_y);
			digitalWrite(ONBOARD_LED, LOW);
			delay(15);
		}
		break;
	case '&':
		val1 = ser->parseInt();

		manuelControl = false;
		if (!manuelControl)
		{
			sagMotor.dur();
			solMotor.dur();
		}
		break;
	}

	//1 sn boyunca data okunmamýþsa motoru durdur
	lastTime = millis();
}

void setup() {
	// put your setup code here, to run once:
	//ros 57600 baud rate ile baþlar tekrar ekleme!!!


	//servo baðlantýlarý
	servoHor.attach(10);    //yatay servo (x)
	servoVer.attach(11);    //dikey servo (y)

	servoHor.write(DEF_SERVO_HOR);
	servoVer.write(DEF_SERVO_VER);

	Serial.begin(9600);
	Serial1.begin(9600);
	Serial2.begin(9600);

	pinMode(STDNBY, OUTPUT);
	pinMode(LEDPIN, OUTPUT);
	pinMode(blinkLed1, OUTPUT);
	pinMode(blinkLed2, OUTPUT);
	pixels.begin();

	//EEPROM 0 MAXHIZ adresi
	MAXHIZ = EEPROM.read(0);

	nh.initNode();

	//subsriber ayarlarý
	nh.subscribe(sub);
	nh.subscribe(subServo);

	//publisher ayarlarý
	nh.advertise(pub_temp);
	nh.advertise(pub_debug);
	nh.advertise(pub_servo_feedBack);

	digitalWrite(STDNBY, HIGH);

	sagMotor.dur();
	solMotor.dur();

	pinMode(ONBOARD_LED, OUTPUT);
}

void loop()
{
	double tempature[2], voltage[2];

	tempature[0] = measureTemp(A8);
	tempature[1] = measureTemp(A9);

	voltage[0] = measureVoltage(A6);
	voltage[1] = measureVoltage(A7);

	String tempString = "#|" + String(tempature[0]) + "|"
		+ String(tempature[1]) + "|"
		+ String(voltage[0]) + "|"
		+ String(voltage[1]) + "|\n";

	char sendData[30];
	tempString.toCharArray(sendData, 30);

	str_msg.data = sendData;
	if (millis() - lastTime2 > 1000)
	{
		pub_temp.publish(&str_msg);
		//web control moduna girmiþsse on board led i yak söndür
		if (statServo)
		{
			digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
		}
		lastTime2 = millis();
	}

	if (Serial1.available())
	{
		dataRead(&Serial1);
	}

	if (manuelControl)
	{
		//autoControl = false;
		digitalWrite(LEDPIN, LOW);
		if (ledStatus)
		{
			//Serial.println("Manuel Control");
			animateLed(2);
			ledStatus = false;
			animateLed(1);
		}
	}
	else {
		//autoControl = true;
		servoHor.write(DEF_SERVO_HOR);
		servoVer.write(DEF_SERVO_VER);
		digitalWrite(LEDPIN, HIGH);
	}


	//1 sn boyunca data okunmamýþsa motoru durdur
	//millis ilgili yerlerde kaydedilmiþ
	if (millis() - lastTime > 1000)
	{
		sagMotor.dur();
		solMotor.dur();
	}

	nh.spinOnce();
	delay(1);
}

void hizAyarla(int val1, int val2)
{
	val1 = map(val1, -512, 512, -MAXHIZ, MAXHIZ);
	val2 = map(val2, 512, -512, -MAXHIZ, MAXHIZ);

	if (val1 > MAXHIZ) val1 = MAXHIZ;
	if (val1 < -MAXHIZ) val1 = -MAXHIZ;

	if (val2 > MAXHIZ) val2 = MAXHIZ;
	if (val2 < -MAXHIZ) val2 = -MAXHIZ;

	//hýz deðeri 70 den büyükken hareket ettir
	if (val2 < 70 && val2 > -70)
	{
		val2 = 0;
		digitalWrite(STDNBY, LOW);
		//sagMotor.dur();
		//solMotor.dur();
	}
	else {
		digitalWrite(STDNBY, HIGH);
	}
	
	//gürülütüyü sýfýrla
	if (val1 < 10 && val1 > -10) val1 = 0;


	if (val2 >= 0)
	{
		if (val1 > 0)
		{
			sagMotor.duzAyarla();
			solMotor.duzAyarla();
			sagMotor.hizAyarla((val2 - val1) > 0 ? (val2 - val1) : 0);
			solMotor.hizAyarla(val2 > 0 ? val2 : 0);
			//analogWrite(sol, val2);
			//analogWrite(sag, val2 - val1);
		}
		else {
			sagMotor.duzAyarla();
			solMotor.duzAyarla();
			sagMotor.hizAyarla(val2 > 0 ? val2 : 0);
			solMotor.hizAyarla((val2 + val1) > 0 ? (val2 + val1) : 0);
			//analogWrite(sag, val2);
			//analogWrite(sol, val2 + val1);
		}
	}
	else {
		val2 = -val2;

		if (val1 > 0)
		{
			sagMotor.geriAyarla();
			solMotor.geriAyarla();
			sagMotor.hizAyarla((val2 - val1) > 0 ? (val2 - val1) : 0);
			solMotor.hizAyarla(val2 > 0 ? val2 : 0);
			//analogWrite(sol, val2);
			//analogWrite(sag, val2 - val1);
		}
		else {
			sagMotor.geriAyarla();
			solMotor.geriAyarla();
			sagMotor.hizAyarla(val2 > 0 ? val2 : 0);
			solMotor.hizAyarla((val2 + val1) > 0 ? (val2 + val1) : 0);
			//analogWrite(sag, val2);
			//analogWrite(sol, val2 + val1);
		}
	}
}

//geliþtirlecek
void animateLed(int kindOf)
{
	switch (kindOf)
	{
	case 1:
		for (int i = 0; i < NUMPIXELS; i++) {
			pixels.setPixelColor(i, pixels.Color(255, 255, 255));
			pixels.setBrightness(50);
			pixels.show();
			//delay(500);
		}
		break;
	case 2:
		for (int i = 0; i < NUMPIXELS; i++) {
			pixels.setPixelColor(i, pixels.Color(0, 255, 255));
			pixels.show();
			delay(100);
		}
		break;

	}
}

//parametre olarak gelen deðeri debug topic e yazar
void debugPublisher(String _data)
{
	char debugArray[30];
	_data.toCharArray(debugArray, 30);
	str_msgDebug.data = debugArray;

	pub_debug.publish(&str_msgDebug);
}

double measureTemp(int pin, int numSamples)
{
	int  sum = 0;
	for (int i = 0; i < numSamples; i++)
	{
		sum += analogRead(pin);
	}
	return (sum / 3.0) * 0.48828125;
}

double measureVoltage(int pin, int numSamples)
{
	int sum = 0;
	for (int i = 0; i < numSamples; i++)
	{
		sum += analogRead(pin);
	}

	return (sum / 3.0) * 12.54 / 460.0;
}


